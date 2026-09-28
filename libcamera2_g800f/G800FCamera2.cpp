/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Camera2 HAL for the Samsung SM-G800F (kminilte).
 *
 * The existing libcamera2 source in the tree is for a different FIMC-IS
 * generation and cannot be used directly on Exynos3470.  This file
 * instantiates a camera2 v2.0 device that uses:
 *   - /dev/video100+ FIMC-IS V4L2 nodes
 *   - camera2_shot_ext with camera2_node_group (kernel ABI)
 *   - the G800FExynosCameraNode / G800FExynosCameraBuffer wrappers
 *     (driven by G800FPipeEngine, the pipeline)
 *

 */

#define LOG_TAG "G800FCamera2"
#include <log/log.h>

#include <cstdint>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include <hardware/camera3.h>
#include <hardware/camera_common.h>
#include <hardware/hardware.h>
#include <system/camera_metadata.h>
#include <system/graphics.h>

#include "G800FCamera2Device.h"

// Forward declaration of the HAL module info symbol (defined in the
// extern "C" block below).  We need this visible inside namespace android
// so that g800f_init() can patch the methods pointer as a workaround
// for the Bionic linker not applying R_ARM_RELATIVE relocations in .data.
extern "C" camera_module_t HMI;

namespace android {

// Expose both cameras: ID 0 = rear (S5K4H5), ID 1 = front (S5K6B2)
static int g_num_cameras = 2;

// Forward declaration — the actual definition with initializer is later.
static hw_module_methods_t g800f_camera_module_methods;

static int g800f_get_number_of_cameras(void)
{
    // Patch HMI.common.methods as a side effect.  The framework calls
    // get_number_of_cameras() before open(), so this ensures the methods
    // pointer is valid even if the constructor's write was lost (e.g.
    // due to COW/linker namespace issues).
    if (::HMI.common.methods != &g800f_camera_module_methods) {
        ALOGW("g800f_get_number_of_cameras: patching methods (%p -> %p)",
              ::HMI.common.methods, &g800f_camera_module_methods);
        ::HMI.common.methods = &g800f_camera_module_methods;
    }
    return g_num_cameras;
}

static int g800f_set_callbacks(__attribute__((unused)) const camera_module_callbacks_t *callbacks)
{
    return 0;
}

static void g800f_get_vendor_tag_ops(__attribute__((unused)) vendor_tag_ops_t* ops)
{
    // no vendor tags
}

// Forward declaration — defined after g800f_init.
static int g800f_set_torch_mode(const char *camera_id, bool enabled);

// g800f_init() is defined later, after g800f_camera_module_methods and
// g800f_get_camera_info, because it references them to patch the HMI
// struct as a workaround for the Bionic linker relocation bug.

static camera_metadata_t* s_camera_info[2] = { NULL, NULL };

static camera_metadata_t* g800f_build_camera_info(int camera_id)
{
    // 16384 bytes was too small and caused add_camera_metadata_entry to fail
    // for later entries, which made this function return NULL.  The camera
    // provider helper (CameraModule::getCameraInfo) on older LineageOS builds
    // does not check the return value of get_camera_info before calling
    // CameraMetadata::append(static_camera_characteristics), and append() has
    // no NULL check, so a NULL characteristics pointer crashes the provider
    // service with a SIGSEGV in get_camera_metadata_entry_count(NULL) and
    // causes a bootloop.  Use a much larger buffer to avoid this.
    camera_metadata_t* m = allocate_camera_metadata(500, 65536);
    if (m == NULL) {
        ALOGE("g800f_build_camera_info(%d): allocate_camera_metadata failed", camera_id);
        return NULL;
    }

    int32_t facing = (camera_id == 0) ? ANDROID_LENS_FACING_BACK : ANDROID_LENS_FACING_FRONT;
    int32_t orientation = (camera_id == 0) ? 90 : 270;
    uint8_t level = ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED;

    // Sensor geometry differs between rear (S5K4H5, 8MP) and front (S5K6B2, 2MP)
    // Rear:  full 3264x2448, active array 3264x2448
    // Front: full 1936x1090, active array 1920x1080
    int32_t activeArray[4];
    int32_t pixelArray[2];
    int32_t preCorrectionActiveArraySize[4];
    float physicalSize[2];
    float focalLengths[1];
    int32_t jpegMaxSize;
    if (camera_id == 0) {
        // Rear camera (S5K4H5, 8MP) — activeArraySize = full sensor
        activeArray[0] = 0; activeArray[1] = 0; activeArray[2] = 3264; activeArray[3] = 2448;
        pixelArray[0] = 3264; pixelArray[1] = 2448;
        preCorrectionActiveArraySize[0] = 0; preCorrectionActiveArraySize[1] = 0;
        preCorrectionActiveArraySize[2] = 3264; preCorrectionActiveArraySize[3] = 2448;
        physicalSize[0] = 3.6f; physicalSize[1] = 2.7f;
        focalLengths[0] = 4.0f;
        jpegMaxSize = 8 * 1024 * 1024;
    } else {
        // Front camera (S5K6B2, 2MP, 1936x1090)
        activeArray[0] = 0; activeArray[1] = 0; activeArray[2] = 1920; activeArray[3] = 1080;
        pixelArray[0] = 1920; pixelArray[1] = 1080;
        preCorrectionActiveArraySize[0] = 0; preCorrectionActiveArraySize[1] = 0;
        preCorrectionActiveArraySize[2] = 1920; preCorrectionActiveArraySize[3] = 1080;
        physicalSize[0] = 2.8f; physicalSize[1] = 1.6f;
        focalLengths[0] = 2.8f;
        jpegMaxSize = 3 * 1024 * 1024;
    }
    uint8_t timestampSource = ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN;
    uint8_t sceneModes[1] = { ANDROID_CONTROL_SCENE_MODE_DISABLED };
    // AE modes: rear camera supports flash (auto, always, redeye),
    // front camera has no flash so only OFF and ON.
    uint8_t aeModes[5];
    int numAeModes;
    if (camera_id == 0) {
        aeModes[0] = ANDROID_CONTROL_AE_MODE_OFF;
        aeModes[1] = ANDROID_CONTROL_AE_MODE_ON;
        aeModes[2] = ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH;
        aeModes[3] = ANDROID_CONTROL_AE_MODE_ON_ALWAYS_FLASH;
        aeModes[4] = ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH_REDEYE;
        numAeModes = 5;
    } else {
        aeModes[0] = ANDROID_CONTROL_AE_MODE_OFF;
        aeModes[1] = ANDROID_CONTROL_AE_MODE_ON;
        numAeModes = 2;
    }
    // Front camera has no autofocus (fixed focus).
    // Rear camera: advertise the full API1-usable set.  Camera2Client's
    // API1 compat layer derives supported focus modes from this list —
    // without AUTO/MACRO here, legacy apps can only pick
    // continuous-picture.
    uint8_t afModes[5];
    int numAfModes;
    if (camera_id == 0) {
        afModes[0] = ANDROID_CONTROL_AF_MODE_OFF;
        afModes[1] = ANDROID_CONTROL_AF_MODE_AUTO;
        afModes[2] = ANDROID_CONTROL_AF_MODE_MACRO;
        afModes[3] = ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE;
        afModes[4] = ANDROID_CONTROL_AF_MODE_CONTINUOUS_VIDEO;
        numAfModes = 5;
    } else {
        afModes[0] = ANDROID_CONTROL_AF_MODE_OFF;
        afModes[1] = ANDROID_CONTROL_AF_MODE_OFF;
        numAfModes = 2;
    }
    uint8_t awbModes[2] = { ANDROID_CONTROL_AWB_MODE_OFF, ANDROID_CONTROL_AWB_MODE_AUTO };
    int32_t maxRegions[3] = { 1, 0, 1 };
    int32_t aeCompensationRange[2] = { -4, 4 };
    camera_metadata_rational_t aeCompensationStep = { 1, 1 };
    int32_t aeTargetFpsRanges[6] = { 15, 30, 24, 24, 30, 30 };
    uint8_t aeAntibandingModes[1] = { ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF };
    uint8_t availableEffects[1] = { ANDROID_CONTROL_EFFECT_MODE_OFF };
    uint8_t availableControlModes[2] = { ANDROID_CONTROL_MODE_OFF, ANDROID_CONTROL_MODE_AUTO };
    uint8_t availableVideoStabilizationModes[1] = { ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF };
    uint8_t aeLockAvailable = 1;
    uint8_t awbLockAvailable = 1;
    // Stream configurations differ: rear supports 3264x2448 JPEG, front 1920x1080
    // YUV_420_888 is needed for CameraX ImageAnalysis (e.g. Aegis QR scanner).
    int32_t streamConfigs[52];
    int64_t minFrameDurations[52];
    int64_t stallDurations[24];
    size_t streamConfigsCount, minFrameDurationsCount, stallDurationsCount;
    if (camera_id == 0) {
        // Rear camera stream configs (8MP)
        int32_t sc[] = {
            HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 1920, 1080, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 1280, 720, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 960, 720, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 640, 480, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_YCbCr_420_888, 640, 480, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_BLOB, 3264, 2448, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_BLOB, 1920, 1080, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_BLOB, 1280, 720, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_BLOB, 960, 720, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_BLOB, 640, 480, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT
        };
        int64_t md[] = {
            (int64_t)HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 1920, 1080, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 1280, 720, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 960, 720, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 640, 480, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_YCbCr_420_888, 640, 480, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 3264, 2448, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 1920, 1080, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 1280, 720, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 960, 720, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 640, 480, 33333333LL
        };
        int64_t sd[] = {
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 3264, 2448, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 1920, 1080, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 1280, 720, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 960, 720, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 640, 480, 33333333LL
        };
        streamConfigsCount = sizeof(sc)/sizeof(sc[0]);
        minFrameDurationsCount = sizeof(md)/sizeof(md[0]);
        stallDurationsCount = sizeof(sd)/sizeof(sd[0]);
        memcpy(streamConfigs, sc, sizeof(sc));
        memcpy(minFrameDurations, md, sizeof(md));
        memcpy(stallDurations, sd, sizeof(sd));
    } else {
        // Front camera stream configs (2MP, max 1920x1080)
        int32_t sc[] = {
            HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 1280, 720, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 960, 720, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 640, 480, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_YCbCr_420_888, 640, 480, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_BLOB, 1920, 1080, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_BLOB, 1280, 720, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_BLOB, 960, 720, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_BLOB, 640, 480, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT,
            HAL_PIXEL_FORMAT_BLOB, 320, 240, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT
        };
        int64_t md[] = {
            (int64_t)HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 1280, 720, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 960, 720, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, 640, 480, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_YCbCr_420_888, 640, 480, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 1920, 1080, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 1280, 720, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 960, 720, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 640, 480, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 320, 240, 33333333LL
        };
        int64_t sd[] = {
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 1920, 1080, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 1280, 720, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 960, 720, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 640, 480, 33333333LL,
            (int64_t)HAL_PIXEL_FORMAT_BLOB, 320, 240, 33333333LL
        };
        streamConfigsCount = sizeof(sc)/sizeof(sc[0]);
        minFrameDurationsCount = sizeof(md)/sizeof(md[0]);
        stallDurationsCount = sizeof(sd)/sizeof(sd[0]);
        memcpy(streamConfigs, sc, sizeof(sc));
        memcpy(minFrameDurations, md, sizeof(md));
        memcpy(stallDurations, sd, sizeof(sd));
    }
    uint8_t croppingType = ANDROID_SCALER_CROPPING_TYPE_CENTER_ONLY;
    float maxDigitalZoom = 1.0f;
    // Flash available only on rear camera (G800F has RT5033 flash LED).
    // Front camera has no flash.
    uint8_t flashAvailable = (camera_id == 0) ? 1 : 0;
    float hyperfocalDistance = 0.31f;
    // Diopters (1/m).  MUST be > 0 on the rear camera: the API1 compat
    // layer (Camera2Client Parameters.cpp) treats 0 as "fixed-focus lens"
    // → apps see only FOCUS_MODE_FIXED, autoFocus() becomes a fake-success
    // no-op and requests carry AF_MODE_OFF (QR scanner could never focus).
    // S5K4H5 + CML AF actuator: ~10 cm closest focus → 10.0 diopters.
    float minimumFocusDistance = (camera_id == 0) ? 10.0f : 0.0f;
    int32_t jpegThumbs[4] = { 0, 0, 640, 480 };
    uint8_t faceDetectModes[1] = { ANDROID_STATISTICS_FACE_DETECT_MODE_OFF };
    int32_t maxFaceCount = 0;
    int32_t syncLatency = ANDROID_SYNC_MAX_LATENCY_UNKNOWN;
    uint8_t availableAberrationModes[1] = { ANDROID_COLOR_CORRECTION_ABERRATION_MODE_OFF };
    uint8_t availableShadingModes[1] = { ANDROID_SHADING_MODE_OFF };
    uint8_t capabilities[1] = { ANDROID_REQUEST_AVAILABLE_CAPABILITIES_BACKWARD_COMPATIBLE };
    // maxNumOutputStreams: [IMPLEMENTATION_DEFINED, JPEG, YUV/RAW, ???]
    // Allow 2 preview, 1 JPEG, 1 YUV_420_888 (for CameraX ImageAnalysis)
    int32_t maxNumOutputStreams[4] = { 2, 1, 1, 0 };
    uint8_t pipelineDepth = 1;
    int32_t rational0[2] __attribute__((unused)) = { 0, 1 };
    int32_t zero __attribute__((unused)) = 0;

    int32_t sessionKeys[] = {};

    int32_t requestKeys[] = {
        ANDROID_COLOR_CORRECTION_ABERRATION_MODE,
        ANDROID_CONTROL_AE_ANTIBANDING_MODE,
        ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION,
        ANDROID_CONTROL_AE_LOCK,
        ANDROID_CONTROL_AE_MODE,
        ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
        ANDROID_CONTROL_AF_MODE,
        ANDROID_CONTROL_AF_TRIGGER,
        ANDROID_CONTROL_AWB_LOCK,
        ANDROID_CONTROL_AWB_MODE,
        ANDROID_CONTROL_CAPTURE_INTENT,
        ANDROID_CONTROL_EFFECT_MODE,
        ANDROID_CONTROL_MODE,
        ANDROID_CONTROL_SCENE_MODE,
        ANDROID_CONTROL_VIDEO_STABILIZATION_MODE,
        ANDROID_FLASH_MODE,
        ANDROID_JPEG_GPS_COORDINATES,
        ANDROID_JPEG_GPS_PROCESSING_METHOD,
        ANDROID_JPEG_GPS_TIMESTAMP,
        ANDROID_JPEG_ORIENTATION,
        ANDROID_JPEG_QUALITY,
        ANDROID_JPEG_THUMBNAIL_QUALITY,
        ANDROID_JPEG_THUMBNAIL_SIZE,
        ANDROID_LENS_FOCAL_LENGTH,
        ANDROID_SCALER_CROP_REGION,
        ANDROID_STATISTICS_FACE_DETECT_MODE,
    };

    int32_t resultKeys[] = {
        ANDROID_CONTROL_AF_STATE,
        ANDROID_REQUEST_PIPELINE_DEPTH,
        ANDROID_SENSOR_TIMESTAMP,
        ANDROID_STATISTICS_FACE_DETECT_MODE,
        ANDROID_STATISTICS_FACE_IDS,
        ANDROID_STATISTICS_FACE_LANDMARKS,
        ANDROID_STATISTICS_FACE_RECTANGLES,
        ANDROID_STATISTICS_FACE_SCORES,
        ANDROID_SYNC_FRAME_NUMBER,
    };

    int32_t characteristicsKeys[] = {
        ANDROID_COLOR_CORRECTION_AVAILABLE_ABERRATION_MODES,
        ANDROID_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES,
        ANDROID_CONTROL_AE_AVAILABLE_MODES,
        ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
        ANDROID_CONTROL_AE_COMPENSATION_RANGE,
        ANDROID_CONTROL_AE_COMPENSATION_STEP,
        ANDROID_CONTROL_AE_LOCK_AVAILABLE,
        ANDROID_CONTROL_AF_AVAILABLE_MODES,
        ANDROID_CONTROL_AVAILABLE_EFFECTS,
        ANDROID_CONTROL_AVAILABLE_MODES,
        ANDROID_CONTROL_AVAILABLE_SCENE_MODES,
        ANDROID_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES,
        ANDROID_CONTROL_AWB_AVAILABLE_MODES,
        ANDROID_CONTROL_AWB_LOCK_AVAILABLE,
        ANDROID_CONTROL_MAX_REGIONS,
        ANDROID_FLASH_INFO_AVAILABLE,
        ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL,
        ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES,
        ANDROID_JPEG_MAX_SIZE,
        ANDROID_LENS_FACING,
        ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS,
        ANDROID_LENS_INFO_HYPERFOCAL_DISTANCE,
        ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE,
        ANDROID_REQUEST_AVAILABLE_CAPABILITIES,
        ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS,
        ANDROID_REQUEST_AVAILABLE_RESULT_KEYS,
        ANDROID_REQUEST_AVAILABLE_SESSION_KEYS,
        ANDROID_REQUEST_MAX_NUM_OUTPUT_STREAMS,
        ANDROID_REQUEST_PIPELINE_MAX_DEPTH,
        ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM,
        ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS,
        ANDROID_SCALER_AVAILABLE_STALL_DURATIONS,
        ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
        ANDROID_SCALER_CROPPING_TYPE,
        ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
        ANDROID_SENSOR_INFO_PHYSICAL_SIZE,
        ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE,
        ANDROID_SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE,
        ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE,
        ANDROID_SENSOR_ORIENTATION,
        ANDROID_SHADING_AVAILABLE_MODES,
        ANDROID_STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES,
        ANDROID_STATISTICS_INFO_MAX_FACE_COUNT,
        ANDROID_SYNC_MAX_LATENCY,
    };

    if (add_camera_metadata_entry(m, ANDROID_COLOR_CORRECTION_AVAILABLE_ABERRATION_MODES, availableAberrationModes, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES, aeAntibandingModes, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_AE_AVAILABLE_MODES, aeModes, numAeModes) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES, aeTargetFpsRanges, 6) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_AE_COMPENSATION_RANGE, aeCompensationRange, 2) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_AE_COMPENSATION_STEP, &aeCompensationStep, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_AE_LOCK_AVAILABLE, &aeLockAvailable, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_AF_AVAILABLE_MODES, afModes, numAfModes) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_AVAILABLE_EFFECTS, availableEffects, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_AVAILABLE_MODES, availableControlModes, 2) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_AVAILABLE_SCENE_MODES, sceneModes, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES, availableVideoStabilizationModes, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_AWB_AVAILABLE_MODES, awbModes, 2) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_AWB_LOCK_AVAILABLE, &awbLockAvailable, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_CONTROL_MAX_REGIONS, maxRegions, 3) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_FLASH_INFO_AVAILABLE, &flashAvailable, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL, &level, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES, jpegThumbs, 4) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_JPEG_MAX_SIZE, &jpegMaxSize, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_LENS_FACING, &facing, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS, focalLengths, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_LENS_INFO_HYPERFOCAL_DISTANCE, &hyperfocalDistance, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE, &minimumFocusDistance, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_REQUEST_AVAILABLE_CAPABILITIES, capabilities, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_REQUEST_AVAILABLE_CHARACTERISTICS_KEYS, characteristicsKeys, sizeof(characteristicsKeys)/sizeof(characteristicsKeys[0])) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS, requestKeys, sizeof(requestKeys)/sizeof(requestKeys[0])) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_REQUEST_AVAILABLE_RESULT_KEYS, resultKeys, sizeof(resultKeys)/sizeof(resultKeys[0])) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_REQUEST_AVAILABLE_SESSION_KEYS, sessionKeys, sizeof(sessionKeys)/sizeof(sessionKeys[0])) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_REQUEST_MAX_NUM_OUTPUT_STREAMS, maxNumOutputStreams, 4) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_REQUEST_PIPELINE_MAX_DEPTH, &pipelineDepth, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM, &maxDigitalZoom, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS, minFrameDurations, minFrameDurationsCount) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_SCALER_AVAILABLE_STALL_DURATIONS, stallDurations, stallDurationsCount) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, streamConfigs, streamConfigsCount) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_SCALER_CROPPING_TYPE, &croppingType, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE, activeArray, 4) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_SENSOR_INFO_PHYSICAL_SIZE, physicalSize, 2) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE, pixelArray, 2) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE, preCorrectionActiveArraySize, 4) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE, &timestampSource, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_SENSOR_ORIENTATION, &orientation, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_SHADING_AVAILABLE_MODES, availableShadingModes, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES, faceDetectModes, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_STATISTICS_INFO_MAX_FACE_COUNT, &maxFaceCount, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }
    if (add_camera_metadata_entry(m, ANDROID_SYNC_MAX_LATENCY, &syncLatency, 1) != 0) { ALOGE("g800f_build_camera_info(%d): add_camera_metadata_entry failed at line %d", camera_id, __LINE__); free_camera_metadata(m); return NULL; }

    ALOGI("g800f_build_camera_info(%d) succeeded", camera_id);
    return m;
}

// Minimal fallback metadata used when g800f_build_camera_info fails.  This
// must never be NULL because the camera provider helper calls
// CameraMetadata::append() on static_camera_characteristics without a NULL
// check, and a NULL pointer crashes the provider service (bootloop).
static camera_metadata_t* g800f_build_minimal_camera_info(int camera_id)
{
    camera_metadata_t* m = allocate_camera_metadata(16, 2048);
    if (m == NULL) {
        ALOGE("g800f_build_minimal_camera_info(%d): allocate failed", camera_id);
        return NULL;
    }
    int32_t facing = (camera_id == 0) ? ANDROID_LENS_FACING_BACK : ANDROID_LENS_FACING_FRONT;
    int32_t orientation = (camera_id == 0) ? 90 : 270;
    uint8_t level = ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED;
    int32_t activeArray[4] = { 0, 0, 3264, 2448 };
    int32_t pixelArray[2] = { 3264, 2448 };
    uint8_t capabilities[1] = { ANDROID_REQUEST_AVAILABLE_CAPABILITIES_BACKWARD_COMPATIBLE };

    add_camera_metadata_entry(m, ANDROID_LENS_FACING, &facing, 1);
    add_camera_metadata_entry(m, ANDROID_SENSOR_ORIENTATION, &orientation, 1);
    add_camera_metadata_entry(m, ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL, &level, 1);
    add_camera_metadata_entry(m, ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE, activeArray, 4);
    add_camera_metadata_entry(m, ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE, pixelArray, 2);
    add_camera_metadata_entry(m, ANDROID_REQUEST_AVAILABLE_CAPABILITIES, capabilities, 1);

    ALOGW("g800f_build_minimal_camera_info(%d): using fallback metadata", camera_id);
    return m;
}


static int g800f_get_camera_info(int camera_id, struct camera_info *info)
{
    ALOGI("g800f_get_camera_info(%d): enter, g_num_cameras=%d", camera_id, g_num_cameras);

    // Safety net: patch methods pointer in case get_number_of_cameras
    // wasn't called first or the patch was lost.
    if (::HMI.common.methods != &g800f_camera_module_methods) {
        ALOGW("g800f_get_camera_info: patching methods (%p -> %p)",
              ::HMI.common.methods, &g800f_camera_module_methods);
        ::HMI.common.methods = &g800f_camera_module_methods;
    }

    if (camera_id < 0 || camera_id >= g_num_cameras) {
        ALOGE("g800f_get_camera_info(%d): invalid camera_id (g_num_cameras=%d)", camera_id, g_num_cameras);
        return -EINVAL;
    }

    // Initialize all fields to safe defaults
    memset(info, 0, sizeof(*info));

    if (s_camera_info[camera_id] == NULL) {
        ALOGI("g800f_get_camera_info(%d): building full metadata", camera_id);
        s_camera_info[camera_id] = g800f_build_camera_info(camera_id);
        ALOGI("g800f_get_camera_info(%d): build_camera_info returned %p", camera_id, s_camera_info[camera_id]);
    }

    // If the full metadata build failed, fall back to minimal metadata.
    // Never return an error with NULL static_camera_characteristics: the
    // camera provider helper (CameraModule::getCameraInfo) on some
    // LineageOS versions ignores the return value and calls
    // CameraMetadata::append(NULL), which crashes the provider service.
    if (s_camera_info[camera_id] == NULL) {
        ALOGE("g800f_get_camera_info: full metadata build failed for camera %d, using fallback", camera_id);
        s_camera_info[camera_id] = g800f_build_minimal_camera_info(camera_id);
        ALOGI("g800f_get_camera_info(%d): minimal build returned %p", camera_id, s_camera_info[camera_id]);
    }

    if (s_camera_info[camera_id] == NULL) {
        // Last-resort: even the fallback failed.  Return error but set
        // characteristics to NULL explicitly so the provider logs an error
        // instead of crashing on an uninitialized pointer.
        ALOGE("g800f_get_camera_info: fallback metadata also failed for camera %d", camera_id);
        info->static_camera_characteristics = NULL;
        return -ENOMEM;
    }

    info->facing = (camera_id == 0) ? CAMERA_FACING_BACK : CAMERA_FACING_FRONT;
    info->orientation = (camera_id == 0) ? 90 : 270;
    info->device_version = CAMERA_DEVICE_API_VERSION_3_2;
    info->static_camera_characteristics = s_camera_info[camera_id];

    ALOGI("g800f_get_camera_info(%d): success, characteristics=%p, facing=%d, version=0x%x",
          camera_id, info->static_camera_characteristics, info->facing, info->device_version);

    return 0;
}

static int g800f_camera_device_open(const hw_module_t* module,
                                    const char* id,
                                    hw_device_t** device)
{
    int cameraId = (int)strtol(id, NULL, 10);
    if (cameraId < 0 || cameraId >= g_num_cameras)
        return -EINVAL;

    // Both cameras use the HAL3 (Camera2) device path
    G800FCamera2Device *cam = new G800FCamera2Device(cameraId, module);
    hw_device_t *dev = cam->open();
    if (dev == NULL) {
        delete cam;
        return -EINVAL;
    }
    *device = dev;

    return 0;
}


// g800f_camera_module_methods was forward-declared above.
// Initialize it here (after g800f_camera_device_open is defined).
static struct hw_module_methods_t_init {
    hw_module_methods_t_init() {
        g800f_camera_module_methods.open = g800f_camera_device_open;
    }
} g800f_camera_module_methods_init;

static int g800f_init(void)
{
    /*
     * Workaround: the Bionic dynamic linker on Exynos 3470 / Android 9
     * sometimes fails to apply R_ARM_RELATIVE relocations in the .data
     * section for the HAL_MODULE_INFO_SYM.methods field, leaving it as
     * an unrelocated offset (e.g. 0x0000f004) instead of a valid mapped
     * address.  This causes a SIGSEGV in CameraModule::open() when it
     * tries to dereference methods->open.
     *
     * We patch the pointer here, after the library has been loaded and
     * the base address is known.  init() is called by the framework
     * before any open() call, so this is early enough.
     */
    ::HMI.common.methods = &g800f_camera_module_methods;
    ::HMI.get_number_of_cameras = g800f_get_number_of_cameras;
    ::HMI.get_camera_info       = g800f_get_camera_info;
    ::HMI.set_callbacks         = g800f_set_callbacks;
    ::HMI.get_vendor_tag_ops    = g800f_get_vendor_tag_ops;
    ::HMI.set_torch_mode        = g800f_set_torch_mode;
    ::HMI.init                  = g800f_init;
    return 0;
}

/*
 * Torch mode control for the G800F rear camera.
 *
 * The G800F uses an RT5033 flash LED driver exposed via sysfs:
 *   /sys/class/camera/flash/rear_torch_flash  (torch + flash, same GPIO)
 *   /sys/class/camera/flash/rear_flash        (alias for rear_torch_flash)
 *
 * Writing '1' turns the torch on, '0' turns it off.
 * Only the rear camera (id=0) has a flash; front camera returns -ENOSYS.
 */
static int g800f_set_torch_mode(const char *camera_id, bool enabled)
{
    if (strcmp(camera_id, "0") != 0) {
        // Front camera has no flash
        return -ENOSYS;
    }

    int fd = open("/sys/class/camera/flash/rear_torch_flash", O_WRONLY);
    if (fd < 0) {
        ALOGE("g800f_set_torch_mode: open rear_torch_flash failed: %s",
              strerror(errno));
        return -errno;
    }

    const char *val = enabled ? "1" : "0";
    ssize_t n = write(fd, val, 1);
    close(fd);

    if (n != 1) {
        ALOGE("g800f_set_torch_mode: write '%s' failed: %s",
              val, strerror(errno));
        return -errno;
    }

    ALOGI("g800f_set_torch_mode: camera=%s torch=%s", camera_id,
          enabled ? "ON" : "OFF");
    return 0;
}

}  // namespace android

extern "C" {

/*
 * Constructor function — runs automatically when the shared library is
 * loaded by the dynamic linker, before any framework code can call HMI
 * function pointers.
 *
 * This is the ONLY reliable workaround for the Bionic linker bug where
 * R_ARM_RELATIVE relocations beyond RELCOUNT are not applied.  The init()
 * function pointer itself is at offset 0xf0a0 which is beyond RELCOUNT=40,
 * so the framework cannot call init() — we must patch HMI here instead.
 */
__attribute__((constructor))
static void g800f_hmi_patch(void) {
    ALOGI("g800f_hmi_patch: &HMI=%p, &methods_struct=%p",
          &HMI, &android::g800f_camera_module_methods);
    ALOGI("g800f_hmi_patch: BEFORE patch: methods=%p, init=%p, get_cameras=%p",
          HMI.common.methods, HMI.init, HMI.get_number_of_cameras);

    HMI.common.methods = &android::g800f_camera_module_methods;
    HMI.get_number_of_cameras = android::g800f_get_number_of_cameras;
    HMI.get_camera_info       = android::g800f_get_camera_info;
    HMI.set_callbacks         = android::g800f_set_callbacks;
    HMI.get_vendor_tag_ops    = android::g800f_get_vendor_tag_ops;
    HMI.open_legacy           = NULL;
    HMI.set_torch_mode        = android::g800f_set_torch_mode;
    HMI.init                  = android::g800f_init;

    // Verify the write stuck by re-reading
    ALOGI("g800f_hmi_patch: AFTER patch: methods=%p, init=%p, get_cameras=%p",
          HMI.common.methods, HMI.init, HMI.get_number_of_cameras);
}

camera_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag                 = HARDWARE_MODULE_TAG,
        .module_api_version  = CAMERA_MODULE_API_VERSION_2_4,
        .hal_api_version     = HARDWARE_HAL_API_VERSION,
        .id                  = CAMERA_HARDWARE_MODULE_ID,
        .name                = "G800F Camera2 HAL",
        .author              = "G800F HAL3",
        .methods             = &android::g800f_camera_module_methods,
        .dso                 = NULL,
        .reserved            = {0},
    },
    .get_number_of_cameras = android::g800f_get_number_of_cameras,
    .get_camera_info       = android::g800f_get_camera_info,
    .set_callbacks         = android::g800f_set_callbacks,
    .get_vendor_tag_ops    = android::g800f_get_vendor_tag_ops,
    .open_legacy           = NULL,
    .set_torch_mode        = android::g800f_set_torch_mode,
    .init                  = android::g800f_init,
    .reserved              = {0},
};

} // extern "C"
