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

#define LOG_TAG "G800FCamera2Device"
#include <log/log.h>

#include "G800FCamera2Device.h"

#include <hardware/gralloc.h>
#include <hardware/camera3.h>
#include <system/graphics.h>
#include <utils/Errors.h>
#include <utils/Timers.h>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <arm_neon.h>
#include <jpeglib.h>

// HW-JPEG-Encoder (ExynosJpegEncoder via /dev/video12)
#include <ExynosJpegApi.h>
#include <ion.h>
#include <sys/mman.h>

// Exynos 3470 gralloc private handle — needed to access the multi-plane
// buffer layout (fd1/base1/fd2/base2/stride/vstride/format) that the
// Samsung gralloc.exynos3 module uses.  The standard Android
// GraphicBufferMapper::lockYCbCr() does not work because this gralloc
// does not implement the lock_ycbgralloc HAL API.
#include "gralloc_priv.h"
#include "exynos_format.h"
#include "G800FFimcScaler.h"

namespace android {

namespace {
__attribute__((unused))
inline int clamp(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }

// Compute Y/Cb/Cr plane pointers from a locked gralloc buffer.
//
// After grmod->lock(), gralloc_map() has populated hnd->base/base1/base2
// (the mmap'd addresses for planes 0/1/2).  The standard gralloc0 lock() API
// only returns a single pointer via *vaddr (= hnd->base, the Y plane).  For
// multi-plane YUV formats the chroma plane pointers must be derived from the
// private_handle_t fields directly.
//
// This replaces the vaddr[1]/vaddr[2] writes that were previously in our
// modified gralloc_lock().  Those writes caused stack corruption in other
// apps (e.g. PipePipe) because the HIDL mapper path (Gralloc0HalImpl::lock)
// passes &data — a single void* on the stack — as the vaddr parameter.
// Writing vaddr[1]/vaddr[2] overwrites the caller's stack canary →
// __stack_chk_fail → SIGABRT.
//
// Layout rules (matching the Exynos 3470 gralloc):
//   Multi-plane (fd1 >= 0): plane 1 = hnd->base1, plane 2 = hnd->base2.
//   NV21 single-plane:      [Y: stride*vstride][VU: stride*(vstride/2)]
//   YV12 single-plane:      [Y: stride*vstride][Cb: cstride*vstride][Cr: cstride*vstride]
//     where cstride = ALIGN(stride/2, 16)
inline void grallocGetPlanePtrs(private_handle_t *hnd,
                                void *&yPlane,
                                void *&c1Plane,
                                void *&c2Plane)
{
    yPlane  = (void *)hnd->base;
    c1Plane = NULL;
    c2Plane = NULL;

    if (hnd->fd1 >= 0) {
        // Multi-plane: separate ION allocations for luma and chroma.
        c1Plane = (void *)hnd->base1;
        if (hnd->fd2 >= 0)
            c2Plane = (void *)hnd->base2;
    } else {
        // Single-plane: chroma is at an offset within hnd->base.
        size_t ySize = hnd->stride * hnd->vstride;
        uint8_t *base = (uint8_t *)hnd->base;
        if (hnd->format == HAL_PIXEL_FORMAT_YCrCb_420_SP) {
            // NV21: interleaved VU after Y plane
            c1Plane = (void *)(base + ySize);
        } else if (hnd->format == HAL_PIXEL_FORMAT_YCbCr_420_888 ||
                   hnd->format == HAL_PIXEL_FORMAT_YV12) {
            // YV12: [Y][Cb][Cr] — each chroma plane cstride*vstride
            size_t cStride = ALIGN(hnd->stride / 2, 16);
            c1Plane = (void *)(base + ySize);
            c2Plane = (void *)(base + ySize + cStride * hnd->vstride);
        }
    }
}
}

camera3_device_ops_t G800FCamera2Device::s_ops = {
    .initialize = G800FCamera2Device::s_initialize,
    .configure_streams = G800FCamera2Device::s_configure_streams,
    .register_stream_buffers = NULL,
    .construct_default_request_settings = G800FCamera2Device::s_construct_default_request_settings,
    .process_capture_request = G800FCamera2Device::s_process_capture_request,
    .get_metadata_vendor_tag_ops = NULL,
    .dump = G800FCamera2Device::s_dump,
    .flush = G800FCamera2Device::s_flush,
    .reserved = {0},
};

G800FCamera2Device::G800FCamera2Device(int cameraId, const hw_module_t* module)
    : m_captureWorkerRunning(false),
      m_analysisWorkerRunning(false),
      m_cameraId(cameraId),
      m_module(module),
      m_pipeEngine(NULL),
      m_fimcScaler(NULL),
      m_fimcVideoScaler(NULL),
      m_analysisScaler(NULL),
      m_stillScaler(NULL),
      m_stillScaleBuf(NULL),
      m_stillScaleBufW(0),
      m_stillScaleBufH(0),
      m_stillScaleSrcW(0),
      m_stillScaleSrcH(0),
      m_stillScaleSrcFmt(0),
      m_stillFimcFailed(false),
      m_fimcFailed(false),
      m_fimcVideoFailed(false),
      m_analysisFimcFailed(false),
      m_analysisBounce(NULL),
      m_analysisBounceW(0),
      m_analysisBounceH(0),
      m_callbackOps(NULL),
      m_previewStream(NULL),
      m_analysisStream(NULL),
      m_videoStream(NULL),
      m_captureStream(NULL),
      m_lastJpegSize(0),
      m_previewStarted(false),
      m_captureThreadRunning(false),
      m_flushing(false),
      m_workerBusy(false)
{
    memset(&m_hw, 0, sizeof(m_hw));
    m_hw.common.tag = HARDWARE_DEVICE_TAG;
    m_hw.common.version = CAMERA_DEVICE_API_VERSION_3_2;
    m_hw.common.module = const_cast<hw_module_t*>(module);
    m_hw.common.close = s_close;
    m_hw.ops = &s_ops;
    m_hw.priv = this;
}

G800FCamera2Device::~G800FCamera2Device()
{
    close();
}

// --- Global async pipe-engine cleanup ---
// The kernel-side exynos_v4l2_close() on the ISP node blocks ~2s (runtime PM
// / firmware shutdown).  close() hands the pipe engine + FIMC scalers to a
// detached pthread so s_close can return immediately, keeping the camera
// app's handler thread alive during rapid close→reopen cycles.
// open() on a new G800FCamera2Device instance waits on s_cleanupCond until
// the previous cleanup finishes, preventing V4L2 fd races.
static Mutex      s_cleanupLock;
static Condition  s_cleanupCond;
static bool       s_cleanupPending = false;

struct CleanupPayload {
    G800FPipeEngine *engine;
    G800FFimcScaler  *scaler;
    G800FFimcScaler  *vscaler;
    G800FFimcScaler  *ascaler;
    G800FFimcScaler  *sscaler;
    G800FExynosCameraBuffer *abounce;
    G800FExynosCameraBuffer *sbuf;
};

static void *cleanupThreadFunc(void *arg)
{
    CleanupPayload *p = static_cast<CleanupPayload*>(arg);
    ALOGI("asyncCleanup: starting async pipe engine deinit");
    if (p->engine) {
        p->engine->deinit();
        delete p->engine;
    }
    if (p->scaler)  delete p->scaler;
    if (p->vscaler) delete p->vscaler;
    if (p->ascaler) delete p->ascaler;
    if (p->sscaler) delete p->sscaler;
    if (p->abounce) delete p->abounce;
    if (p->sbuf)    delete p->sbuf;
    ALOGI("asyncCleanup: async pipe engine deinit complete");
    {
        Mutex::Autolock l(s_cleanupLock);
        s_cleanupPending = false;
        s_cleanupCond.broadcast();
    }
    delete p;
    return NULL;
}

hw_device_t* G800FCamera2Device::open()
{
    // Wait for any pending global async cleanup from a previous close()
    // to finish before creating a new pipe engine. This prevents fd races
    // on the same V4L2 nodes (especially the ISP node whose kernel close
    // takes ~2s). Each open() creates a new G800FCamera2Device instance,
    // so we can't use a per-instance member — the cleanup is global.
    {
        Mutex::Autolock l(s_cleanupLock);
        while (s_cleanupPending) {
            ALOGI("%s: waiting for previous async cleanup to finish", __FUNCTION__);
            s_cleanupCond.wait(s_cleanupLock);
        }
    }

    if (m_pipeEngine) {
        ALOGE("%s: already open", __FUNCTION__);
        return NULL;
    }

    // Turn off standalone torch before the ISP takes control.
    // If the flashlight is active via Quick Settings (rear_torch_flash='1'),
    // the RT5033 drives the LED directly via GPIO. When opening the camera
    // device, the ISP must take over the flash pins — otherwise there is a
    // conflict between GPIO torch and ISP-controlled flash.
    // Only relevant for the rear camera (cameraId==0, has flash).
    if (m_cameraId == 0) {
        int torchFd = ::open("/sys/class/camera/flash/rear_torch_flash", O_WRONLY);
        if (torchFd >= 0) {
            ssize_t n = ::write(torchFd, "0", 1);
            ::close(torchFd);
            if (n == 1) {
                ALOGI("%s: standalone torch turned off (rear_torch_flash=0)", __FUNCTION__);
            } else {
                ALOGW("%s: failed to turn off standalone torch: %s",
                      __FUNCTION__, strerror(errno));
            }
        }
        // ENOENT/EPERM is not treated as fatal — torch may already be off.
    }

    // Layer 1+2: PipeEngine with conformant pipes
    m_pipeEngine = new G800FPipeEngine(m_cameraId);
    if (m_pipeEngine == NULL)
        return NULL;

    if (m_pipeEngine->init() != NO_ERROR) {
        ALOGE("%s: G800FPipeEngine init failed", __FUNCTION__);
        delete m_pipeEngine;
        m_pipeEngine = NULL;
        return NULL;
    }

    return &m_hw.common;
}

int G800FCamera2Device::close()
{
    ALOGI("%s: called (previewStarted=%d)", __FUNCTION__, (int)m_previewStarted.load());
    Mutex::Autolock l(m_lock);

    // Log: how many requests are still in the queues?
    {
        Mutex::Autolock ql(m_queueLock);
        ALOGI("%s: m_requestQueue=%d m_workerBusy=%d m_captureThreadRunning=%d",
              __FUNCTION__, (int)m_requestQueue.size(),
              (int)m_workerBusy, (int)m_captureThreadRunning);
    }
    {
        Mutex::Autolock cl(m_captureQueueLock);
        ALOGI("%s: m_captureQueue=%d m_captureWorkerRunning=%d",
              __FUNCTION__, (int)m_captureQueue.size(),
              (int)m_captureWorkerRunning);
    }

    // CRITICAL (sensor protection): Turn off flash FIRST, before anything
    // else is shut down. If the camera app crashes or is closed during an
    // active flash capture, the flash must turn off immediately, otherwise
    // sensor overheating/damage may occur.
    if (m_pipeEngine && m_pipeEngine->isFlashSequenceActive()) {
        ALOGW("%s: Flash still active (state=%s), forcing endFlashSequence()",
              __FUNCTION__,
              G800FPipeEngine::flashSeqStateName(m_pipeEngine->getFlashSeqState()));
        m_pipeEngine->endFlashSequence();  // sets aeflashMode=OFF + releaseFlash()
    }

    // Cancel AF: sets afMode=OFF in all shot_ext buffers.
    if (m_pipeEngine) {
        m_pipeEngine->cancelAutoFocus();
    }

    // Stop the capture thread first
    ALOGI("%s: stopping capture thread", __FUNCTION__);
    {
        Mutex::Autolock ql(m_queueLock);
        m_captureThreadRunning = false;
        m_flushing = true;
        m_queueCond.broadcast();
    }
    if (m_captureThread != NULL) {
        m_captureThread->requestExitAndWait();
        m_captureThread.clear();
    }
    // Stop the async capture worker thread
    ALOGI("%s: stopping capture worker thread", __FUNCTION__);
    {
        Mutex::Autolock cl(m_captureQueueLock);
        m_captureWorkerRunning = false;
        m_captureQueueCond.broadcast();
    }
    if (m_captureWorkerThread != NULL) {
        m_captureWorkerThread->requestExitAndWait();
        m_captureWorkerThread.clear();
    }
    // Stop the analysis/callback worker thread.  requestExitAndWait joins
    // the thread — an in-progress job finishes first, then the loop exits.
    // Leftover queued jobs are drained below with error results.
    ALOGI("%s: stopping analysis worker thread", __FUNCTION__);
    {
        Mutex::Autolock al(m_analysisQueueLock);
        m_analysisWorkerRunning = false;
        m_analysisQueueCond.broadcast();
    }
    if (m_analysisThread != NULL) {
        m_analysisThread->requestExitAndWait();
        m_analysisThread.clear();
    }
    // Drain analysis queue: return buffers with error status and release
    // the held preview frames back to SCP.  Must happen before the pipe
    // engine handoff below (jobs reference pipe-owned buffers).
    // Send results outside the lock — the framework may reenter the HAL.
    std::list<AnalysisJob> abortedAnalysis;
    {
        Mutex::Autolock al(m_analysisQueueLock);
        abortedAnalysis.swap(m_analysisQueue);
    }
    for (std::list<AnalysisJob>::iterator it = abortedAnalysis.begin();
         it != abortedAnalysis.end(); ++it) {
        errorAnalysisJob(*it);
        if (it->frame && m_pipeEngine) m_pipeEngine->releasePreviewFrame(it->frame);
    }
    ALOGI("%s: capture thread stopped, now deinit pipe engine", __FUNCTION__);
    // Drain any remaining queued requests (mark as error)
    {
        Mutex::Autolock ql(m_queueLock);
        for (auto it = m_requestQueue.begin(); it != m_requestQueue.end(); ++it) {
            sendErrorResult(it->frame_number, it->output_buffers, it->num_output_buffers);
            free(it->output_buffers);
            if (it->settings) free_camera_metadata(it->settings);
        }
        m_requestQueue.clear();
    }
    // Drain capture queue too
    {
        Mutex::Autolock cl(m_captureQueueLock);
        for (auto it = m_captureQueue.begin(); it != m_captureQueue.end(); ++it) {
            sendErrorResult(it->frame_number, it->output_buffers, it->num_output_buffers);
            free(it->output_buffers);
            if (it->settings) free_camera_metadata(it->settings);
        }
        m_captureQueue.clear();
    }
    if (m_pipeEngine) {
        // The pipe engine deinit includes a kernel-side exynos_v4l2_close()
        // on the ISP node that blocks ~2s (runtime PM / firmware shutdown).
        // Running it synchronously blocks the framework's disconnect path,
        // which can cause the camera app's handler thread to die during
        // rapid close→reopen cycles (e.g. Gallery → back to camera).
        // Instead, hand the pipe engine + FIMC scalers to a detached
        // pthread and return immediately. The next open() (which creates
        // a new G800FCamera2Device instance) waits on the global
        // s_cleanupCond until this cleanup finishes, preventing V4L2 fd
        // races on the same nodes.
        ALOGI("%s: starting async pipe engine cleanup", __FUNCTION__);
        CleanupPayload *payload = new CleanupPayload;
        payload->engine  = m_pipeEngine;
        payload->scaler  = m_fimcScaler;
        payload->vscaler = m_fimcVideoScaler;
        payload->ascaler = m_analysisScaler;
        payload->sscaler = m_stillScaler;
        payload->abounce = m_analysisBounce;
        payload->sbuf    = m_stillScaleBuf;
        m_pipeEngine      = NULL;
        m_fimcScaler      = NULL;
        m_fimcVideoScaler = NULL;
        m_analysisScaler  = NULL;
        m_stillScaler     = NULL;
        m_analysisBounce  = NULL;
        m_stillScaleBuf   = NULL;
        m_analysisBounceW = 0;
        m_analysisBounceH = 0;
        m_stillScaleBufW  = 0;
        m_stillScaleBufH  = 0;
        m_fimcFailed      = false;
        m_fimcVideoFailed = false;
        m_analysisFimcFailed = false;
        m_stillFimcFailed = false;
        {
            Mutex::Autolock l(s_cleanupLock);
            s_cleanupPending = true;
        }
        pthread_t tid;
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        if (pthread_create(&tid, &attr, cleanupThreadFunc, payload) != 0) {
            // Fallback: synchronous cleanup if thread creation fails
            ALOGE("%s: pthread_create failed, doing synchronous cleanup", __FUNCTION__);
            {
                Mutex::Autolock l(s_cleanupLock);
                s_cleanupPending = false;
            }
            payload->engine->deinit();
            delete payload->engine;
            if (payload->scaler)  delete payload->scaler;
            if (payload->vscaler) delete payload->vscaler;
            if (payload->ascaler) delete payload->ascaler;
            if (payload->sscaler) delete payload->sscaler;
            if (payload->abounce) delete payload->abounce;
            if (payload->sbuf)    delete payload->sbuf;
            delete payload;
        }
        pthread_attr_destroy(&attr);
    }
    // Turn off standalone torch: If an app set FLASH_MODE_TORCH via
    // FIMC-IS, endFlashSequence() only turns off the capture flash,
    // not the continuous torch. Write '0' to rear_torch_flash
    // to ensure the LED is actually off.
    // Only for the rear camera (cameraId==0).
    if (m_cameraId == 0) {
        int torchFd = ::open("/sys/class/camera/flash/rear_torch_flash", O_WRONLY);
        if (torchFd >= 0) {
            ssize_t n = ::write(torchFd, "0", 1);
            ::close(torchFd);
            if (n == 1) {
                ALOGI("%s: torch off (rear_torch_flash=0)", __FUNCTION__);
            } else {
                ALOGW("%s: failed to turn off torch: %s",
                      __FUNCTION__, strerror(errno));
            }
        }
    }

    m_callbackOps = NULL;
    m_previewStream = NULL;
    m_analysisStream = NULL;
    m_captureStream = NULL;
    return 0;
}

int G800FCamera2Device::initialize(const camera3_callback_ops_t *callback_ops)
{
    Mutex::Autolock l(m_lock);
    m_callbackOps = callback_ops;

    // Start the capture worker thread (preview dispatcher)
    m_captureThreadRunning = true;
    m_flushing = false;
    m_captureThread = new G800FCaptureThread(this);
    if (m_captureThread->run("G800FCap", PRIORITY_URGENT_DISPLAY) != NO_ERROR) {
        ALOGE("%s: failed to start capture thread", __FUNCTION__);
        m_captureThread.clear();
        m_captureThreadRunning = false;
        return NO_INIT;
    }
    // Start the async still-capture worker thread
    m_captureWorkerRunning = true;
    m_captureWorkerThread = new G800FCaptureWorkerThread(this);
    if (m_captureWorkerThread->run("G800FCapW", PRIORITY_DEFAULT) != NO_ERROR) {
        ALOGE("%s: failed to start capture worker thread", __FUNCTION__);
        m_captureWorkerThread.clear();
        m_captureWorkerRunning = false;
        return NO_INIT;
    }
    // Start the analysis/callback-stream worker thread
    m_analysisWorkerRunning = true;
    m_analysisThread = new G800FAnalysisThread(this);
    if (m_analysisThread->run("G800FAna", PRIORITY_DEFAULT) != NO_ERROR) {
        ALOGE("%s: failed to start analysis worker thread", __FUNCTION__);
        m_analysisThread.clear();
        m_analysisWorkerRunning = false;
        return NO_INIT;
    }
    ALOGI("%s: capture + worker + analysis threads started", __FUNCTION__);
    return 0;
}


static android_dataspace_t unMapLegacyDataSpace(android_dataspace_t ds)
{
    switch ((int)ds) {
        case 512:  return static_cast<android_dataspace_t>(138477568); // SRGB_LINEAR
        case 513:  return static_cast<android_dataspace_t>(142671872); // SRGB
        case 257:  return static_cast<android_dataspace_t>(146931712); // JFIF
        case 258:  return static_cast<android_dataspace_t>(281149440); // BT601_625
        case 259:  return static_cast<android_dataspace_t>(281280512); // BT601_525
        case 260:  return static_cast<android_dataspace_t>(281083904); // BT709
        default:   return ds;
    }
}

int G800FCamera2Device::configureStreams(camera3_stream_configuration_t *stream_list)
{
    if (stream_list == NULL || stream_list->num_streams == 0)
        return -EINVAL;

    ALOGI("%s: num_streams=%u operation_mode=%u", __FUNCTION__,
          stream_list->num_streams, stream_list->operation_mode);

    // Stream count check: maxNumOutputStreams = [2 preview, 1 JPEG, 1 YUV].
    // The framework should not request more than 4 output streams.
    if (stream_list->num_streams > 4) {
        ALOGE("%s: too many streams: %u (max 4)", __FUNCTION__,
              stream_list->num_streams);
        return -EINVAL;
    }

    // Dump all streams BEFORE modification (framework perspective).
    for (uint32_t i = 0; i < stream_list->num_streams; i++) {
        camera3_stream_t *s = stream_list->streams[i];
        if (s == NULL) {
            ALOGW("%s: stream[%u] is NULL", __FUNCTION__, i);
            continue;
        }
        ALOGI("%s: stream[%u] BEFORE: type=%d format=0x%x usage=0x%llx "
              "dataspace=0x%x rotation=%d w=%d h=%d max_buffers=%d",
              __FUNCTION__, i, s->stream_type, s->format,
              (unsigned long long)s->usage, s->data_space, s->rotation,
              s->width, s->height, s->max_buffers);
    }

    camera3_stream_t *preview = NULL;
    camera3_stream_t *analysis = NULL;
    camera3_stream_t *video = NULL;
    camera3_stream_t *capture = NULL;
    int previewCount = 0, captureCount = 0;
    for (uint32_t i = 0; i < stream_list->num_streams; i++) {
        camera3_stream_t *s = stream_list->streams[i];
        if (s == NULL || s->stream_type != CAMERA3_STREAM_OUTPUT)
            return -EINVAL;
        if (s->format == HAL_PIXEL_FORMAT_BLOB) {
            captureCount++;
            if (capture == NULL)
                capture = s;
        } else if (s->format == HAL_PIXEL_FORMAT_YCRCB_420_SP ||
                   s->format == HAL_PIXEL_FORMAT_YCbCr_420_888 ||
                   s->format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED ||
                   s->format == HAL_PIXEL_FORMAT_RGBA_8888 ||
                   s->format == HAL_PIXEL_FORMAT_RGBX_8888) {
            /* Video stream detection:
             * - IMPLEMENTATION_DEFINED with GRALLOC_USAGE_HW_VIDEO_ENCODER
             * - YCbCr_420_888 with VIDEO_ENCODER usage
             * Video streams are treated like preview (same SCP source),
             * but tracked separately so SCP is configured to max(preview,video)
             * and servePreviewRequest serves both streams. */
            bool isVideo = (s->usage & GRALLOC_USAGE_HW_VIDEO_ENCODER) != 0;
            if (isVideo && video == NULL) {
                video = s;
                previewCount++;  // video counts as a non-BLOB output stream
            } else {
                previewCount++;
                if (preview == NULL)
                    preview = s;
                else if (analysis == NULL)
                    analysis = s;
            }
            /*
             * SCP produces NV21M (Y + interleaved VU).
             * convertingHalPreviewFormat(NV21M) → 0x11e
             * (EXYNOS_YCrCb_420_SP_M_FULL, multi-plane fd+fd1).
             *
             * BUT: In HAL3 the framework/SurfaceTexture allocates the buffer
             * in a different process. private_handle_t has sNumFds=3, i.e.
             * fd, fd1, fd2 are all translated via Binder. For
             * single-plane NV21, fd1=-1 (no chroma fd).
             *
             * 0x11e works when the buffer is allocated in-process
             * (same-process gralloc allocator).
             *
             * Solution for HAL3: IMPLEMENTATION_DEFINED → 0x11
             * (HAL_PIXEL_FORMAT_YCrCb_420_SP = NV21 single-plane).
             * Only 1 fd needed, works across process boundaries.
             * The HAL copies NV21M (2-plane) → NV21 (1-plane) in
             * copyPreviewToStream/copyPreviewToStreamFromData.
             *
             * Video streams: same treatment — gralloc would for
             * IMPLEMENTATION_DEFINED + VIDEO_ENCODER allocate NV12M (multi-plane),
             * which does not work across processes.
             * NV21 single-plane works and the MediaCodec encoder
             * accepts NV21 as input format.
             */
            if (s->format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED) {
                ALOGI("%s: stream %d: format 0x%x -> YCrCb_420_SP (NV21, 0x11) "
                      "[video=%d usage=0x%llx]",
                      __FUNCTION__, i, s->format, isVideo,
                      (unsigned long long)s->usage);
                s->format = HAL_PIXEL_FORMAT_YCrCb_420_SP;
                s->usage |= (GRALLOC_USAGE_HW_COMPOSER |
                             GRALLOC_USAGE_HW_TEXTURE |
                             GRALLOC_USAGE_SW_WRITE_OFTEN);
            } else if (s->format == HAL_PIXEL_FORMAT_YCbCr_420_888 ||
                       s->format == HAL_PIXEL_FORMAT_YCRCB_420_SP) {
                // YCbCr_420_888 (0x23) and YCrCb_420_SP (0x11) cannot be
                // overridden by the HAL (framework rejects it).  Keep the
                // format and add SW_WRITE so the HAL can lock and fill it.
                s->usage |= (GRALLOC_USAGE_HW_COMPOSER |
                             GRALLOC_USAGE_HW_TEXTURE |
                             GRALLOC_USAGE_SW_WRITE_OFTEN);
            }
        } else {
            ALOGW("%s: unsupported format 0x%x", __FUNCTION__, s->format);
            return -EINVAL;
        }
        s->data_space = unMapLegacyDataSpace(s->data_space);
        s->max_buffers = 2;
        ALOGI("%s: stream[%u] AFTER:  type=%d format=0x%x usage=0x%llx "
              "dataspace=0x%x rotation=%d w=%d h=%d max_buffers=%d",
              __FUNCTION__, i, s->stream_type, s->format,
              (unsigned long long)s->usage, s->data_space, s->rotation,
              s->width, s->height, s->max_buffers);
    }
    if (preview == NULL)
        return -EINVAL;

    // Stream count validation against maxNumOutputStreams = [2, 1, 1, 0].
    if (previewCount > 2) {
        ALOGE("%s: too many preview/YUV streams: %d (max 2)", __FUNCTION__,
              previewCount);
        return -EINVAL;
    }
    if (captureCount > 1) {
        ALOGE("%s: too many JPEG/BLOB streams: %d (max 1)", __FUNCTION__,
              captureCount);
        return -EINVAL;
    }
    ALOGI("%s: streams classified: preview=%d capture=%d analysis=%p video=%p",
          __FUNCTION__, previewCount, captureCount, analysis, video);

    Mutex::Autolock l(m_lock);
    m_previewStream = preview;
    m_analysisStream = analysis;
    m_videoStream = video;
    m_captureStream = capture;

    preview->max_buffers = 4;  // NUM_SCP_BUFFERS
    if (video != NULL) {
        // Video stream: max_buffers = 4 (like preview, same SCP source)
        video->max_buffers = 4;
    }
    if (capture != NULL) {
        // BLOB/JPEG stream: set JFIF dataspace and allow 1 buffer
        capture->max_buffers = 1;
        capture->data_space = HAL_DATASPACE_V0_JFIF;
        ALOGI("%s: capture stream %dx%d, dataspace=JFIF",
              __FUNCTION__, capture->width, capture->height);
        // SCC runs continuously at full-res (3264x2448).
        // The capture resolution is scaled during JPEG encoding.
    }

    if (m_pipeEngine == NULL) {
        ALOGE("%s: pipeEngine not initialized", __FUNCTION__);
        return -ENODEV;
    }

    ALOGI("%s: preview %dx%d fmt=0x%x capture=%p", __FUNCTION__,
          preview->width, preview->height, preview->format, capture);
    ALOGI("%s: assignment: preview=%p analysis=%p video=%p capture=%p "
          "(preview max_buffers=%d, capture max_buffers=%d)",
          __FUNCTION__, preview, analysis, video, capture,
          preview->max_buffers, capture ? capture->max_buffers : -1);

    // SCP dimensions: BDS size for the aspect ratio of the preview size.
    // startPreview() and reconfigureScp() always configure SCP to BDS
    // (e.g. 1920x1080 for 16:9), not to the requested preview size.
    // Preview 1280x720 is software-scaled down from 1920x1080.
    // If a video stream with 1920x1080 is added later, reconfigureScp
    // is a no-op (same BDS size) — no preview interruption.
    // We compute BDS here so the same-size fast path in reconfigureScp
    // triggers (newW == m_previewW). startPreview() also computes BDS
    // internally, but the values must be consistent.
    int bdsW, bdsH;
    G800FPipeEngine::getPreviewBdsSize(preview->width, preview->height, &bdsW, &bdsH);
    int scpW = bdsW;
    int scpH = bdsH;
    if (video != NULL) {
        // Video stream can be larger than preview (e.g. preview 1280x720,
        // video 1920x1080). If video had a different BDS size than preview,
        // we would need to recompute BDS. In practice both have the same
        // aspect ratio (16:9), so the same BDS. We just log it.
        int videoBdsW, videoBdsH;
        G800FPipeEngine::getPreviewBdsSize(video->width, video->height,
                                           &videoBdsW, &videoBdsH);
        ALOGI("%s: video %dx%d → BDS %dx%d (preview BDS=%dx%d)",
              __FUNCTION__, video->width, video->height,
              videoBdsW, videoBdsH, scpW, scpH);
        if (videoBdsW != scpW || videoBdsH != scpH) {
            // Aspect ratio change between preview and video — take max BDS.
            if (videoBdsW > scpW) scpW = videoBdsW;
            if (videoBdsH > scpH) scpH = videoBdsH;
            ALOGI("%s: video BDS differs → SCP=%dx%d", __FUNCTION__, scpW, scpH);
        }
    }

    // Start the preview pipeline here in configureStreams() instead of
    // deferring to the first processCaptureRequest().  The framework's
    // syncWithDevice() has a ~800ms timeout; if startPreview() takes
    // ~700ms (ISP setfile + ION alloc) and the first frame arrives after
    // that, syncWithDevice times out and the camera crashes at startup.
    // Starting the pipeline here means it's already running when the
    // first request arrives, so the first frame comes back quickly.
    if (m_previewStarted && m_pipeEngine) {
        ALOGI("%s: reconfigure — calling reconfigureScp(%dx%d)", __FUNCTION__, scpW, scpH);
        status_t rcErr = m_pipeEngine->reconfigureScp(scpW, scpH);
        if (rcErr != NO_ERROR) {
            ALOGE("%s: reconfigureScp failed: %d — falling back to full restart",
                  __FUNCTION__, rcErr);
            // Full restart: stopPreview will be done by startPreview's
            // internal guard, then startPreview() with the new dimensions.
            m_pipeEngine->stopPreview();
            m_previewStarted = false;
        }
        // m_previewStarted stays true — pipeline is still running
    } else {
        m_previewStarted = false;
    }

    if (!m_previewStarted && m_pipeEngine) {
        ALOGI("%s: starting preview pipeline (%dx%d)", __FUNCTION__, scpW, scpH);
        status_t err = m_pipeEngine->startPreview(scpW, scpH);
        if (err != NO_ERROR) {
            ALOGE("%s: startPreview failed: %d", __FUNCTION__, err);
            return -ENODEV;
        }
        m_previewStarted = true;
        ALOGI("%s: preview pipeline started in configureStreams", __FUNCTION__);
    }
    return NO_ERROR;
}

const camera_metadata_t* G800FCamera2Device::constructDefaultRequestSettings(int type)
{
    camera_metadata_t *m = allocate_camera_metadata(16, 256);
    if (m == NULL)
        return NULL;

    int32_t requestType = ANDROID_REQUEST_TYPE_CAPTURE;
    add_camera_metadata_entry(m, ANDROID_REQUEST_TYPE, &requestType, 1);

    int32_t controlMode = ANDROID_CONTROL_MODE_AUTO;
    add_camera_metadata_entry(m, ANDROID_CONTROL_MODE, &controlMode, 1);

    int32_t intent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
    switch (type) {
        case CAMERA3_TEMPLATE_STILL_CAPTURE:
            intent = ANDROID_CONTROL_CAPTURE_INTENT_STILL_CAPTURE;
            break;
        case CAMERA3_TEMPLATE_VIDEO_RECORD:
            intent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_RECORD;
            break;
        case CAMERA3_TEMPLATE_VIDEO_SNAPSHOT:
            intent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_SNAPSHOT;
            break;
        case CAMERA3_TEMPLATE_ZERO_SHUTTER_LAG:
            intent = ANDROID_CONTROL_CAPTURE_INTENT_ZERO_SHUTTER_LAG;
            break;
        case CAMERA3_TEMPLATE_MANUAL:
            intent = ANDROID_CONTROL_CAPTURE_INTENT_MANUAL;
            break;
        case CAMERA3_TEMPLATE_PREVIEW:
        default:
            intent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
            break;
    }
    add_camera_metadata_entry(m, ANDROID_CONTROL_CAPTURE_INTENT, &intent, 1);

    int32_t aeMode = ANDROID_CONTROL_AE_MODE_ON;
    add_camera_metadata_entry(m, ANDROID_CONTROL_AE_MODE, &aeMode, 1);

    int32_t afMode = ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE;
    add_camera_metadata_entry(m, ANDROID_CONTROL_AF_MODE, &afMode, 1);

    int32_t awbMode = ANDROID_CONTROL_AWB_MODE_AUTO;
    add_camera_metadata_entry(m, ANDROID_CONTROL_AWB_MODE, &awbMode, 1);

    int32_t flashMode = ANDROID_FLASH_MODE_OFF;
    add_camera_metadata_entry(m, ANDROID_FLASH_MODE, &flashMode, 1);

    return m;
}

int G800FCamera2Device::processCaptureRequest(camera3_capture_request_t *request)
{
    //ALOGI("%s: frame=%u num_buffers=%u settings=%p input_buffer=%p "
    //      "num_physcam_settings=%u (async queue)",
    //      __FUNCTION__, request ? request->frame_number : 0,
    //      request ? request->num_output_buffers : 0,
    //      request ? request->settings : NULL,
    //      request ? request->input_buffer : NULL,
    //      request ? request->num_physcam_settings : 0);
    if (request == NULL || m_callbackOps == NULL)
        return -EINVAL;

    // Dump output buffers with stream assignment (framework perspective).
    for (uint32_t i = 0; i < request->num_output_buffers; i++) {
        //const camera3_stream_buffer_t *sb = &request->output_buffers[i];
        //camera3_stream_t *s = sb ? sb->stream : NULL;
        //buffer_handle_t buf = (sb && sb->buffer) ? *sb->buffer : NULL;
        //ALOGI("%s: out_buf[%u] stream=%p{fmt=0x%x %dx%d usage=0x%llx} "
        //      "buffer=%p status=%d acquire_fence=%d release_fence=%d",
        //      __FUNCTION__, i, s, s ? s->format : 0,
        //      s ? s->width : 0, s ? s->height : 0,
        //      s ? (unsigned long long)s->usage : 0ULL,
        //      buf, sb ? sb->status : -1,
        //      sb ? sb->acquire_fence : -1,
        //      sb ? sb->release_fence : -1);
    }

    Mutex::Autolock l(m_lock);
    if (m_pipeEngine == NULL || m_previewStream == NULL)
        return -ENODEV;

    // Clone the request into a QueuedRequest so the worker thread can
    // process it independently after we return.
    QueuedRequest qr;
    qr.frame_number = request->frame_number;
    qr.num_output_buffers = request->num_output_buffers;
    qr.shutter_timestamp = systemTime(SYSTEM_TIME_MONOTONIC);
    qr.shutter_sent = false;
    qr.metadata_sent = false;
    qr.flashFired = false;

    qr.output_buffers = (camera3_stream_buffer_t *)
        malloc(sizeof(camera3_stream_buffer_t) * request->num_output_buffers);
    if (qr.output_buffers == NULL) {
        ALOGE("%s: failed to clone output buffers", __FUNCTION__);
        return NO_MEMORY;
    }
    memcpy(qr.output_buffers, request->output_buffers,
           sizeof(camera3_stream_buffer_t) * request->num_output_buffers);

    if (request->settings != NULL) {
        qr.settings = clone_camera_metadata(request->settings);
        if (qr.settings == NULL) {
            ALOGE("%s: failed to clone settings", __FUNCTION__);
            free(qr.output_buffers);
            return NO_MEMORY;
        }
    } else {
        qr.settings = NULL;
    }

    {
        Mutex::Autolock ql(m_queueLock);
        m_requestQueue.push_back(qr);
        //ALOGV("%s: frame=%u queued (queue=%d)", __FUNCTION__,
        //      qr.frame_number, (int)m_requestQueue.size());
        m_queueCond.signal();
    }

    // Return immediately — the worker thread processes async.
    return NO_ERROR;
}

bool G800FCamera2Device::captureThreadLoop()
{
    QueuedRequest qr;
    {
        Mutex::Autolock ql(m_queueLock);
        while (m_requestQueue.empty() && m_captureThreadRunning) {
            //ALOGI("%s: idle (queue empty, waiting)", __FUNCTION__);
            m_queueCond.wait(m_queueLock);
        }
        if (!m_captureThreadRunning && m_requestQueue.empty())
            return false;
        if (m_requestQueue.empty())
            return true;
        qr = m_requestQueue.front();
        m_requestQueue.pop_front();
        //ALOGV("%s: dequeued frame=%u (queue=%d)", __FUNCTION__,
        //      qr.frame_number, (int)m_requestQueue.size());
        // Tell flush() that we are inside a request so it can wait for us.
        m_workerBusy = true;
    }

    //ALOGD("%s: dispatching frame=%u (queue=%d)", __FUNCTION__, qr.frame_number, (int)m_requestQueue.size());

    // Classify: preview-only vs still-capture
    // Per camera3 HAL spec: "The actual request processing is asynchronous,
    // with the results of capture being returned through process_capture_result().
    // Multiple requests are expected to be in flight at once."
    //
    // Preview-only requests are handled inline (fast, non-blocking).
    // Still-capture requests are pushed to the async capture worker queue
    // so this thread can immediately continue serving preview.
    bool hasCaptureBuffer = false;
    for (uint32_t i = 0; i < qr.num_output_buffers; i++) {
        if (qr.output_buffers[i].stream == m_captureStream &&
            m_captureStream != NULL &&
            m_captureStream->format == HAL_PIXEL_FORMAT_BLOB) {
            hasCaptureBuffer = true;
            break;
        }
    }

    if (hasCaptureBuffer) {
        // Send shutter notify NOW, before pushing to the async worker.
        // The capture worker may process this frame later than the next
        // preview frame (which is handled inline by this thread), causing
        // out-of-order shutter notifications → Camera3Device error.
        // By sending the shutter here (in frame-number order), we avoid
        // the race. The shutter timestamp is systemTime() (not the sensor
        // timestamp), but for still captures this is acceptable — the
        // result metadata will still carry the sensor timestamp.
        qr.shutter_timestamp = systemTime(SYSTEM_TIME_MONOTONIC);
        camera3_notify_msg_t notify_msg;
        memset(&notify_msg, 0, sizeof(notify_msg));
        notify_msg.type = CAMERA3_MSG_SHUTTER;
        notify_msg.message.shutter.frame_number = qr.frame_number;
        notify_msg.message.shutter.timestamp = qr.shutter_timestamp;
        ALOGI("%s: frame=%u shutter (early) ts=%llu", __FUNCTION__,
              qr.frame_number, (unsigned long long)qr.shutter_timestamp);
        m_callbackOps->notify(m_callbackOps, &notify_msg);
        qr.shutter_sent = true;

        // Send result metadata early — before the async worker processes
        // the still capture.  The framework requires capture result metadata
        // in monotonically increasing frame-number order.  If the worker
        // sends frame N's metadata after the main thread already sent
        // results for frames N+1..M, the framework rejects it as
        // out-of-order.  By sending the metadata here (in order) and the
        // buffers later (without metadata → no ordering check), we satisfy
        // the framework's ordering requirement while keeping the async
        // capture worker.
        camera_metadata_t *earlyMeta = buildResultMetadata(
            qr, qr.shutter_timestamp, false, NULL);
        sendResult(qr.frame_number, NULL, 0, earlyMeta, qr.shutter_timestamp);
        qr.metadata_sent = true;

        // Push to async capture worker queue — non-blocking
        {
            Mutex::Autolock cl(m_captureQueueLock);
            m_captureQueue.push_back(qr);
            m_captureQueueCond.signal();
        }
        // Don't free qr here — the capture worker owns it now.
    } else {
        // Preview-only: handle inline (fast path)
        handleRequest(qr);
        free(qr.output_buffers);
        if (qr.settings) free_camera_metadata(qr.settings);
    }

    {
        Mutex::Autolock ql(m_queueLock);
        m_workerBusy = false;
        m_queueCond.broadcast();
    }

    return true;
}

// Async still-capture worker: processes capture requests in a separate thread
// so the main captureThreadLoop can continue serving preview without blocking.
// This matches the camera3 HAL spec: "Multiple requests are expected to be
// in flight at once, to maintain full output frame rate."
bool G800FCamera2Device::captureWorkerLoop()
{
    QueuedRequest qr;
    {
        Mutex::Autolock cl(m_captureQueueLock);
        while (m_captureQueue.empty() && m_captureWorkerRunning) {
            m_captureQueueCond.wait(m_captureQueueLock);
        }
        if (!m_captureWorkerRunning && m_captureQueue.empty())
            return false;
        if (m_captureQueue.empty())
            return true;
        qr = m_captureQueue.front();
        m_captureQueue.pop_front();
    }

    //ALOGI("%s: processing capture frame=%u", __FUNCTION__, qr.frame_number);

    // Handle the full capture flow (flash metering, reprocessing, JPEG, sendResult)
    // This may block for seconds (reprocessing + JPEG encode), but since it runs
    // in its own thread, the preview thread (captureThreadLoop) stays responsive.
    handleRequest(qr);

    free(qr.output_buffers);
    if (qr.settings) free_camera_metadata(qr.settings);

    return true;
}

void G800FCamera2Device::sendResult(uint32_t frameNumber,
                                     const camera3_stream_buffer_t *buffers,
                                     uint32_t numBuffers,
                                     camera_metadata_t *meta,
                                     int64_t /*timestamp*/)
{
    if (m_callbackOps == NULL) {
        ALOGW("%s: frame=%u m_callbackOps=NULL, dropping result", __FUNCTION__, frameNumber);
        if (meta) free_camera_metadata(meta);
        return;
    }
    //ALOGV("%s: frame=%u numBuffers=%u meta=%p partial_result=1",
    //      __FUNCTION__, frameNumber, numBuffers, meta);
    for (uint32_t i = 0; i < numBuffers; i++) {
        //const camera3_stream_buffer_t *sb = &buffers[i];
        //camera3_stream_t *s = sb ? sb->stream : NULL;
        //ALOGI("%s:   result_buf[%u] stream=%p{fmt=0x%x %dx%d} status=%d",
        //      __FUNCTION__, i, s, s ? s->format : 0,
        //      s ? s->width : 0, s ? s->height : 0,
        //      sb ? sb->status : -1);
    }
    camera3_capture_result_t result;
    memset(&result, 0, sizeof(result));
    result.frame_number = frameNumber;
    result.num_output_buffers = numBuffers;
    result.output_buffers = buffers;
    result.partial_result = 1;
    result.result = meta;
    m_callbackOps->process_capture_result(m_callbackOps, &result);
    if (meta) free_camera_metadata(meta);
}

void G800FCamera2Device::sendErrorResult(uint32_t frameNumber,
                                          camera3_stream_buffer_t *buffers,
                                          uint32_t numBuffers)
{
    ALOGI("%s: frame=%u numBuffers=%u", __FUNCTION__, frameNumber, numBuffers);
    if (buffers == NULL) return;

    // Per camera3 HAL spec: "If a complete request is unable to be captured,
    // it must send a CAMERA3_MSG_ERROR notification with ERROR_REQUEST code."
    // The framework sets skipResultMetadata=true only when it receives this
    // notify. Without it, the in-flight request is never removed from the
    // framework's InFlightMap (numBuffersLeft==0 but skipResultMetadata==false
    // and haveResultMetadata==false and shutterTimestamp==0), causing
    // waitUntilDrained to time out.
    if (m_callbackOps != NULL) {
        camera3_notify_msg_t err_msg;
        memset(&err_msg, 0, sizeof(err_msg));
        err_msg.type = CAMERA3_MSG_ERROR;
        err_msg.message.error.frame_number = frameNumber;
        err_msg.message.error.error_code = CAMERA3_MSG_ERROR_REQUEST;
        err_msg.message.error.error_stream = NULL;
        m_callbackOps->notify(m_callbackOps, &err_msg);
    }

    for (uint32_t i = 0; i < numBuffers; i++) {
        buffers[i].status = CAMERA3_BUFFER_STATUS_ERROR;
        buffers[i].release_fence = -1;
    }

    sendResult(frameNumber, buffers, numBuffers, NULL, 0);
}

void G800FCamera2Device::dumpRequestSettings(const camera_metadata_t *settings)
{
    if (settings == NULL) return;

    // Helper macro: reads a metadata key and logs it.
    // Uses data.u8[0] / data.i32[0] / data.i64[0] depending on type.
    // We log only the keys relevant to HAL3/ISP.
    camera_metadata_ro_entry_t e;

    // --- AE (Auto-Exposure) ---
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AE_MODE, &e) == OK && e.count > 0)
        //ALOGI("%s:   AE_MODE=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION, &e) == OK && e.count > 0)
        //ALOGI("%s:   AE_EXPOSURE_COMPENSATION=%d", __FUNCTION__, e.data.i32[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AE_LOCK, &e) == OK && e.count > 0)
        //ALOGI("%s:   AE_LOCK=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AE_TARGET_FPS_RANGE, &e) == OK && e.count >= 2)
        //ALOGI("%s:   AE_TARGET_FPS_RANGE=[%d,%d]", __FUNCTION__, e.data.i32[0], e.data.i32[1]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER, &e) == OK && e.count > 0)
        //ALOGI("%s:   AE_PRECAPTURE_TRIGGER=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_SENSOR_EXPOSURE_TIME, &e) == OK && e.count > 0)
        //ALOGI("%s:   SENSOR_EXPOSURE_TIME=%lld", __FUNCTION__, (long long)e.data.i64[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_SENSOR_SENSITIVITY, &e) == OK && e.count > 0)
        //ALOGI("%s:   SENSOR_SENSITIVITY=%d", __FUNCTION__, e.data.i32[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AE_REGIONS, &e) == OK && e.count > 0) {
        //ALOGI("%s:   AE_REGIONS count=%d (first=%d,%d,%d,%d,weight=%d)",
        //      __FUNCTION__, e.count, e.data.i32[0], e.data.i32[1],
        //      e.data.i32[2], e.data.i32[3], e.data.i32[4]);
    }

    // --- AWB (Auto-White-Balance) ---
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AWB_MODE, &e) == OK && e.count > 0)
        //ALOGI("%s:   AWB_MODE=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AWB_LOCK, &e) == OK && e.count > 0)
        //ALOGI("%s:   AWB_LOCK=%d", __FUNCTION__, e.data.u8[0]);
        ;

    // --- AF (Auto-Focus) ---
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AF_MODE, &e) == OK && e.count > 0)
        //ALOGI("%s:   AF_MODE=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AF_TRIGGER, &e) == OK && e.count > 0)
        //ALOGI("%s:   AF_TRIGGER=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AF_REGIONS, &e) == OK && e.count > 0) {
        //ALOGI("%s:   AF_REGIONS count=%d (first=%d,%d,%d,%d,weight=%d)",
        //      __FUNCTION__, e.count, e.data.i32[0], e.data.i32[1],
        //      e.data.i32[2], e.data.i32[3], e.data.i32[4]);
    }

    // --- Flash ---
    if (find_camera_metadata_ro_entry(settings, ANDROID_FLASH_MODE, &e) == OK && e.count > 0)
        //ALOGI("%s:   FLASH_MODE=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_FLASH_STATE, &e) == OK && e.count > 0)
        //ALOGI("%s:   FLASH_STATE=%d", __FUNCTION__, e.data.u8[0]);
        ;

    // --- Scene / Mode ---
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_SCENE_MODE, &e) == OK && e.count > 0)
        //ALOGI("%s:   SCENE_MODE=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_MODE, &e) == OK && e.count > 0)
        //ALOGI("%s:   CONTROL_MODE=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_EFFECT_MODE, &e) == OK && e.count > 0)
        //ALOGI("%s:   EFFECT_MODE=%d", __FUNCTION__, e.data.u8[0]);
        ;

    // --- Capture Intent / Edge / Noise ---
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_CAPTURE_INTENT, &e) == OK && e.count > 0)
        //ALOGI("%s:   CAPTURE_INTENT=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_EDGE_MODE, &e) == OK && e.count > 0)
        //ALOGI("%s:   EDGE_MODE=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_NOISE_REDUCTION_MODE, &e) == OK && e.count > 0)
        //ALOGI("%s:   NOISE_REDUCTION_MODE=%d", __FUNCTION__, e.data.u8[0]);
        ;

    // --- Zoom / Crop / Scaler ---
    if (find_camera_metadata_ro_entry(settings, ANDROID_SCALER_CROP_REGION, &e) == OK && e.count >= 4)
        //ALOGI("%s:   SCALER_CROP_REGION=[x=%d,y=%d,w=%d,h=%d]",
        //      __FUNCTION__, e.data.i32[0], e.data.i32[1],
        //      e.data.i32[2], e.data.i32[3]);
        ;

    // --- JPEG ---
    if (find_camera_metadata_ro_entry(settings, ANDROID_JPEG_QUALITY, &e) == OK && e.count > 0)
        //ALOGI("%s:   JPEG_QUALITY=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_JPEG_THUMBNAIL_QUALITY, &e) == OK && e.count > 0)
        //ALOGI("%s:   JPEG_THUMBNAIL_QUALITY=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_JPEG_THUMBNAIL_SIZE, &e) == OK && e.count >= 2)
        //ALOGI("%s:   JPEG_THUMBNAIL_SIZE=[%d,%d]", __FUNCTION__, e.data.i32[0], e.data.i32[1]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_JPEG_ORIENTATION, &e) == OK && e.count > 0)
        //ALOGI("%s:   JPEG_ORIENTATION=%d", __FUNCTION__, e.data.i32[0]);
        ;

    // --- Statistics / Lens ---
    if (find_camera_metadata_ro_entry(settings, ANDROID_STATISTICS_FACE_DETECT_MODE, &e) == OK && e.count > 0)
        //ALOGI("%s:   FACE_DETECT_MODE=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_LENS_OPTICAL_STABILIZATION_MODE, &e) == OK && e.count > 0)
        //ALOGI("%s:   LENS_OIS_MODE=%d", __FUNCTION__, e.data.u8[0]);
        ;
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_VIDEO_STABILIZATION_MODE, &e) == OK && e.count > 0)
        //ALOGI("%s:   VIDEO_STAB_MODE=%d", __FUNCTION__, e.data.u8[0]);
        ;

    // --- Antibanding ---
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AE_ANTIBANDING_MODE, &e) == OK && e.count > 0)
        //ALOGI("%s:   AE_ANTIBANDING_MODE=%d", __FUNCTION__, e.data.u8[0]);
        ;
}

void G800FCamera2Device::handleRequest(QueuedRequest &qr)
{
    //ALOGI("%s: frame=%u start", __FUNCTION__, qr.frame_number);

    // Full dump of request settings (Camera2 metadata keys) — disabled:
    // fires on every settings-bearing request.
    //if (qr.settings != NULL) {
    //    dumpRequestSettings(qr.settings);
    //}

    // Preview pipeline is now started in configureStreams() so that the
    // first frame is produced quickly enough for syncWithDevice().
    // If it's not running here, something went wrong — return error.
    if (!m_previewStarted) {
        ALOGE("%s: preview not started — returning error buffers", __FUNCTION__);
        sendErrorResult(qr.frame_number, qr.output_buffers, qr.num_output_buffers);
        return;
    }

    // NOTE: Shutter notify is deferred until after getPreviewFrame() to
    // set the timestamp at actual frame-acquisition time, not queue time.
    // This is critical for AV-sync: the MPEG4 writer uses
    // ANDROID_SENSOR_TIMESTAMP as PTS for each video frame.  If the
    // timestamp is set at queue time (processCaptureRequest), the
    // variable pipeline delay (retries, EAGAIN, ISP latency) causes
    // irregular PTS and audio/video drift.

    // Check if this request has a BLOB/JPEG capture buffer.
    bool hasCaptureBuffer = false;
    for (uint32_t i = 0; i < qr.num_output_buffers; i++) {
        if (qr.output_buffers[i].stream == m_captureStream &&
            m_captureStream != NULL &&
            m_captureStream->format == HAL_PIXEL_FORMAT_BLOB) {
            hasCaptureBuffer = true;
            break;
        }
    }

    // Apply Camera2 request settings to FLITE shot_ext
    m_pipeEngine->applyRequestSettings(qr.settings);

    // Flash control for still capture (rear camera ID 0 only)
    // Full flash sequence:
    //   1. beginFlashSequence()        → IDLE→START (pre-flash on)
    //   2. pumpPreviewWhileFlashMetering() → START→ON→METERING→READY (in FLITE thread)
    //      Serve preview requests meanwhile so the SCP queue does not overflow.
    //   3. armFlashCapture()           → READY→CAPTURE (main flash armed)
    //   4. requestStillCapture()       → FrameSelector selects CAPTURE frame
    //   5. endFlashSequence()          → CAPTURE→IDLE (LED off + strobe unlock)
    //
    // flashFired is a PER-REQUEST flag (QueuedRequest field): the still
    // capture runs on the capture worker thread while preview requests keep
    // flowing through handleRequest on the capture thread.  A shared member
    // (m_flashFired) was reset by interleaved preview requests, which skipped
    // endFlashSequence() → the flash state machine stayed in DONE → aeMode/
    // awbMode stayed OFF (applyRequestSettings skips them while active) →
    // green-tinted preview until the 5 s watchdog fired.
    qr.flashFired = false;
    if (m_cameraId == 0) {
        int32_t flashMode = ANDROID_FLASH_MODE_OFF;
        int32_t aeMode = ANDROID_CONTROL_AE_MODE_ON;
        int32_t aePrecaptureTrigger = ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER_IDLE;
        if (qr.settings != NULL) {
            camera_metadata_ro_entry_t entry;
            if (find_camera_metadata_ro_entry(qr.settings,
                                              ANDROID_FLASH_MODE, &entry) == OK && entry.count > 0)
                flashMode = entry.data.u8[0];
            if (find_camera_metadata_ro_entry(qr.settings,
                                              ANDROID_CONTROL_AE_MODE, &entry) == OK && entry.count > 0)
                aeMode = entry.data.u8[0];
            if (find_camera_metadata_ro_entry(qr.settings,
                                              ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER, &entry) == OK && entry.count > 0)
                aePrecaptureTrigger = entry.data.u8[0];
        }
        bool autoFlash = (aeMode == ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH ||
                          aeMode == ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH_REDEYE);
        // ON_ALWAYS_FLASH and FLASH_MODE_TORCH are CONTINUOUS-light modes
        // — the LED stays lit and must NOT trigger the
        // preflash/strobe sequence.  Running the sequence here fired the
        // main strobe on top of the already-lit scene → overexposed photo.
        // Only SINGLE (and auto-flash's precapture path) arms the strobe.
        bool singleFlash = (flashMode == ANDROID_FLASH_MODE_SINGLE);
        bool precaptureStart = (aePrecaptureTrigger == ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER_START);

        // Precapture trigger starts the flash sequence (pre-flash on).
        // The state machine then runs in the FLITE thread (advanceFlashSequence).
        if (precaptureStart && autoFlash) {
            if (m_pipeEngine && !m_pipeEngine->isFlashSequenceActive()) {
                ALOGI("%s: frame %d: AE precapture START → begin flash sequence",
                      __FUNCTION__, qr.frame_number);
                m_pipeEngine->beginFlashSequence();
            }
        }

        // Still capture with flash: complete sequence (arm + capture)
        if (hasCaptureBuffer && (autoFlash || singleFlash)) {
            qr.flashFired = true;
            if (m_pipeEngine) {
                // If sequence not yet active (no precapture trigger):
                // start it now.
                if (!m_pipeEngine->isFlashSequenceActive()) {
                    ALOGI("%s: frame %d: flash capture — starting sequence now",
                          __FUNCTION__, qr.frame_number);
                    m_pipeEngine->beginFlashSequence();
                }
                // Wait until flash metering is done (READY) AND the firmware
                // reports dm.flash.flashReady==2 (MAIN_READY gate — up
                // to ~1.1 s after READY), serving preview meanwhile.
                // Worst case ~2.5+1.2+1.1 s ≈ 5 s, bounded by the 8 s
                // sequence watchdog.
                int meteringMs = pumpPreviewWhileFlashMetering(5000);

                if (!m_pipeEngine->isFlashSequenceActive()) {
                    // Sequence died during the wait (watchdog/timeout).
                    // Arming now would write a CAPTURE tuple into a dead
                    // sequence — the aeflashMode OFF→CAPTURE transition
                    // would fire the strobe unarmed.  Skip to the normal
                    // capture path (JPEG from preview frame).
                    ALOGW("%s: frame %d: flash sequence died during metering "
                          "— skipping arm, falling back", __FUNCTION__,
                          qr.frame_number);
                } else {
                // IMPORTANT: call requestStillCapture() BEFORE armFlashCapture()!
                // The capture redirect (setCaptureRedirect) must be active BEFORE
                // the main flash frame is produced, so the FrameSelector captures
                // the flash frame and passes it to the reprocThread.
                // Previously the order was: arm → getPreview → requestStillCapture
                // → the flash frame was already gone, reprocThread got a
                // later frame without flash → overexposed photo.
                // requestStillCapture: target dimensions = SCC full-res (3264x2448).
                // The BLOB stream width/height are buffer dimensions (maxJpegSize × 1),
                // NOT image dimensions — don't pass them as capture target.
                int reprocTargetW = m_pipeEngine->getSccWidth();
                int reprocTargetH = m_pipeEngine->getSccHeight();
                status_t reprocErr = m_pipeEngine->requestStillCapture(reprocTargetW, reprocTargetH, true);
                if (reprocErr != NO_ERROR) {
                    ALOGE("%s: frame %d: requestStillCapture failed: %d — falling back",
                          __FUNCTION__, qr.frame_number, reprocErr);
                    // Fallback: no reprocessing, JPEG from preview frame
                    // (serveCaptureRequest will handle the fallback)
                } else {
                    ALOGI("%s: frame %d: reprocessing armed before flash capture",
                          __FUNCTION__, qr.frame_number);
                }

                // Arm main flash (READY → CAPTURE).
                // Now that capture redirect is active, the flash frame
                // will be captured by the FrameSelector and routed to reprocThread.
                m_pipeEngine->armFlashCapture();
                ALOGI("%s: frame %d: flash armed after %dms metering, state=%s",
                      __FUNCTION__, qr.frame_number, meteringMs,
                      G800FPipeEngine::flashSeqStateName(
                          m_pipeEngine->getFlashSeqState()));
                }
            }
        }
    }

    // --- Dispatcher: Preview vs. Capture ---
    // Get preview frame — with retry, since the pipeline just started
    // and the first frames may not be available yet (EAGAIN).
    // 90 retries × 33ms = ~3 seconds — enough for ISP/SCP startup +
    // AE/AWB convergence (setFrameSkipCount(8) ≈ 8 frames ≈ 270ms).
    G800FFrame* previewFrame = NULL;
    status_t err = NO_ERROR;
    int retryCount = 0;
    for (int retry = 0; retry < 90; retry++) {
        err = m_pipeEngine->getPreviewFrame(&previewFrame);
        if (err == NO_ERROR && previewFrame != NULL) break;
        retryCount++;
        if (err != -EAGAIN && err != NO_ERROR) break;  // real error
        usleep(33000);  // ~1 frame at 30fps
    }
    if (retryCount > 0) {
        ALOGW("%s: frame=%u getPreviewFrame retried %d times, err=%d, frame=%p",
              __FUNCTION__, qr.frame_number, retryCount, err, previewFrame);
    }
    if (err != NO_ERROR || previewFrame == NULL) {
        ALOGE("%s: getPreviewFrame failed: %d (after %d retries)", __FUNCTION__, err, retryCount);
        sendErrorResult(qr.frame_number, qr.output_buffers, qr.num_output_buffers);
        if (previewFrame) m_pipeEngine->releasePreviewFrame(previewFrame);
        return;
    }

    // Send shutter notify now — after the frame is actually acquired from
    // the sensor.  Use the kernel-provided sensor timestamp (dm.sensor.timeStamp)
    // from the shot_ext metadata, which is the exact time the sensor captured
    // the frame (CLOCK_MONOTONIC, ns).  This eliminates the full pipeline
    // latency (FLITE → ISP → SCP ≈ 200-300ms) from the timestamp, which is
    // critical for AV-sync: the MPEG4 writer uses ANDROID_SENSOR_TIMESTAMP as
    // PTS for each video frame.  Falls back to systemTime() if kernel didn't
    // set the timestamp (e.g. shot_ext not available).
    //
    // For still captures, the shutter was already sent early in
    // captureThreadLoop (with systemTime()) to avoid out-of-order shutter
    // notifications. The framework requires ANDROID_SENSOR_TIMESTAMP in the
    // result metadata to match the shutter timestamp, so we must NOT
    // overwrite qr.shutter_timestamp with the sensor timestamp here.
    // The sensor timestamp is still available via previewFrame->timestamp
    // for any app-side use, but the metadata will carry the early timestamp.
    if (!qr.shutter_sent) {
        if (previewFrame && previewFrame->timestamp > 0) {
            qr.shutter_timestamp = (int64_t)previewFrame->timestamp;
        } else {
            qr.shutter_timestamp = systemTime(SYSTEM_TIME_MONOTONIC);
        }
        camera3_notify_msg_t notify_msg;
        memset(&notify_msg, 0, sizeof(notify_msg));
        notify_msg.type = CAMERA3_MSG_SHUTTER;
        notify_msg.message.shutter.frame_number = qr.frame_number;
        notify_msg.message.shutter.timestamp = qr.shutter_timestamp;
        //ALOGV("%s: frame=%u shutter ts=%llu", __FUNCTION__,
        //      qr.frame_number, (unsigned long long)qr.shutter_timestamp);
        m_callbackOps->notify(m_callbackOps, &notify_msg);
    }

    if (hasCaptureBuffer) {
        // Still capture: FrameSelector + asynchronous reprocessing
        serveCaptureRequest(qr, previewFrame);
    } else {
        // Preview-only: deliver SCP frame directly
        servePreviewRequest(qr, previewFrame);
    }

    //ALOGI("%s: frame=%u done", __FUNCTION__, qr.frame_number);
}

bool G800FCamera2Device::servePreviewRequest(QueuedRequest &qr, G800FFrame* previewFrame)
{
    // Direct V4L2 path: previewFrame->buffer points to the SCP V4L2 buffer.
    // For preview stream: FIMC M2M hardware scaling (zero-copy, no CPU).
    // For video/analysis: NEON software scaling (copyPreviewToStream).
    // Fallback (ownsPreviewData): old malloc path with copyPreviewToStreamFromData.
    G800FExynosCameraBuffer *camBuf = previewFrame ? previewFrame->buffer : NULL;
    bool hasPreviewData = previewFrame && previewFrame->ownsPreviewData
                          && previewFrame->previewData[0] != NULL;
    if (!hasPreviewData && camBuf == NULL) {
        ALOGW("%s: frame %u: no preview buffer", __FUNCTION__, qr.frame_number);
        sendErrorResult(qr.frame_number, qr.output_buffers, qr.num_output_buffers);
        m_pipeEngine->releasePreviewFrame(previewFrame);
        return false;
    }

    // Analysis/callback buffers (Camera1 setPreviewCallbackWithBuffer →
    // YCbCr_420_888 stream) are deferred to the analysis worker thread:
    // the scalar CPU copy would otherwise sit on the capture thread's
    // critical path and directly extend every frame interval.
    // The frame's buffers are split across two results: preview/video go
    // out with the metadata now; the analysis buffers follow in a second
    // buffers-only result for the same frame number — legal per camera3
    // spec (same mechanism as the async still-capture path).
    AnalysisJob analysisJob;
    analysisJob.frame_number = qr.frame_number;
    analysisJob.frame = previewFrame;   // ownership transfers when queued
    bool hasAnalysis = false;
    uint32_t syncCount = 0;   // compacted prefix of qr.output_buffers

    for (uint32_t i = 0; i < qr.num_output_buffers; i++) {
        camera3_stream_buffer_t *sb = &qr.output_buffers[i];
        bool isPreview = (sb->stream == m_previewStream);
        bool isAnalysis = (sb->stream == m_analysisStream && m_analysisStream != NULL);
        bool isVideo = (sb->stream == m_videoStream && m_videoStream != NULL);
        buffer_handle_t buf = sb->buffer ? *sb->buffer : NULL;
        if ((!isPreview && !isAnalysis && !isVideo) || buf == NULL) {
            sb->status = CAMERA3_BUFFER_STATUS_ERROR;
            sb->release_fence = -1;
            if (syncCount != i) qr.output_buffers[syncCount] = *sb;
            syncCount++;
            continue;
        }

        if (isAnalysis) {
            // Defer: sb.buffer points into framework request storage and
            // stays valid until the buffer is returned (same guarantee the
            // async still-capture path relies on).
            AnalysisBuf ab;
            ab.sb = *sb;
            ab.w = sb->stream->width;
            ab.h = sb->stream->height;
            analysisJob.bufs.push_back(ab);
            hasAnalysis = true;
            continue;
        }

        // FIMC M2M hardware scaling for preview and video streams (zero-copy).
        // Preview uses FIMC node 0, video uses FIMC node 1.
        // Only when direct V4L2 path is active (camBuf != NULL, !hasPreviewData).
        bool fimcUsed = false;
        if ((isPreview || isVideo) && !hasPreviewData && camBuf != NULL) {
            bool fimcFail = isPreview ? m_fimcFailed : m_fimcVideoFailed;
            G800FFimcScaler* &scaler = isPreview ? m_fimcScaler : m_fimcVideoScaler;
            int fimcNode = isPreview ? 0 : 1;

            if (!fimcFail) {
                private_handle_t *hnd = (private_handle_t *)buf;
                int dstFd = hnd->fd;
                int dstW = sb->stream->width;
                int dstH = sb->stream->height;

                // Source: SCP NV21M 2-plane dmabuf fds
                int srcFdY  = camBuf->dmaBufFd(0);
                int srcFdVU = camBuf->dmaBufFd(1);
                int srcW = m_pipeEngine->getPreviewWidth();
                int srcH = m_pipeEngine->getPreviewHeight();

                if (srcFdY >= 0 && dstFd >= 0 && srcW > 0 && srcH > 0) {
                    bool needConfigure = false;
                    if (!scaler) {
                        scaler = new G800FFimcScaler();
                        if (scaler->open(fimcNode) != NO_ERROR) {
                            ALOGE("%s: FIMC node %d open failed, fallback to NEON",
                                  __FUNCTION__, fimcNode);
                            delete scaler;
                            scaler = NULL;
                            if (isPreview) m_fimcFailed = true;
                            else m_fimcVideoFailed = true;
                        } else {
                            needConfigure = true;
                        }
                    } else if (scaler->isConfigured() &&
                               (scaler->getDstWidth() != dstW ||
                                scaler->getDstHeight() != dstH)) {
                        needConfigure = true;
                    }

                    if (scaler && needConfigure) {
                        if (scaler->configure(srcW, srcH, V4L2_PIX_FMT_NV21M,
                                              dstW, dstH, V4L2_PIX_FMT_NV21) != NO_ERROR) {
                            ALOGE("%s: FIMC node %d configure failed, fallback to NEON",
                                  __FUNCTION__, fimcNode);
                            scaler->close();
                            delete scaler;
                            scaler = NULL;
                            if (isPreview) m_fimcFailed = true;
                            else m_fimcVideoFailed = true;
                        }
                    }

                    if (scaler && scaler->isConfigured()) {
                        status_t ferr = scaler->processFrame(srcFdY, srcFdVU, dstFd);
                        if (ferr == NO_ERROR) {
                            fimcUsed = true;
                            //ALOGD("%s: FIMC[%d] scaled %dx%d → %dx%d (zero-copy)",
                            //      __FUNCTION__, fimcNode, srcW, srcH, dstW, dstH);
                        } else {
                            ALOGE("%s: FIMC[%d] processFrame failed: %d, fallback to NEON",
                                  __FUNCTION__, fimcNode, ferr);
                        }
                    }
                }
            }
        }

        if (!fimcUsed) {
            if (hasPreviewData) {
                // Software scaler: copies/scales SCP NV21M → target format.
                copyPreviewToStreamFromData(previewFrame->previewData[0],
                                            previewFrame->previewData[1],
                                            previewFrame->previewStride[0],
                                            previewFrame->previewHeight,
                                            buf, sb->stream->width, sb->stream->height);
            } else {
                copyPreviewToStream(camBuf, buf, sb->stream->width, sb->stream->height);
            }
        }
        sb->status = CAMERA3_BUFFER_STATUS_OK;
        sb->release_fence = -1;
        if (syncCount != i) qr.output_buffers[syncCount] = *sb;
        syncCount++;
    }

    // Build result metadata BEFORE releasing previewFrame, so the per-frame
    // 3A metadata (afState, aeState, awbState, sensitivity, exposureTime)
    // can be read from the frame.
    camera_metadata_t *meta = buildResultMetadata(qr, qr.shutter_timestamp, false,
                                                  previewFrame);
    sendResult(qr.frame_number, qr.output_buffers, syncCount,
               meta, qr.shutter_timestamp);

    if (hasAnalysis) {
        bool queued = false;
        {
            Mutex::Autolock al(m_analysisQueueLock);
            // Cap the queue depth: each pending job pins one SCP V4L2
            // buffer, so an unbounded queue could starve the pipeline.
            if (m_analysisWorkerRunning && m_analysisQueue.size() < 4) {
                m_analysisQueue.push_back(analysisJob);
                m_analysisQueueCond.signal();
                queued = true;
            }
        }
        if (!queued) {
            // Worker unavailable or queue full — fill inline (slow but
            // correct) so the buffer is still returned.
            fillAnalysisJob(analysisJob);
            if (analysisJob.frame && m_pipeEngine)
                m_pipeEngine->releasePreviewFrame(analysisJob.frame);
        }
        // else: worker owns the frame now — it releases after fill.
    } else {
        m_pipeEngine->releasePreviewFrame(previewFrame);
    }
    return true;
}

// Fills the deferred analysis/callback buffers for one frame and returns
// them as a buffers-only result (result=NULL → no metadata ordering
// requirement).  Called on the analysis worker thread — or inline from
// the capture thread as a fallback when the queue is full.
// Does NOT release job.frame — the caller owns it.
void G800FCamera2Device::fillAnalysisJob(AnalysisJob &job)
{
    G800FExynosCameraBuffer *camBuf = job.frame ? job.frame->buffer : NULL;
    bool hasPreviewData = job.frame && job.frame->ownsPreviewData
                          && job.frame->previewData[0] != NULL;
    if (job.bufs.empty()) return;

    // Build a contiguous sb array for sendResult (AnalysisBuf wraps the sb).
    std::vector<camera3_stream_buffer_t> sbs(job.bufs.size());
    for (size_t i = 0; i < job.bufs.size(); i++) {
        sbs[i] = job.bufs[i].sb;
        buffer_handle_t buf = sbs[i].buffer ? *sbs[i].buffer : NULL;
        if (buf == NULL) {
            sbs[i].status = CAMERA3_BUFFER_STATUS_ERROR;
        } else if (hasPreviewData) {
            copyPreviewToStreamFromData(job.frame->previewData[0],
                                        job.frame->previewData[1],
                                        job.frame->previewStride[0],
                                        job.frame->previewHeight,
                                        buf, job.bufs[i].w, job.bufs[i].h);
            sbs[i].status = CAMERA3_BUFFER_STATUS_OK;
        } else if (camBuf != NULL) {
            // Fast path: FIMC scale into NV21 bounce, deinterleave into
            // the destination.  Falls back to the scalar CPU scaler.
            if (fillAnalysisViaFimc(camBuf, buf, job.bufs[i].w,
                                    job.bufs[i].h) != NO_ERROR) {
                copyPreviewToStream(camBuf, buf, job.bufs[i].w, job.bufs[i].h);
            }
            sbs[i].status = CAMERA3_BUFFER_STATUS_OK;
        } else {
            sbs[i].status = CAMERA3_BUFFER_STATUS_ERROR;
        }
        sbs[i].release_fence = -1;
    }

    sendResult(job.frame_number, sbs.data(), sbs.size(), NULL, 0);
}

// Fast analysis fill: FIMC M2M (node 2) scales SCP NV21M → NV21 into a
// HAL-owned bounce dmabuf, then copyBounceNv21ToStream deinterleaves it
// into the destination buffer's layout.  The scalar path is ~150-250ms
// per 960x720 frame on uncached ION memory; this path is ~15ms.
status_t G800FCamera2Device::fillAnalysisViaFimc(G800FExynosCameraBuffer *camBuf,
                                               buffer_handle_t outBuf,
                                               int w, int h)
{
    if (m_analysisFimcFailed) return NO_INIT;

    int srcFdY  = camBuf->dmaBufFd(0);
    int srcFdVU = camBuf->dmaBufFd(1);
    int srcW = m_pipeEngine ? m_pipeEngine->getPreviewWidth()  : 0;
    int srcH = m_pipeEngine ? m_pipeEngine->getPreviewHeight() : 0;
    if (srcFdY < 0 || srcW <= 0 || srcH <= 0)
        return BAD_VALUE;

    // Lazy scaler init — FIMC node 2 (0=preview, 1=video).
    if (!m_analysisScaler) {
        m_analysisScaler = new G800FFimcScaler();
        if (m_analysisScaler->open(2) != NO_ERROR) {
            ALOGE("%s: FIMC node 2 open failed — permanent CPU fallback",
                  __FUNCTION__);
            delete m_analysisScaler;
            m_analysisScaler = NULL;
            m_analysisFimcFailed = true;
            return NO_INIT;
        }
    }
    if (!m_analysisScaler->isConfigured() ||
        m_analysisScaler->getDstWidth() != w ||
        m_analysisScaler->getDstHeight() != h) {
        if (m_analysisScaler->configure(srcW, srcH, V4L2_PIX_FMT_NV21M,
                                        w, h, V4L2_PIX_FMT_NV21) != NO_ERROR) {
            ALOGE("%s: FIMC node 2 configure failed", __FUNCTION__);
            m_analysisScaler->close();
            delete m_analysisScaler;
            m_analysisScaler = NULL;
            m_analysisFimcFailed = true;
            return NO_INIT;
        }
    }

    // Bounce dmabuf: NV21 = w*h*1.5, one plane.  Realloc on size change.
    if (m_analysisBounce == NULL || m_analysisBounceW != w ||
        m_analysisBounceH != h) {
        if (m_analysisBounce) {
            delete m_analysisBounce;
            m_analysisBounce = NULL;
        }
        m_analysisBounce = new G800FExynosCameraBuffer();
        size_t planeSize = (size_t)w * h * 3 / 2;
        if (m_analysisBounce->allocDmabuf(1, &planeSize) != NO_ERROR) {
            ALOGE("%s: bounce dmabuf alloc failed (%zu bytes)", __FUNCTION__, planeSize);
            delete m_analysisBounce;
            m_analysisBounce = NULL;
            return NO_MEMORY;
        }
        m_analysisBounceW = w;
        m_analysisBounceH = h;
        ALOGI("%s: analysis bounce allocated %dx%d (%zu bytes)",
              __FUNCTION__, w, h, planeSize);
    }

    status_t ret = m_analysisScaler->processFrame(srcFdY, srcFdVU,
                                                  m_analysisBounce->dmaBufFd(0));
    if (ret != NO_ERROR) {
        ALOGE("%s: FIMC node 2 processFrame failed: %d", __FUNCTION__, ret);
        return ret;
    }

    copyBounceNv21ToStream(outBuf, w, h);
    return NO_ERROR;
}

// Copies the NV21 bounce buffer (filled by FIMC) into a gralloc output
// buffer.  NV21 dst → row memcpy; planar (YCbCr_420_888/YV12) → chroma
// deinterleave VU → separate Cb/Cr planes.
void G800FCamera2Device::copyBounceNv21ToStream(buffer_handle_t outBuf,
                                              int w, int h)
{
    if (m_analysisBounce == NULL) return;

    const gralloc_module_t *grmod = NULL;
    hw_module_t const *hwmod = NULL;
    if (hw_get_module(GRALLOC_HARDWARE_MODULE_ID, &hwmod) != 0)
        return;
    grmod = reinterpret_cast<const gralloc_module_t*>(hwmod);

    void *vaddr = NULL;
    if (grmod->lock(grmod, outBuf, GRALLOC_USAGE_SW_WRITE_OFTEN,
                    0, 0, w, h, &vaddr) != 0 || vaddr == NULL) {
        ALOGE("%s: gralloc lock failed", __FUNCTION__);
        return;
    }

    private_handle_t *hnd = (private_handle_t *)outBuf;
    int dstStride = hnd->stride;
    if (dstStride <= 0 || dstStride < w) dstStride = w;

    void *dstC1 = NULL, *dstC2 = NULL;
    grallocGetPlanePtrs(hnd, vaddr, dstC1, dstC2);

    const uint8_t *srcY  = (const uint8_t *)m_analysisBounce->planeVaddr(0);
    const uint8_t *srcVU = srcY + (size_t)w * h;   // NV21: VU follows Y
    uint8_t *dstY = (uint8_t *)vaddr;
    int chromaRows = h / 2;

    if (hnd->format == HAL_PIXEL_FORMAT_YCrCb_420_SP && dstC1 != NULL) {
        // NV21 dst: row copy for Y and interleaved VU
        for (int row = 0; row < h; row++)
            memcpy(dstY + row * dstStride, srcY + row * w, w);
        uint8_t *dstVU = (uint8_t *)dstC1;
        for (int row = 0; row < chromaRows; row++)
            memcpy(dstVU + row * dstStride, srcVU + row * w, w);
    } else if (dstC2 != NULL) {
        // Planar YCbCr_420_888/YV12: memcpy Y, deinterleave VU → Cb/Cr
        int cStride = ALIGN(dstStride / 2, 16);
        uint8_t *dstCb = (uint8_t *)dstC1;
        uint8_t *dstCr = (uint8_t *)dstC2;
        for (int row = 0; row < h; row++)
            memcpy(dstY + row * dstStride, srcY + row * w, w);
        for (int row = 0; row < chromaRows; row++) {
            const uint8_t *s = srcVU + row * w;
            uint8_t *dCb = dstCb + row * cStride;
            uint8_t *dCr = dstCr + row * cStride;
            for (int col = 0; col < w / 2; col++) {
                dCr[col] = s[2 * col];      // NV21: V first
                dCb[col] = s[2 * col + 1];  // U second
            }
        }
    } else if (dstC1 != NULL) {
        // 2-plane dst (NV21M/NV12M): copy Y rows + VU rows verbatim
        for (int row = 0; row < h; row++)
            memcpy(dstY + row * dstStride, srcY + row * w, w);
        uint8_t *dstC = (uint8_t *)dstC1;
        for (int row = 0; row < chromaRows; row++)
            memcpy(dstC + row * dstStride, srcVU + row * w, w);
    } else {
        ALOGW("%s: unsupported dst format 0x%x", __FUNCTION__, hnd->format);
    }

    grmod->unlock(grmod, outBuf);
}

// Worker loop: waits for deferred analysis jobs, fills their buffers
// (FIMC node 2 → NV21 bounce → deinterleave; scalar CPU fallback), sends
// them as a buffers-only result and releases the pinned preview frame
// back to SCP.
bool G800FCamera2Device::analysisWorkerLoop()
{
    AnalysisJob job;
    {
        Mutex::Autolock al(m_analysisQueueLock);
        while (m_analysisQueue.empty() && m_analysisWorkerRunning) {
            m_analysisQueueCond.wait(m_analysisQueueLock);
        }
        if (!m_analysisWorkerRunning && m_analysisQueue.empty())
            return false;
        if (m_analysisQueue.empty())
            return true;
        job.bufs.swap(m_analysisQueue.front().bufs);
        job.frame = m_analysisQueue.front().frame;
        job.frame_number = m_analysisQueue.front().frame_number;
        m_analysisQueue.pop_front();
    }

    fillAnalysisJob(job);
    if (job.frame && m_pipeEngine)
        m_pipeEngine->releasePreviewFrame(job.frame);
    return true;
}

// Returns deferred analysis buffers with BUFFER_STATUS_ERROR and the
// required CAMERA3_MSG_ERROR_BUFFER notify (flush/close drain path).
void G800FCamera2Device::errorAnalysisJob(AnalysisJob &job)
{
    if (job.bufs.empty()) return;

    if (m_callbackOps != NULL) {
        camera3_notify_msg_t err_msg;
        memset(&err_msg, 0, sizeof(err_msg));
        err_msg.type = CAMERA3_MSG_ERROR;
        err_msg.message.error.frame_number = job.frame_number;
        err_msg.message.error.error_code = CAMERA3_MSG_ERROR_BUFFER;
        err_msg.message.error.error_stream = job.bufs[0].sb.stream;
        m_callbackOps->notify(m_callbackOps, &err_msg);
    }

    std::vector<camera3_stream_buffer_t> sbs(job.bufs.size());
    for (size_t i = 0; i < job.bufs.size(); i++) {
        sbs[i] = job.bufs[i].sb;
        sbs[i].status = CAMERA3_BUFFER_STATUS_ERROR;
        sbs[i].release_fence = -1;
    }
    sendResult(job.frame_number, sbs.data(), sbs.size(), NULL, 0);
}

bool G800FCamera2Device::serveCaptureRequest(QueuedRequest &qr, G800FFrame* previewFrame)
{
    // BLOB stream buffer capacity = width * height bytes (BLOB format = 1 byte/pixel).
    // Standard Android JPEG BLOB: w=maxJpegSize, h=1 → capacity = maxJpegSize.
    // Some apps (e.g. Simple Camera) use w=imageW, h=imageH → capacity = imageW*imageH.
    // The actual image dimensions come from the reprocessing output (SCC_W × SCC_H).
    int blobBufSize = 0;
    if (m_captureStream && m_captureStream->format == HAL_PIXEL_FORMAT_BLOB) {
        blobBufSize = m_captureStream->width * m_captureStream->height;
    }
    // capW/capH for requestStillCapture: the reprocessing target.  The PipeEngine
    // always produces SCC_W × SCC_H (3264 × 2448) regardless of this parameter,
    // but we pass the real capture dimensions for clarity.
    int capW = m_pipeEngine ? m_pipeEngine->getSccWidth()  : (m_captureStream ? m_captureStream->width : 0);
    int capH = m_pipeEngine ? m_pipeEngine->getSccHeight() : (m_captureStream ? m_captureStream->height : 0);

    // Target JPEG dimensions: the BLOB stream dimensions represent the
    // requested JPEG image size.  When height > 1, the app specifies exact
    // image dimensions (e.g. 640×480).  When height == 1, this is the
    // standard maxJpegSize case → use SCC full resolution as image size.
    int jpegW = capW, jpegH = capH;
    if (m_captureStream && m_captureStream->format == HAL_PIXEL_FORMAT_BLOB
        && m_captureStream->height > 1) {
        jpegW = m_captureStream->width;
        jpegH = m_captureStream->height;
    }

    ALOGI("%s: frame %d: still capture %dx%d (jpeg=%dx%d, blobBufSize=%d, flash=%d, async reprocessing)",
          __FUNCTION__, qr.frame_number, capW, capH, jpegW, jpegH, blobBufSize, qr.flashFired ? 1 : 0);

    // Request asynchronous reprocessing (FrameSelector + ISP_REPROC + SCC_REPROC)
    // FLITE continues running, no setSize, no stream off.
    //
    // For flash capture, requestStillCapture() was already called in handleRequest()
    // BEFORE armFlashCapture(), so the FrameSelector captures the flash frame.
    // Here we check if reprocessing is already running.
    status_t err = NO_ERROR;
    if (!m_pipeEngine->isReprocessingActive()) {
        err = m_pipeEngine->requestStillCapture(capW, capH, qr.flashFired);
    }
    if (err != NO_ERROR) {
        ALOGE("%s: requestStillCapture failed: %d — falling back to preview",
              __FUNCTION__, err);
        // Fallback: JPEG from preview frame
        G800FExynosCameraBuffer *camBuf = previewFrame ? previewFrame->buffer : NULL;
        bool hasPreviewData = previewFrame && previewFrame->ownsPreviewData
                              && previewFrame->previewData[0] != NULL;
        if (camBuf || hasPreviewData) {
            int srcW = m_pipeEngine->getPreviewWidth();
            int srcH = m_pipeEngine->getPreviewHeight();
            for (uint32_t i = 0; i < qr.num_output_buffers; i++) {
                camera3_stream_buffer_t *sb = &qr.output_buffers[i];
                buffer_handle_t buf = sb->buffer ? *sb->buffer : NULL;
                bool isCapture = (sb->stream == m_captureStream &&
                                  m_captureStream != NULL &&
                                  m_captureStream->format == HAL_PIXEL_FORMAT_BLOB);
                bool isPreview = (sb->stream == m_previewStream);
                bool isAnalysis = (sb->stream == m_analysisStream && m_analysisStream != NULL);
                bool isVideo = (sb->stream == m_videoStream && m_videoStream != NULL);
                if (buf == NULL) {
                    sb->status = CAMERA3_BUFFER_STATUS_ERROR;
                    sb->release_fence = -1;
                    continue;
                }
                if (isCapture) {
                    if (camBuf) {
                        int jpegOrientation = 0;
                        if (qr.settings != NULL) {
                            camera_metadata_ro_entry_t entry;
                            if (find_camera_metadata_ro_entry(qr.settings,
                                                              ANDROID_JPEG_ORIENTATION,
                                                              &entry) == OK && entry.count > 0)
                                jpegOrientation = entry.data.i32[0];
                        }
                        // Scale to the requested JPEG size via FIMC first —
                        // the HW encoder always encodes at input resolution.
                        G800FExynosCameraBuffer *encBuf = camBuf;
                        int encW = srcW, encH = srcH;
                        G800FExynosCameraBuffer *scaled = scaleForJpeg(
                                camBuf, srcW, srcH, jpegW, jpegH, false);
                        if (scaled) { encBuf = scaled; encW = jpegW; encH = jpegH; }
                        int js = copyCaptureToHwJpeg(encBuf, buf, encW, encH,
                                                     90, jpegOrientation, jpegW, jpegH,
                                                     false, blobBufSize);
                        if (js <= 0) {
                            ALOGI("%s: HW-JPEG failed (%d), falling back to libjpeg",
                                  __FUNCTION__, js);
                            js = copyCaptureToJpeg(camBuf, buf, srcW, srcH,
                                                   90, jpegOrientation, jpegW, jpegH,
                                                   false, blobBufSize);
                        }
                        sb->status = (js > 0) ? CAMERA3_BUFFER_STATUS_OK : CAMERA3_BUFFER_STATUS_ERROR;
                        if (js > 0) m_lastJpegSize = js;
                    } else {
                        // No V4L2 buffer (previewData-only) — cannot produce JPEG
                        sb->status = CAMERA3_BUFFER_STATUS_ERROR;
                    }
                } else if (isPreview || isAnalysis || isVideo) {
                    if (hasPreviewData) {
                        copyPreviewToStreamFromData(previewFrame->previewData[0],
                                                    previewFrame->previewData[1],
                                                    previewFrame->previewStride[0],
                                                    previewFrame->previewHeight,
                                                    buf, sb->stream->width, sb->stream->height);
                    } else {
                        copyPreviewToStream(camBuf, buf, sb->stream->width, sb->stream->height);
                    }
                    sb->status = CAMERA3_BUFFER_STATUS_OK;
                } else {
                    sb->status = CAMERA3_BUFFER_STATUS_ERROR;
                }
                sb->release_fence = -1;
            }
        } else {
            sendErrorResult(qr.frame_number, qr.output_buffers, qr.num_output_buffers);
        }
        if (qr.flashFired && m_pipeEngine) m_pipeEngine->endFlashSequence();
        // Metadata was already sent early (in captureThreadLoop) for ordering.
        // Send buffers only — result=NULL skips the ordering check.
        camera_metadata_t *meta = qr.metadata_sent ? NULL :
            buildResultMetadata(qr, qr.shutter_timestamp, qr.flashFired, previewFrame);
        m_pipeEngine->releasePreviewFrame(previewFrame);
        sendResult(qr.frame_number, qr.output_buffers, qr.num_output_buffers,
                   meta, qr.shutter_timestamp);
        return false;
    }

    // Warm path: the flash-lit frame was also routed through the warm preview
    // ISP[0] → SCC[0], whose firmware applies the flash-weighted AWB at the
    // lit moment (neutral output, Cb/Cr≈128).
    //
    // The match target is the flash-redirect fcount — readable the moment the
    // lit frame enters ISP[1], ~1s BEFORE the slow cold reproc output arrives.
    // Waiting on it (not on getReprocessedFrame) shortens the still capture —
    // and the frozen-preview window for API1 apps — by ~1s.
    G800FFrame* warmFrame = NULL;
    if (qr.flashFired) {
        uint32_t target = m_pipeEngine->waitFlashReprocTarget(2500);
        if (target != 0 &&
                m_pipeEngine->getWarmCaptureFrame(&warmFrame, target, 2000) == NO_ERROR
                && warmFrame) {
            ALOGI("%s: frame %d: warm-path still fcount=%u (SCC[0]/ISP[0], "
                  "redirect target)", __FUNCTION__, qr.frame_number,
                  (unsigned)warmFrame->frameCount);
        }
    }

    // Wait for reprocessed frame (asynchronous, with timeout)
    int reprocTimeoutMs = 10000;
    G800FFrame* reprocFrame = NULL;
    if (warmFrame == NULL) {
    err = m_pipeEngine->getReprocessedFrame(&reprocFrame, reprocTimeoutMs);
    if (err != NO_ERROR || reprocFrame == NULL) {
        ALOGE("%s: getReprocessedFrame failed: %d — falling back to preview",
              __FUNCTION__, err);
        // Fallback: JPEG from preview frame
        G800FExynosCameraBuffer *camBuf = previewFrame ? previewFrame->buffer : NULL;
        bool hasPreviewData = previewFrame && previewFrame->ownsPreviewData
                              && previewFrame->previewData[0] != NULL;
        if (camBuf || hasPreviewData) {
            int srcW = m_pipeEngine->getPreviewWidth();
            int srcH = m_pipeEngine->getPreviewHeight();
            for (uint32_t i = 0; i < qr.num_output_buffers; i++) {
                camera3_stream_buffer_t *sb = &qr.output_buffers[i];
                buffer_handle_t buf = sb->buffer ? *sb->buffer : NULL;
                bool isCapture = (sb->stream == m_captureStream &&
                                  m_captureStream != NULL &&
                                  m_captureStream->format == HAL_PIXEL_FORMAT_BLOB);
                bool isPreview = (sb->stream == m_previewStream);
                bool isAnalysis = (sb->stream == m_analysisStream && m_analysisStream != NULL);
                bool isVideo = (sb->stream == m_videoStream && m_videoStream != NULL);
                if (buf == NULL) {
                    sb->status = CAMERA3_BUFFER_STATUS_ERROR;
                    sb->release_fence = -1;
                    continue;
                }
                if (isCapture) {
                    if (camBuf) {
                        int jpegOrientation = 0;
                        if (qr.settings != NULL) {
                            camera_metadata_ro_entry_t entry;
                            if (find_camera_metadata_ro_entry(qr.settings,
                                                              ANDROID_JPEG_ORIENTATION,
                                                              &entry) == OK && entry.count > 0)
                                jpegOrientation = entry.data.i32[0];
                        }
                        // Scale to the requested JPEG size via FIMC first —
                        // the HW encoder always encodes at input resolution.
                        G800FExynosCameraBuffer *encBuf = camBuf;
                        int encW = srcW, encH = srcH;
                        G800FExynosCameraBuffer *scaled = scaleForJpeg(
                                camBuf, srcW, srcH, jpegW, jpegH, false);
                        if (scaled) { encBuf = scaled; encW = jpegW; encH = jpegH; }
                        int js = copyCaptureToHwJpeg(encBuf, buf, encW, encH,
                                                     90, jpegOrientation, jpegW, jpegH,
                                                     false, blobBufSize);
                        if (js <= 0) {
                            ALOGI("%s: HW-JPEG failed (%d), falling back to libjpeg",
                                  __FUNCTION__, js);
                            js = copyCaptureToJpeg(camBuf, buf, srcW, srcH,
                                                   90, jpegOrientation, jpegW, jpegH,
                                                   false, blobBufSize);
                        }
                        sb->status = (js > 0) ? CAMERA3_BUFFER_STATUS_OK : CAMERA3_BUFFER_STATUS_ERROR;
                        if (js > 0) m_lastJpegSize = js;
                    } else {
                        sb->status = CAMERA3_BUFFER_STATUS_ERROR;
                    }
                } else if (isPreview || isAnalysis || isVideo) {
                    if (hasPreviewData) {
                        copyPreviewToStreamFromData(previewFrame->previewData[0],
                                                    previewFrame->previewData[1],
                                                    previewFrame->previewStride[0],
                                                    previewFrame->previewHeight,
                                                    buf, sb->stream->width, sb->stream->height);
                    } else {
                        copyPreviewToStream(camBuf, buf, sb->stream->width, sb->stream->height);
                    }
                    sb->status = CAMERA3_BUFFER_STATUS_OK;
                } else {
                    sb->status = CAMERA3_BUFFER_STATUS_ERROR;
                }
                sb->release_fence = -1;
            }
        } else {
            sendErrorResult(qr.frame_number, qr.output_buffers, qr.num_output_buffers);
        }
        // Deliver the (fallback) result BEFORE the slow cleanup — the reproc
        // thread join in finishStillCapture() can take ~1s and must not block
        // the capture result, otherwise the app freezes (Snap: preview keeps
        // running but the JPEG never arrives).
        camera_metadata_t *meta = qr.metadata_sent ? NULL :
            buildResultMetadata(qr, qr.shutter_timestamp, qr.flashFired, previewFrame);
        m_pipeEngine->releasePreviewFrame(previewFrame);
        sendResult(qr.frame_number, qr.output_buffers, qr.num_output_buffers,
                   meta, qr.shutter_timestamp);
        m_pipeEngine->finishStillCapture();
        if (qr.flashFired && m_pipeEngine) m_pipeEngine->endFlashSequence();
        return false;
    }

        // Cold path taken (warm fast-path missed): reprocFrame->frameCount is
        // the confirmed lit frame — retry the warm match on it before falling
        // back to the cold buffer (the reproc wait was already paid here).
        if (qr.flashFired &&
                m_pipeEngine->getWarmCaptureFrame(&warmFrame,
                        reprocFrame->frameCount, 800) == NO_ERROR && warmFrame) {
            ALOGI("%s: frame %d: warm-path still fcount=%u (SCC[0]/ISP[0], "
                  "reproc oracle)", __FUNCTION__, qr.frame_number,
                  (unsigned)warmFrame->frameCount);
        }
    }

    // Success: stillFrame = warm substitute (SCC[0]/ISP[0]) or cold reproc.
    G800FFrame* stillFrame = warmFrame ? warmFrame : reprocFrame;
    G800FExynosCameraBuffer *reprocBuf = stillFrame->buffer;
    G800FExynosCameraBuffer *previewBuf = previewFrame ? previewFrame->buffer : NULL;
    bool hasPreviewData = previewFrame && previewFrame->ownsPreviewData
                          && previewFrame->previewData[0] != NULL;
    int reprocW = m_pipeEngine->getSccWidth();   // 3264
    int reprocH = m_pipeEngine->getSccHeight();  // 2448

    // Flash off before JPEG encode
    if (qr.flashFired) {
        if (m_pipeEngine) m_pipeEngine->endFlashSequence();
        ALOGI("%s: frame %d: flash sequence closed before JPEG encode",
              __FUNCTION__, qr.frame_number);
    }

    for (uint32_t i = 0; i < qr.num_output_buffers; i++) {
        camera3_stream_buffer_t *sb = &qr.output_buffers[i];
        buffer_handle_t buf = sb->buffer ? *sb->buffer : NULL;
        bool isCapture = (sb->stream == m_captureStream &&
                          m_captureStream != NULL &&
                          m_captureStream->format == HAL_PIXEL_FORMAT_BLOB);
        bool isPreview = (sb->stream == m_previewStream);
        bool isAnalysis = (sb->stream == m_analysisStream && m_analysisStream != NULL);
        bool isVideo = (sb->stream == m_videoStream && m_videoStream != NULL);

        if (buf == NULL) {
            sb->status = CAMERA3_BUFFER_STATUS_ERROR;
            sb->release_fence = -1;
            continue;
        }

        if (isCapture && reprocBuf != NULL) {
            // JPEG from reprocessed SCC frame (full-res 3264x2448)
            int jpegOrientation = 0;
            if (qr.settings != NULL) {
                camera_metadata_ro_entry_t entry;
                if (find_camera_metadata_ro_entry(qr.settings,
                                                  ANDROID_JPEG_ORIENTATION,
                                                  &entry) == OK && entry.count > 0)
                    jpegOrientation = entry.data.i32[0];
            }
            // SCC[1] reprocessed buffer is YUYV packed (2 bytes/pixel),
            // NOT NV21M — pass isYuyv=true to select the correct conversion.
            // src = reprocW/reprocH (3264x2448, SCC full res).
            // dst = jpegW/jpegH (BLOB stream dimensions = requested JPEG size).
            // blobBufSize = BLOB buffer capacity (stream->width * stream->height).
            // Try HW-JPEG first (YUYV directly, no RGB conversion), fall back
            // to libjpeg if HW encoder fails or rotation is needed.
            // When jpegW/jpegH differ from the reproc size, FIMC-scale the
            // input first — the encoder always encodes at input resolution,
            // so a full-res JPEG would overflow the small BLOB buffer.
            G800FExynosCameraBuffer *encBuf = reprocBuf;
            int encW = reprocW, encH = reprocH;
            G800FExynosCameraBuffer *scaled = scaleForJpeg(
                    reprocBuf, reprocW, reprocH, jpegW, jpegH, true);
            if (scaled) { encBuf = scaled; encW = jpegW; encH = jpegH; }
            int js = copyCaptureToHwJpeg(encBuf, buf, encW, encH,
                                        90, jpegOrientation, jpegW, jpegH,
                                        true, blobBufSize);
            if (js <= 0) {
                ALOGI("%s: HW-JPEG failed (%d), falling back to libjpeg",
                      __FUNCTION__, js);
                js = copyCaptureToJpeg(reprocBuf, buf, reprocW, reprocH,
                                       90, jpegOrientation, jpegW, jpegH,
                                       true, blobBufSize);
            }
            if (js > 0) {
                sb->status = CAMERA3_BUFFER_STATUS_OK;
                m_lastJpegSize = js;
            } else {
                ALOGE("%s: JPEG encoding failed", __FUNCTION__);
                sb->status = CAMERA3_BUFFER_STATUS_ERROR;
                m_lastJpegSize = 0;
            }
        } else if ((isPreview || isAnalysis || isVideo) && (previewBuf != NULL || hasPreviewData)) {
            if (hasPreviewData) {
                copyPreviewToStreamFromData(previewFrame->previewData[0],
                                            previewFrame->previewData[1],
                                            previewFrame->previewStride[0],
                                            previewFrame->previewHeight,
                                            buf, sb->stream->width, sb->stream->height);
            } else {
                copyPreviewToStream(previewBuf, buf, sb->stream->width, sb->stream->height);
            }
            sb->status = CAMERA3_BUFFER_STATUS_OK;
        } else {
            sb->status = CAMERA3_BUFFER_STATUS_ERROR;
        }
        sb->release_fence = -1;
    }

    // Release frames — metadata was already sent early for ordering.
    m_pipeEngine->releasePreviewFrame(previewFrame);
    m_pipeEngine->releaseReprocessingFrame(reprocFrame);
    // Warm path: release the SCC[0] buffer back to the pool (if substituted).
    if (warmFrame) m_pipeEngine->releaseCaptureFrame(warmFrame);

    // Send the result BEFORE finishStillCapture(). The capture sequencer
    // (especially Camera1 API apps like Snap) has a short timeout (~2s) and
    // finishStillCapture() blocks ~1s joining the reproc thread. Sending
    // the result first ensures the app receives the JPEG in time.
    // Metadata was already sent early (in captureThreadLoop) — send buffers
    // only (result=NULL) to skip the out-of-order metadata check.
    camera_metadata_t *meta = qr.metadata_sent ? NULL :
        buildResultMetadata(qr, qr.shutter_timestamp, qr.flashFired, NULL);
    sendResult(qr.frame_number, qr.output_buffers, qr.num_output_buffers,
               meta, qr.shutter_timestamp);

    // Now do the slow cleanup (reproc thread join, FrameSelector reset).
    // This runs after the result is delivered, so the app is unblocked.
    m_pipeEngine->finishStillCapture();

    // Safety net: ensure flash sequence is ended
    if (qr.flashFired && m_pipeEngine) m_pipeEngine->endFlashSequence();

    return true;
}

camera_metadata_t* G800FCamera2Device::buildResultMetadata(
        const QueuedRequest &qr, int64_t timestamp, bool flashFired,
        G800FFrame* previewFrame)
{
    camera_metadata_t *meta;
    if (qr.settings != NULL) {
        size_t entryCount = get_camera_metadata_entry_count(qr.settings);
        size_t dataCount  = get_camera_metadata_data_count(qr.settings);
        meta = allocate_camera_metadata(entryCount + 16, dataCount + 1024);
        if (meta != NULL) {
            int res = append_camera_metadata(meta, qr.settings);
            if (res != 0) {
                ALOGE("%s: append_camera_metadata failed: %d", __FUNCTION__, res);
                free_camera_metadata(meta);
                meta = allocate_camera_metadata(16, 256);
            }
        }
    } else {
        meta = allocate_camera_metadata(16, 256);
    }
    if (meta != NULL) {
        add_camera_metadata_entry(meta, ANDROID_SENSOR_TIMESTAMP, &timestamp, 1);

        // Conformant per-frame metadata: if previewFrame carries valid
        // 3A values (associated from ISP via m_ispToScpQ), use them.
        // Otherwise fall back to the global latest values.
        int frameAfState = (previewFrame && previewFrame->frameCount > 0)
            ? previewFrame->afState : -1;
        int frameAeState = (previewFrame && previewFrame->frameCount > 0)
            ? previewFrame->aeState : -1;
        int frameAwbState = (previewFrame && previewFrame->frameCount > 0)
            ? previewFrame->awbState : -1;
        int frameSensitivity = (previewFrame && previewFrame->frameCount > 0)
            ? previewFrame->sensitivity : 0;
        int64_t frameExposureTime = (previewFrame && previewFrame->frameCount > 0)
            ? previewFrame->exposureTime : 0;

        int32_t afMode = ANDROID_CONTROL_AF_MODE_OFF;
        int32_t afState = ANDROID_CONTROL_AF_STATE_INACTIVE;
        if (m_pipeEngine && m_cameraId == 0) {
            afMode = m_pipeEngine->getLastReqAfMode();
            int dmAfState = (frameAfState >= 0) ? frameAfState
                                                : m_pipeEngine->getLatestAfState();
            // dm.aa.afState uses the kernel's aa_afstate enum (1-based,
            // fimc-is-metadata.h): 1=INACTIVE 2=PASSIVE_SCAN
            // 3=ACTIVE_SCAN 4=ACQUIRED_FOCUS 5=FAILED_FOCUS.
            // ANDROID_CONTROL_AF_STATE is 0-based with different values —
            // map it, and split ACQUIRED/FAILED into LOCKED vs PASSIVE
            // depending on the active afMode (continuous modes use the
            // PASSIVE_* states per the HAL3 spec).
            bool continuous =
                (afMode == ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE ||
                 afMode == ANDROID_CONTROL_AF_MODE_CONTINUOUS_VIDEO);
            switch (dmAfState) {
                case 1:  // AA_AFSTATE_INACTIVE
                    afState = ANDROID_CONTROL_AF_STATE_INACTIVE;
                    break;
                case 2:  // AA_AFSTATE_PASSIVE_SCAN
                    afState = ANDROID_CONTROL_AF_STATE_PASSIVE_SCAN;
                    break;
                case 3:  // AA_AFSTATE_ACTIVE_SCAN
                    afState = ANDROID_CONTROL_AF_STATE_ACTIVE_SCAN;
                    break;
                case 4:  // AA_AFSTATE_AF_ACQUIRED_FOCUS
                    afState = continuous
                            ? ANDROID_CONTROL_AF_STATE_PASSIVE_FOCUSED
                            : ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED;
                    break;
                case 5:  // AA_AFSTATE_AF_FAILED_FOCUS
                    afState = continuous
                            ? ANDROID_CONTROL_AF_STATE_PASSIVE_UNFOCUSED
                            : ANDROID_CONTROL_AF_STATE_NOT_FOCUSED_LOCKED;
                    break;
                default:
                    afState = ANDROID_CONTROL_AF_STATE_INACTIVE;
                    break;
            }
        }
        add_camera_metadata_entry(meta, ANDROID_CONTROL_AF_MODE, &afMode, 1);
        add_camera_metadata_entry(meta, ANDROID_CONTROL_AF_STATE, &afState, 1);

        int32_t aeModeResult = ANDROID_CONTROL_AE_MODE_ON;
        int32_t aeStateResult = ANDROID_CONTROL_AE_STATE_INACTIVE;
        if (m_pipeEngine && m_cameraId == 0) {
            aeModeResult = ANDROID_CONTROL_AE_MODE_ON;
            int dmAeState = (frameAeState >= 0) ? frameAeState
                                                : m_pipeEngine->getLatestAeState();
            if (dmAeState >= 1 && dmAeState <= 6)
                aeStateResult = dmAeState - 1;
            // While our flash sequence is metering (pre-flash AE/AF converge,
            // state < READY) report AE_STATE=PRECAPTURE.  The API1
            // CaptureSequencer only holds the still request while AE is in
            // PRECAPTURE — if it sees CONVERGED/FLASH_REQUIRED early it submits
            // the still immediately and our ~1.6s metering then runs inside the
            // capture's hard 4s budget (kMaxTimeoutsForCaptureEnd) → timeout →
            // frozen preview.  Reporting PRECAPTURE moves the metering into the
            // precapture phase so the still only has to arm+fire+encode.
            if (m_pipeEngine->isFlashSequenceActive() &&
                    m_pipeEngine->getFlashSeqState() < G800FPipeEngine::FLASH_SEQ_READY)
                aeStateResult = ANDROID_CONTROL_AE_STATE_PRECAPTURE;
        }
        add_camera_metadata_entry(meta, ANDROID_CONTROL_AE_MODE, &aeModeResult, 1);
        add_camera_metadata_entry(meta, ANDROID_CONTROL_AE_STATE, &aeStateResult, 1);

        int32_t awbStateResult = ANDROID_CONTROL_AWB_STATE_INACTIVE;
        if (m_pipeEngine && m_cameraId == 0) {
            int dmAwbState = (frameAwbState >= 0) ? frameAwbState
                                                  : m_pipeEngine->getLatestAwbState();
            if (dmAwbState >= 1 && dmAwbState <= 4)
                awbStateResult = dmAwbState - 1;
        }
        add_camera_metadata_entry(meta, ANDROID_CONTROL_AWB_STATE, &awbStateResult, 1);

        int32_t flashStateResult = ANDROID_FLASH_STATE_UNAVAILABLE;
        if (m_pipeEngine && m_cameraId == 0) {
            if (flashFired)
                flashStateResult = ANDROID_FLASH_STATE_FIRED;
            else
                flashStateResult = ANDROID_FLASH_STATE_READY;
        }
        add_camera_metadata_entry(meta, ANDROID_FLASH_STATE, &flashStateResult, 1);

        int32_t sensorSensitivity = (frameSensitivity > 0)
            ? frameSensitivity
            : (m_pipeEngine ? m_pipeEngine->getLatestSensitivity() : 0);
        int64_t sensorExposureTime = (frameExposureTime > 0)
            ? frameExposureTime
            : (m_pipeEngine ? m_pipeEngine->getLatestExposureTime() : 0);
        if (sensorSensitivity > 0)
            add_camera_metadata_entry(meta, ANDROID_SENSOR_SENSITIVITY, &sensorSensitivity, 1);
        if (sensorExposureTime > 0)
            add_camera_metadata_entry(meta, ANDROID_SENSOR_EXPOSURE_TIME, &sensorExposureTime, 1);
        if (m_lastJpegSize > 0) {
            int res = add_camera_metadata_entry(meta, ANDROID_JPEG_SIZE,
                                                &m_lastJpegSize, 1);
            if (res != 0)
                ALOGE("%s: add ANDROID_JPEG_SIZE failed: %d", __FUNCTION__, res);
            m_lastJpegSize = 0;
        }
    }
    return meta;
}

// --- Flash: delegation to G800FPipeEngine ---
// The full 7-state flash state machine (IDLE→START→ON→METERING→
// READY→CAPTURE→DONE→IDLE) runs in G800FPipeEngine. The FLITE thread
// calls advanceFlashSequence() per frame. The device only calls
// beginFlashSequence(), pumpPreviewWhileFlashMetering(), armFlashCapture()
// and endFlashSequence() on the pipe engine.

int G800FCamera2Device::pumpPreviewWhileFlashMetering(int timeoutMs)
{
    // Waits until the flash state machine is READY (AE converged).
    //
    // NOTE: do NOT drain m_previewQ here!  An earlier version of this
    // function popped and released preview frames in a tight ~5ms loop
    // to "keep the kernel queue rotating".  That stole every SCP frame
    // from captureThreadLoop, which serves preview requests from the
    // same queue → preview froze for the whole ~1.8s torch/preflash
    // metering phase.
    //
    // Draining is unnecessary anyway: m_previewQ has overflow
    // protection (drops the oldest frame at capacity), so the SCP
    // V4L2 buffers keep cycling even if no preview request is in
    // flight.  When preview requests do arrive, captureThreadLoop
    // pops them from m_previewQ itself — that is the whole point of
    // the async capture worker design.
    nsecs_t start = systemTime(SYSTEM_TIME_MONOTONIC);
    bool ready = m_pipeEngine
        ? m_pipeEngine->waitFlashSequenceReady(timeoutMs)
        : false;

    int totalMs = (int)((systemTime(SYSTEM_TIME_MONOTONIC) - start) / 1000000);
    if (m_pipeEngine) {
        ALOGI("FLASHPUMP: metering wait %dms — ready=%d state=%s aeState=%d flashReady=%d",
              totalMs, (int)ready,
              G800FPipeEngine::flashSeqStateName(m_pipeEngine->getFlashSeqState()),
              m_pipeEngine->getLatestAeState(), m_pipeEngine->getLatestFlashReady());
        if (!ready)
            ALOGW("FLASHPUMP: metering wait timed out / sequence died (state=%s)",
                  G800FPipeEngine::flashSeqStateName(m_pipeEngine->getFlashSeqState()));
    }
    return totalMs;
}

int G800FCamera2Device::copyCaptureToJpeg(G800FExynosCameraBuffer *camBuf,
                                           buffer_handle_t outBuf,
                                           int srcW, int srcH,
                                           int quality, int rotation,
                                           int dstW, int dstH,
                                           bool isYuyv,
                                           int blobBufSize)
{
    if (camBuf == NULL || outBuf == NULL)
        return -1;

    // Lock the BLOB buffer (single-plane, SW write)
    const gralloc_module_t *grmod = NULL;
    hw_module_t const *hwmod = NULL;
    if (hw_get_module(GRALLOC_HARDWARE_MODULE_ID, &hwmod) != 0) {
        ALOGE("%s: cannot get gralloc module", __FUNCTION__);
        return -1;
    }
    grmod = reinterpret_cast<const gralloc_module_t*>(hwmod);

    // BLOB buffer capacity = blobBufSize (caller passes stream->width * stream->height
    // for HAL_PIXEL_FORMAT_BLOB, since BLOB format = 1 byte/pixel).
    // If not provided (0), fall back to a safe upper bound from target dimensions.
    int blobSize = (blobBufSize > 0)
        ? blobBufSize
        : (dstW * dstH * 3);  // fallback: worst-case JPEG size

    void *vaddr = NULL;
    int err = grmod->lock(grmod, outBuf,
                          GRALLOC_USAGE_SW_WRITE_OFTEN,
                          0, 0, blobSize, 1, &vaddr);
    if (err != 0 || vaddr == NULL) {
        ALOGE("%s: gralloc_lock failed: %d vaddr=%p", __FUNCTION__, err, vaddr);
        return -1;
    }
    ALOGI("%s: locked BLOB buf=%p vaddr=%p blobSize=%d isYuyv=%d",
          __FUNCTION__, outBuf, vaddr, blobSize, isYuyv);

    // Source buffer access
    uint8_t *srcPlane0 = reinterpret_cast<uint8_t*>(camBuf->planeVaddr(0));
    if (srcPlane0 == NULL) {
        ALOGE("%s: null plane 0", __FUNCTION__);
        grmod->unlock(grmod, outBuf);
        return -1;
    }

    // Convert to RGB at source resolution, then scale to target.
    // libjpeg writes directly into the BLOB buffer (vaddr), which is
    // what the Camera2 framework expects.  The HW JPEG encoder
    // (ExynosJpegEncoder) uses MMAP/DMA_BUF internally and does NOT
    // write to a userptr destination, so it cannot be used directly
    // with a gralloc BLOB buffer.
    int srcRgbSize = srcW * srcH * 3;
    uint8_t *srcRgb = (uint8_t *)malloc(srcRgbSize);
    if (srcRgb == NULL) {
        ALOGE("%s: cannot allocate src RGB buffer", __FUNCTION__);
        grmod->unlock(grmod, outBuf);
        return -1;
    }

    if (isYuyv) {
        // YUYV packed: Y0 U0 Y1 V0 Y2 U2 ... (2 bytes/pixel, all in plane 0)
        // plane 1 = SPARE metadata, NOT chroma — do not access it as VU.
        // Each pair of pixels shares one (U,V) pair.
        int yuyvStride = srcW * 2;
        for (int row = 0; row < srcH; row++) {
            uint8_t *yuyvRow = srcPlane0 + row * yuyvStride;
            for (int col = 0; col < srcW; col += 2) {
                int y0 = yuyvRow[col * 2];
                int u  = yuyvRow[col * 2 + 1];
                int y1 = yuyvRow[col * 2 + 2];
                int v  = yuyvRow[col * 2 + 3];
                int d = u - 128;
                int e = v - 128;
                int r = y0 + ((180 * e + 64) >> 7);
                int g = y0 - ((44 * d + 91 * e + 64) >> 7);
                int b = y0 + ((227 * d + 64) >> 7);
                int off = (row * srcW + col) * 3;
                srcRgb[off]     = (uint8_t)clamp(r);
                srcRgb[off + 1] = (uint8_t)clamp(g);
                srcRgb[off + 2] = (uint8_t)clamp(b);
                if (col + 1 < srcW) {
                    r = y1 + ((180 * e + 64) >> 7);
                    g = y1 - ((44 * d + 91 * e + 64) >> 7);
                    b = y1 + ((227 * d + 64) >> 7);
                    srcRgb[off + 3] = (uint8_t)clamp(r);
                    srcRgb[off + 4] = (uint8_t)clamp(g);
                    srcRgb[off + 5] = (uint8_t)clamp(b);
                }
            }
        }
    } else {
        // NV21M: Y plane (plane 0) + interleaved VU (plane 1)
        uint8_t *srcY  = srcPlane0;
        uint8_t *srcVU = reinterpret_cast<uint8_t*>(camBuf->planeVaddr(1));
        if (srcVU == NULL) {
            ALOGE("%s: null NV21M VU plane", __FUNCTION__);
            free(srcRgb);
            grmod->unlock(grmod, outBuf);
            return -1;
        }
        // NV21M → RGB conversion (full-range BT601, same as preview)
        int srcYStride = srcW;
        int srcCStride = srcW;
        for (int row = 0; row < srcH; row++) {
            for (int col = 0; col < srcW; col++) {
                int y = srcY[row * srcYStride + col];
                int uvIdx = (row >> 1) * srcCStride + (col & ~1);
                int v = srcVU[uvIdx];
                int u = srcVU[uvIdx + 1];
                int d = u - 128;
                int e = v - 128;
                int r = y + ((180 * e + 64) >> 7);
                int g = y - ((44 * d + 91 * e + 64) >> 7);
                int b = y + ((227 * d + 64) >> 7);
                int off = (row * srcW + col) * 3;
                srcRgb[off]     = (uint8_t)clamp(r);
                srcRgb[off + 1] = (uint8_t)clamp(g);
                srcRgb[off + 2] = (uint8_t)clamp(b);
            }
        }
    }

    // Scale to target dimensions if different from source.
    // Uses nearest-neighbor for speed (bilinear would be better quality
    // but slower on Cortex-A7 without NEON optimization).
    uint8_t *rgb = srcRgb;
    int rgbW = srcW, rgbH = srcH;
    if (dstW != srcW || dstH != srcH) {
        int dstRgbSize = dstW * dstH * 3;
        uint8_t *dstRgb = (uint8_t *)malloc(dstRgbSize);
        if (dstRgb == NULL) {
            ALOGE("%s: cannot allocate dst RGB buffer %dx%d", __FUNCTION__, dstW, dstH);
            free(srcRgb);
            grmod->unlock(grmod, outBuf);
            return -1;
        }
        for (int dy = 0; dy < dstH; dy++) {
            int sy = (dy * srcH) / dstH;
            if (sy >= srcH) sy = srcH - 1;
            for (int dx = 0; dx < dstW; dx++) {
                int sx = (dx * srcW) / dstW;
                if (sx >= srcW) sx = srcW - 1;
                int srcOff = (sy * srcW + sx) * 3;
                int dstOff = (dy * dstW + dx) * 3;
                dstRgb[dstOff]     = srcRgb[srcOff];
                dstRgb[dstOff + 1] = srcRgb[srcOff + 1];
                dstRgb[dstOff + 2] = srcRgb[srcOff + 2];
            }
        }
        free(srcRgb);
        rgb = dstRgb;
        rgbW = dstW;
        rgbH = dstH;
    }

    // Apply JPEG rotation (ANDROID_JPEG_ORIENTATION).
    // The sensor outputs landscape (width > height).  The framework requests
    // rotation to match the device orientation.  We rotate the RGB buffer
    // in-place before JPEG compression.
    int rotW = rgbW, rotH = rgbH;
    if (rotation == 90 || rotation == 270) {
        rotW = rgbH;
        rotH = rgbW;
    }
    if (rotation != 0) {
        uint8_t *rotRgb = (uint8_t *)malloc(rotW * rotH * 3);
        if (rotRgb != NULL) {
            for (int dstRow = 0; dstRow < rotH; dstRow++) {
                for (int dstCol = 0; dstCol < rotW; dstCol++) {
                    int srcRow, srcCol;
                    if (rotation == 90) {
                        // 90° CW: dst(dstRow,dstCol) ← src(rgbH-1-dstCol, dstRow)
                        srcRow = rgbH - 1 - dstCol;
                        srcCol = dstRow;
                    } else if (rotation == 180) {
                        srcRow = rgbH - 1 - dstRow;
                        srcCol = rgbW - 1 - dstCol;
                    } else { // 270
                        // 270° CW: dst(dstRow,dstCol) ← src(dstCol, rgbW-1-dstRow)
                        srcRow = dstCol;
                        srcCol = rgbW - 1 - dstRow;
                    }
                    int srcOff = (srcRow * rgbW + srcCol) * 3;
                    int dstOff = (dstRow * rotW + dstCol) * 3;
                    rotRgb[dstOff]     = rgb[srcOff];
                    rotRgb[dstOff + 1] = rgb[srcOff + 1];
                    rotRgb[dstOff + 2] = rgb[srcOff + 2];
                }
            }
            free(rgb);
            rgb = rotRgb;
        }
    }

    // JPEG compress using libjpeg.
    // jpeg_mem_dest() always allocates its own internal buffer when called
    // with NULL pointers.  We then memcpy the result into the gralloc BLOB
    // buffer (vaddr), which is what the Camera2 framework reads.
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    unsigned char *jpegBuf = NULL;
    unsigned long jpegSize = 0;
    jpeg_mem_dest(&cinfo, &jpegBuf, &jpegSize);

    cinfo.image_width = rotW;
    cinfo.image_height = rotH;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    cinfo.density_unit = 1;  // DPI
    cinfo.X_density = 72;
    cinfo.Y_density = 72;

    jpeg_start_compress(&cinfo, TRUE);

    JSAMPROW rowPtr[1];
    while (cinfo.next_scanline < cinfo.image_height) {
        rowPtr[0] = &rgb[cinfo.next_scanline * rotW * 3];
        jpeg_write_scanlines(&cinfo, rowPtr, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    // Copy the JPEG data from libjpeg's internal buffer into the BLOB buffer
    if (jpegBuf != NULL && jpegSize > 0 && jpegSize <= (unsigned long)blobSize) {
        memcpy(vaddr, jpegBuf, jpegSize);
        ALOGI("%s: copied %lu bytes from libjpeg buf=%p to vaddr=%p",
              __FUNCTION__, jpegSize, jpegBuf, vaddr);
    } else {
        ALOGE("%s: JPEG encode produced invalid result: jpegBuf=%p jpegSize=%lu blobSize=%d",
              __FUNCTION__, jpegBuf, jpegSize, blobSize);
        jpegSize = 0;
    }

    // Free libjpeg's internal buffer
    free(jpegBuf);

    free(rgb);
    grmod->unlock(grmod, outBuf);

    ALOGI("%s: JPEG src=%dx%d dst=%dx%d rot=%d → %lux%lu q=%d → %lu bytes",
          __FUNCTION__, srcW, srcH, dstW, dstH, rotation,
          (unsigned long)cinfo.image_width, (unsigned long)cinfo.image_height,
          quality, jpegSize);
    return (int)jpegSize;
}

/* ============================================================================
 * scaleForJpeg — FIMC M2M downscale before HW JPEG encoding.
 *
 * The HW JPEG encoder always encodes at input resolution and cannot scale.
 * When the app requests a JPEG smaller than the sensor output (e.g. Snap
 * picture-size 640x480 while the reproc path produces 3264x2448), the
 * full-res JPEG would exceed the BLOB buffer capacity (w*h bytes) and the
 * whole capture falls back to ~2.6s of libjpeg — blowing past the
 * Camera2Client CaptureSequencer's 4s timeout and freezing the preview.
 *
 * Scaling the YUV input via FIMC (node 3 — nodes 0/1/2 are used by the
 * preview/video/analysis paths) makes the encoder output fit the blob
 * and cuts the encode time roughly proportionally to the pixel count.
 * ============================================================================ */
G800FExynosCameraBuffer* G800FCamera2Device::scaleForJpeg(
        G800FExynosCameraBuffer *src, int srcW, int srcH,
        int dstW, int dstH, bool isYuyv)
{
    if (src == NULL || dstW <= 0 || dstH <= 0)
        return NULL;
    if (srcW == dstW && srcH == dstH)
        return NULL;  // no scaling needed
    if (m_stillFimcFailed)
        return NULL;

    int srcFdY = src->dmaBufFd(0);
    int srcFdVU = isYuyv ? -1 : src->dmaBufFd(1);
    if (srcFdY < 0 || (!isYuyv && srcFdVU < 0))
        return NULL;  // no dmabuf — cannot feed FIMC

    if (m_stillScaler == NULL) {
        m_stillScaler = new G800FFimcScaler();
        if (m_stillScaler->open(3) != NO_ERROR) {
            ALOGE("%s: FIMC node 3 open failed", __FUNCTION__);
            delete m_stillScaler;
            m_stillScaler = NULL;
            m_stillFimcFailed = true;
            return NULL;
        }
    }

    if (m_stillScaleBuf == NULL ||
            m_stillScaleBufW != dstW || m_stillScaleBufH != dstH) {
        delete m_stillScaleBuf;
        m_stillScaleBuf = new G800FExynosCameraBuffer();
        size_t planeSize = isYuyv ? (size_t)dstW * dstH * 2
                                  : (size_t)dstW * dstH * 3 / 2;
        if (m_stillScaleBuf->allocDmabuf(1, &planeSize, (1 << 5), 0)
                != NO_ERROR) {
            ALOGE("%s: scale dst buffer alloc failed (%dx%d)",
                  __FUNCTION__, dstW, dstH);
            delete m_stillScaleBuf;
            m_stillScaleBuf = NULL;
            m_stillFimcFailed = true;
            return NULL;
        }
        m_stillScaleBufW = dstW;
        m_stillScaleBufH = dstH;
    }

    // Reconfigure whenever src/dst geometry or format changed.
    unsigned int srcFmt = isYuyv ? V4L2_PIX_FMT_YUYV : V4L2_PIX_FMT_NV21M;
    unsigned int dstFmt = isYuyv ? V4L2_PIX_FMT_YUYV : V4L2_PIX_FMT_NV21;
    if (!m_stillScaler->isConfigured() ||
            m_stillScaleSrcW   != srcW || m_stillScaleSrcH   != srcH ||
            m_stillScaleSrcFmt != srcFmt ||
            m_stillScaler->getDstWidth()  != dstW ||
            m_stillScaler->getDstHeight() != dstH) {
        if (m_stillScaler->configure(srcW, srcH, srcFmt,
                                     dstW, dstH, dstFmt) != NO_ERROR) {
            ALOGE("%s: FIMC configure %dx%d→%dx%d failed",
                  __FUNCTION__, srcW, srcH, dstW, dstH);
            m_stillFimcFailed = true;
            return NULL;
        }
        m_stillScaleSrcW   = srcW;
        m_stillScaleSrcH   = srcH;
        m_stillScaleSrcFmt = srcFmt;
    }

    int64_t t0 = systemTime(CLOCK_MONOTONIC);
    status_t err = m_stillScaler->processFrame(srcFdY, srcFdVU,
                                               m_stillScaleBuf->dmaBufFd(0));
    int64_t t1 = systemTime(CLOCK_MONOTONIC);
    if (err != NO_ERROR) {
        ALOGE("%s: FIMC processFrame failed: %d", __FUNCTION__, err);
        return NULL;  // not fatal — caller falls back to unscaled input
    }
    ALOGI("%s: FIMC scaled %dx%d→%dx%d (%s) in %.1f ms",
          __FUNCTION__, srcW, srcH, dstW, dstH, isYuyv ? "YUYV" : "NV21",
          (t1 - t0) / 1000000.0);
    return m_stillScaleBuf;
}

/* ============================================================================
 * HW JPEG encoding via ExynosJpegEncoder (/dev/video12).
 *
 * The Exynos 3470 has a hardware JPEG encoder at /dev/video12.
 * It accepts YUYV (V4L2_PIX_FMT_YUYV) and NV12/NV21 directly as dmabuf —
 * no RGB conversion needed! The firmware does JPEG encoding in
 * hardware, which is ~10-50× faster than libjpeg on the Cortex-A7.
 *
 * Architecture:
 *   Input:  SCC buffer (YUYV, dmabuf fd) or preview buffer (NV21M, dmabuf fd)
 *   Output: ION buffer (for JPEG), then memcpy into gralloc BLOB
 *
 * Rotation: The HW encoder does not support rotation. Instead of expensive
 *   pixel rotation (2620ms for 8MP YUYV on Cortex-A7), the JPEG encoder is
 *   called without rotation and afterwards an EXIF APP1 segment with the
 *   orientation tag is inserted. Gallery apps rotate automatically based
 *   on the EXIF tag. This saves ~2.6s per photo.
 *
 * EXIF orientation mapping (Android JPEG_ORIENTATION → EXIF):
 *   0°  → 1 (normal)
 *   90° → 6 (90° CW)
 *   180°→ 3 (180°)
 *   270°→ 8 (90° CCW)
 * ============================================================================ */
int G800FCamera2Device::copyCaptureToHwJpeg(G800FExynosCameraBuffer *camBuf,
                                             buffer_handle_t outBuf,
                                             int srcW, int srcH,
                                             int quality, int rotation,
                                             int dstW, int dstH,
                                             bool isYuyv,
                                             int blobBufSize)
{
    if (camBuf == NULL || outBuf == NULL)
        return -1;

    // Check if the source buffer already has a DMABUF fd — the SCC_REPROC
    // buffers are allocated via allocDmabuf (ION_HEAP_EXYNOS), so after
    // dqbuf the fd is valid and synced for CPU.  Passing it directly to
    // the JPEG encoder avoids the ~360ms CPU memcpy of ~16MB.
    int srcFd = camBuf->dmaBufFd(0);
    int srcPlaneSize = camBuf->planeSize(0);
    bool directDmabuf = (srcFd >= 0 && srcPlaneSize > 0);

    uint8_t *srcVaddr = NULL;
    int inSize = 0;
    if (!directDmabuf) {
        // Fallback path: MMAP buffer — need vaddr for memcpy into ION buffer.
        srcVaddr = reinterpret_cast<uint8_t*>(camBuf->planeVaddr(0));
        if (srcVaddr == NULL) {
            ALOGW("%s: no planeVaddr on camBuf plane 0, falling back to libjpeg",
                  __FUNCTION__);
            return -1;
        }
        inSize = srcPlaneSize;
        if (inSize <= 0) {
            ALOGW("%s: invalid plane 0 size %d, falling back", __FUNCTION__, inSize);
            return -1;
        }
    }

    // Derive EXIF orientation from Android rotation.
    // The HW encoder cannot rotate — instead we set the
    // EXIF orientation tag, which gallery apps use for display rotation.
    int exifOrientation = 1;  // 1 = normal
    switch (rotation) {
        case 0:   exifOrientation = 1; break;  // normal
        case 90:  exifOrientation = 6; break;  // 90° CW
        case 180: exifOrientation = 3; break;  // 180°
        case 270: exifOrientation = 8; break;  // 90° CCW
        default:  exifOrientation = 1; break;
    }

    // ION client and buffers for input and JPEG output.
    ion_client ionClient = -1;
    ion_buffer ionInBuf = -1;    // ION buffer for YUV input (copy from camBuf)
    char *ionInVaddr = NULL;
    ion_buffer ionOutBuf = -1;   // ION buffer for JPEG output
    char *ionOutVaddr = NULL;

    // Lock BLOB buffer (for JPEG output copy).
    const gralloc_module_t *grmod = NULL;
    hw_module_t const *hwmod = NULL;
    if (hw_get_module(GRALLOC_HARDWARE_MODULE_ID, &hwmod) != 0) {
        ALOGE("%s: cannot get gralloc module", __FUNCTION__);
        return -1;
    }
    grmod = reinterpret_cast<const gralloc_module_t*>(hwmod);

    int blobSize = (blobBufSize > 0) ? blobBufSize : (dstW * dstH * 3);
    void *blobVaddr = NULL;
    int err = grmod->lock(grmod, outBuf,
                          GRALLOC_USAGE_SW_WRITE_OFTEN,
                          0, 0, blobSize, 1, &blobVaddr);
    if (err != 0 || blobVaddr == NULL) {
        ALOGE("%s: gralloc_lock BLOB failed: %d", __FUNCTION__, err);
        return -1;
    }

    // Create ION client.
    ionClient = ion_client_create();
    if (ionClient < 0) {
        ALOGE("%s: ion_client_create failed: %d", __FUNCTION__, ionClient);
        grmod->unlock(grmod, outBuf);
        return -1;
    }

    // Input size must match the kernel queue_setup expectation:
    //   sizes[i] = in_width * in_height * in_depth / 8
    // For YUYV (depth=16): srcW * srcH * 2
    // For NV21 (depth=12): srcW * srcH * 3/2
    int inBufSize;
    if (isYuyv) {
        inBufSize = srcW * srcH * 2;  // YUYV: 16 bits/pixel
    } else {
        inBufSize = (srcW * srcH * 3) / 2;  // NV21: 12 bits/pixel
    }
    if (inBufSize < srcPlaneSize) {
        inBufSize = srcPlaneSize;
    }
    inBufSize = (inBufSize + 4095) & ~4095;

    // For direct DMABUF, inFd is the source buffer's fd — no ION alloc needed.
    // For the memcpy fallback, we allocate our own ION buffer.
    int inFd = -1;

    if (directDmabuf) {
        // Verify the DMABUF is large enough for the kernel's expectation.
        int expectedSize = isYuyv ? srcW * srcH * 2 : (srcW * srcH * 3) / 2;
        if (srcPlaneSize < expectedSize) {
            ALOGW("%s: DMABUF planeSize %d < expected %d, falling back to memcpy",
                  __FUNCTION__, srcPlaneSize, expectedSize);
            directDmabuf = false;
            srcVaddr = reinterpret_cast<uint8_t*>(camBuf->planeVaddr(0));
            if (srcVaddr == NULL) {
                ion_client_destroy(ionClient);
                grmod->unlock(grmod, outBuf);
                return -1;
            }
            inSize = srcPlaneSize;
        } else {
            inFd = srcFd;
            inBufSize = srcPlaneSize;
            //ALOGI("%s: direct DMABUF fd=%d size=%d — no CPU copy",
            //      __FUNCTION__, srcFd, srcPlaneSize);
        }
    }

    if (!directDmabuf) {
        // Allocate ION input buffer and copy YUV data (fallback path).
        //ALOGI("%s: inBufSize=%d (kernel expects %d for %dx%d %s, camBuf=%d)",
        //      __FUNCTION__, inBufSize, isYuyv ? srcW*srcH*2 : (srcW*srcH*3)/2,
        //      srcW, srcH, isYuyv ? "YUYV" : "NV21", inSize);

        ionInBuf = ion_alloc(ionClient, inBufSize, 0,
                              ION_HEAP_SYSTEM_MASK, 0);
        if (ionInBuf == -1 || ionInBuf == 0) {
            ALOGE("%s: ion_alloc(%d) for YUV input failed", __FUNCTION__, inBufSize);
            ion_client_destroy(ionClient);
            grmod->unlock(grmod, outBuf);
            return -1;
        }
        ionInVaddr = (char *)ion_map(ionInBuf, inBufSize, 0);
        if (ionInVaddr == (char *)MAP_FAILED || ionInVaddr == NULL) {
            ALOGE("%s: ion_map YUV input failed", __FUNCTION__);
            ion_free(ionInBuf);
            ionInBuf = -1;
            ion_client_destroy(ionClient);
            grmod->unlock(grmod, outBuf);
            return -1;
        }

        // Copy YUV data from camera buffer to ION buffer.
        memcpy(ionInVaddr, srcVaddr, inSize);
        inFd = ionInBuf;
    }

    // Allocate ION output buffer for the JPEG encoder.
    // Kernel queue_setup for CAPTURE: sizes[i] = out_width * out_height * out_depth * 2 / 8
    // The kernel sets out_width=height, out_height=width (swapped in s_fmt_cap).
    // For JPEG_422/444/420/GRAY, out_depth=8, so:
    //   sizes[0] = srcW * srcH * 8 * 2 / 8 = srcW * srcH * 2
    // This is the SAME SIZE as the input buffer (YUYV: srcW*srcH*2)!
    // The kernel thus reserves double the storage for the JPEG output,
    // even if the actual JPEG is much smaller.
    int jpegBufSize = srcW * srcH * 2;  // Kernel expects srcW*srcH*2 for CAPTURE
    if (jpegBufSize < blobSize) jpegBufSize = blobSize;
    jpegBufSize = (jpegBufSize + 4095) & ~4095;
    //ALOGI("%s: jpegBufSize=%d (kernel CAPTURE expects %d for %dx%d JPEG_out)",
    //      __FUNCTION__, jpegBufSize, srcW*srcH*2, srcW, srcH);

    ionOutBuf = ion_alloc(ionClient, jpegBufSize, 0,
                          ION_HEAP_SYSTEM_MASK, 0);
    if (ionOutBuf == -1 || ionOutBuf == 0) {
        ALOGE("%s: ion_alloc(%d) for JPEG output failed", __FUNCTION__, jpegBufSize);
        if (ionInVaddr) ion_unmap(ionInVaddr, inBufSize);
        if (ionInBuf >= 0) ion_free(ionInBuf);
        ion_client_destroy(ionClient);
        grmod->unlock(grmod, outBuf);
        return -1;
    }

    ionOutVaddr = (char *)ion_map(ionOutBuf, jpegBufSize, 0);
    if (ionOutVaddr == (char *)MAP_FAILED || ionOutVaddr == NULL) {
        ALOGE("%s: ion_map JPEG output failed", __FUNCTION__);
        ion_free(ionOutBuf);
        if (ionInVaddr) ion_unmap(ionInVaddr, inBufSize);
        if (ionInBuf >= 0) ion_free(ionInBuf);
        ion_client_destroy(ionClient);
        grmod->unlock(grmod, outBuf);
        return -1;
    }

    ALOGI("%s: HW-JPEG DMABUF src=%dx%d rot=%d(exif=%d) isYuyv=%d q=%d "
          "in{fd=%d,size=%d,direct=%d} ionOut{fd=%d,size=%d} blobSize=%d",
          __FUNCTION__, srcW, srcH, rotation, exifOrientation, isYuyv, quality,
          inFd, inBufSize, (int)directDmabuf, ionOutBuf, jpegBufSize, blobSize);

    // Create and configure the ExynosJpegEncoder.
    ExynosJpegEncoder enc;
    int ret = enc.create();
    if (ret != 0) {
        ALOGE("%s: ExynosJpegEncoder::create() failed: %d", __FUNCTION__, ret);
        ion_unmap(ionOutVaddr, jpegBufSize);
        ion_free(ionOutBuf);
        if (ionInVaddr) ion_unmap(ionInVaddr, inBufSize);
        if (ionInBuf >= 0) ion_free(ionInBuf);
        ion_client_destroy(ionClient);
        grmod->unlock(grmod, outBuf);
        return -1;
    }

    // Configure encoder with original dimensions (no rotation).
    enc.setSize(srcW, srcH);
    enc.setQuality(quality);
    if (isYuyv) {
        enc.setColorFormat(V4L2_PIX_FMT_YUYV);
        enc.setJpegFormat(V4L2_PIX_FMT_JPEG_422);
    } else {
        enc.setColorFormat(V4L2_PIX_FMT_NV21);
        enc.setJpegFormat(V4L2_PIX_FMT_JPEG_420);
    }

    // Set input buffer (DMABUF).
    // For direct DMABUF, inFd is the SCC_REPROC buffer's own dma_buf fd —
    // the JPEG driver imports it via VB2_DMABUF.  No CPU copy needed.
    int inFds[1] = { inFd };
    int inSizes[1] = { inBufSize };
    ret = enc.setInBuf(inFds, inSizes);
    if (ret != 0) {
        ALOGE("%s: setInBuf(DMABUF) failed: %d", __FUNCTION__, ret);
        enc.destroy();
        ion_unmap(ionOutVaddr, jpegBufSize);
        ion_free(ionOutBuf);
        if (ionInVaddr) ion_unmap(ionInVaddr, inBufSize);
        if (ionInBuf >= 0) ion_free(ionInBuf);
        ion_client_destroy(ionClient);
        grmod->unlock(grmod, outBuf);
        return -1;
    }

    // Set output buffer (DMABUF for JPEG output).
    // The JPEG encoder output is set via setOutBuf(jpegBuf.fd[0], jpegBuf.size[0]+[1]+[2])
    ret = enc.setOutBuf(ionOutBuf, jpegBufSize);
    if (ret != 0) {
        ALOGE("%s: setOutBuf(DMABUF) failed: %d", __FUNCTION__, ret);
        enc.destroy();
        ion_unmap(ionOutVaddr, jpegBufSize);
        ion_free(ionOutBuf);
        if (ionInVaddr) ion_unmap(ionInVaddr, inBufSize);
        if (ionInBuf >= 0) ion_free(ionInBuf);
        ion_client_destroy(ionClient);
        grmod->unlock(grmod, outBuf);
        return -1;
    }

    ret = enc.updateConfig();
    if (ret != 0) {
        ALOGE("%s: updateConfig failed: %d", __FUNCTION__, ret);
        enc.destroy();
        ion_unmap(ionOutVaddr, jpegBufSize);
        ion_free(ionOutBuf);
        if (ionInVaddr) ion_unmap(ionInVaddr, inBufSize);
        if (ionInBuf >= 0) ion_free(ionInBuf);
        ion_client_destroy(ionClient);
        grmod->unlock(grmod, outBuf);
        return -1;
    }

    // Encode!
    int64_t t0 = systemTime(CLOCK_MONOTONIC);
    ret = enc.encode();
    int64_t t1 = systemTime(CLOCK_MONOTONIC);
    double encodeMs = (t1 - t0) / 1000000.0;

    if (ret != 0) {
        ALOGE("%s: HW-JPEG encode() failed: %d (%.1f ms) — kernel expects "
              "in_size=%d (%dx%d YUYV), out_size=%d",
              __FUNCTION__, ret, encodeMs,
              isYuyv ? srcW*srcH*2 : (srcW*srcH*3)/2,
              srcW, srcH, jpegBufSize);
        enc.destroy();
        ion_unmap(ionOutVaddr, jpegBufSize);
        ion_free(ionOutBuf);
        if (ionInVaddr) ion_unmap(ionInVaddr, inBufSize);
        if (ionInBuf >= 0) ion_free(ionInBuf);
        ion_client_destroy(ionClient);
        grmod->unlock(grmod, outBuf);
        return -1;
    }

    int jpegSize = enc.getJpegSize();
    ALOGI("%s: HW-JPEG encoded %d bytes in %.1f ms", __FUNCTION__, jpegSize, encodeMs);

    if (jpegSize <= 0 || jpegSize > blobSize - 1024) {
        // Reserve 1024 bytes for EXIF insertion.
        ALOGE("%s: HW-JPEG invalid size %d (blob=%d)", __FUNCTION__, jpegSize, blobSize);
        enc.destroy();
        ion_unmap(ionOutVaddr, jpegBufSize);
        ion_free(ionOutBuf);
        if (ionInVaddr) ion_unmap(ionInVaddr, inBufSize);
        if (ionInBuf >= 0) ion_free(ionInBuf);
        ion_client_destroy(ionClient);
        grmod->unlock(grmod, outBuf);
        return -1;
    }

    // Copy JPEG from ION buffer to gralloc BLOB.
    // When rotation!=0 we must make room for the EXIF APP1 segment:
    // JPEG structure: [SOI][APP0/JFIF][...][SOS][data][EOI]
    // We insert APP1(Exif) right after SOI:
    // [SOI][APP1(Exif)][APP0(JFIF)][...][SOS][data][EOI]
    // To do this: copy SOI (2 bytes), then EXIF, then the rest from APP0.

    unsigned char *srcJpeg = (unsigned char *)ionOutVaddr;
    unsigned char *dstJpeg = (unsigned char *)blobVaddr;

    if (rotation != 0 && exifOrientation != 1) {
        // Minimal EXIF APP1 segment with only the orientation tag.
        // Structure:
        //   FF E1          APP1 marker
        //   00 1E          Length = 30 bytes (incl. length field, excl. marker)
        //   45 78 69 66 00 00   "Exif\0\0"
        //   49 49 2A 00 08 00 00 00   TIFF header (II, 0x002A, offset 8)
        //   IFD0 (at offset 8):
        //     01 00          Number of entries = 1
        //     12 01          Tag = 0x0112 (Orientation)
        //     03 00          Type = SHORT
        //     01 00 00 00    Count = 1
        //     XX 00 00 00    Value = exifOrientation (padded to 4 bytes)
        //   00 00 00 00      Next IFD offset = 0 (no next IFD)
        static const uint8_t exifTemplate[] = {
            0xFF, 0xE1,             // APP1 marker
            0x00, 0x22,             // Length = 34 (incl. length field, excl. marker)
            0x45, 0x78, 0x69, 0x66, 0x00, 0x00,  // "Exif\0\0"
            0x49, 0x49,             // TIFF byte order: II (little-endian)
            0x2A, 0x00,             // TIFF magic: 0x002A
            0x08, 0x00, 0x00, 0x00, // Offset to IFD0 = 8
            // IFD0:
            0x01, 0x00,             // Number of entries = 1
            0x12, 0x01,             // Tag = 0x0112 (Orientation)
            0x03, 0x00,             // Type = SHORT
            0x01, 0x00, 0x00, 0x00, // Count = 1
            0x00, 0x00, 0x00, 0x00, // Value (will be filled)
            0x00, 0x00, 0x00, 0x00  // Next IFD offset = 0
        };
        int exifLen = sizeof(exifTemplate);

        // Verify SOI marker.
        if (srcJpeg[0] != 0xFF || srcJpeg[1] != 0xD8) {
            ALOGE("%s: invalid JPEG SOI: 0x%02x 0x%02x", __FUNCTION__,
                  srcJpeg[0], srcJpeg[1]);
            enc.destroy();
            ion_unmap(ionOutVaddr, jpegBufSize);
            ion_free(ionOutBuf);
            ion_client_destroy(ionClient);
            grmod->unlock(grmod, outBuf);
            return -1;
        }

        // Copy SOI (2 bytes).
        memcpy(dstJpeg, srcJpeg, 2);
        // Insert EXIF APP1.
        memcpy(dstJpeg + 2, exifTemplate, exifLen);
        // Fill in orientation value (little-endian SHORT at value offset 28 in template).
        // Template layout: [0-1]marker [2-3]len [4-9]Exif [10-11]II [12-13]magic
        //   [14-17]ifd0_off [18-19]num_entries [20-21]tag [22-23]type [24-27]count
        //   [28-31]value [32-35]next_ifd
        dstJpeg[2 + 28] = (uint8_t)exifOrientation;
        dstJpeg[2 + 29] = 0;
        // Copy rest of JPEG (after SOI).
        memcpy(dstJpeg + 2 + exifLen, srcJpeg + 2, jpegSize - 2);
        jpegSize += exifLen;

        ALOGI("%s: inserted EXIF orientation=%d (%d bytes), total JPEG=%d",
              __FUNCTION__, exifOrientation, exifLen, jpegSize);
    } else {
        // No rotation — just copy JPEG as-is.
        memcpy(dstJpeg, srcJpeg, jpegSize);
    }

    // Cleanup.
    enc.destroy();
    ion_unmap(ionOutVaddr, jpegBufSize);
    ion_free(ionOutBuf);
    if (ionInVaddr) ion_unmap(ionInVaddr, inBufSize);
    if (ionInBuf >= 0) ion_free(ionInBuf);
    ion_client_destroy(ionClient);
    grmod->unlock(grmod, outBuf);

    ALOGI("%s: HW-JPEG done: %d bytes (src=%dx%d rot=%d isYuyv=%d %.1f ms)",
          __FUNCTION__, jpegSize, srcW, srcH, rotation, isYuyv, encodeMs);
    return jpegSize;
}

int G800FCamera2Device::flush()
{
    ALOGI("%s: flushing capture queue (previewStarted=%d)", __FUNCTION__, (int)m_previewStarted.load());

    // CRITICAL (sensor protection): turn off flash on flush.
    // The framework calls flush() when switching between preview/video/still,
    // on configuration changes, and on error conditions. If a flash
    // capture is aborted, the flash must turn off immediately, otherwise
    // sensor overheating/damage may occur.
    if (m_pipeEngine && m_pipeEngine->isFlashSequenceActive()) {
        ALOGW("%s: Flash active during flush (state=%s), forcing endFlashSequence()",
              __FUNCTION__,
              G800FPipeEngine::flashSeqStateName(m_pipeEngine->getFlashSeqState()));
        m_pipeEngine->endFlashSequence();  // sets aeflashMode=OFF + releaseFlash()
    }

    // Cancel AF: sets afMode=OFF in all shot_ext buffers.
    // Prevents AF state inconsistencies on rapid mode switches.
    if (m_pipeEngine) {
        m_pipeEngine->cancelAutoFocus();
    }

    // Take every pending request out of the queue under the lock, then
    // complete them with an error status *outside* the lock — the framework
    // may call back into processCaptureRequest() from those callbacks, which
    // would deadlock on m_queueLock.
    std::list<QueuedRequest> aborted;
    {
        Mutex::Autolock ql(m_queueLock);
        m_flushing = true;
        aborted.swap(m_requestQueue);
        m_queueCond.broadcast();
    }
    // Also flush the async capture worker queue
    std::list<QueuedRequest> abortedCaptures;
    {
        Mutex::Autolock cl(m_captureQueueLock);
        abortedCaptures.swap(m_captureQueue);
        m_captureQueueCond.broadcast();
    }
    // And the deferred analysis/callback jobs — a job already being
    // processed by the worker completes normally (returns its buffer
    // a few ms later — legal for an async HAL).
    std::list<AnalysisJob> abortedAnalysis;
    {
        Mutex::Autolock al(m_analysisQueueLock);
        abortedAnalysis.swap(m_analysisQueue);
        m_analysisQueueCond.broadcast();
    }

    size_t abortedCount = aborted.size() + abortedCaptures.size()
                          + abortedAnalysis.size();
    for (std::list<QueuedRequest>::iterator it = aborted.begin();
         it != aborted.end(); ++it) {
        sendErrorResult(it->frame_number, it->output_buffers,
                        it->num_output_buffers);
        free(it->output_buffers);
        if (it->settings) free_camera_metadata(it->settings);
    }
    for (std::list<QueuedRequest>::iterator it = abortedCaptures.begin();
         it != abortedCaptures.end(); ++it) {
        sendErrorResult(it->frame_number, it->output_buffers,
                        it->num_output_buffers);
        free(it->output_buffers);
        if (it->settings) free_camera_metadata(it->settings);
    }
    for (std::list<AnalysisJob>::iterator it = abortedAnalysis.begin();
         it != abortedAnalysis.end(); ++it) {
        errorAnalysisJob(*it);
        if (it->frame && m_pipeEngine) m_pipeEngine->releasePreviewFrame(it->frame);
    }

    // Wait (bounded) for the main dispatch worker to finish the request it is
    // currently in.  The dispatch worker (captureThreadLoop) only handles
    // preview or dispatches to the async capture worker, so this is fast.
    // Spec: flush should return in 100ms, must return in 1000ms.
    int waitedMs = 0;
    {
        Mutex::Autolock ql(m_queueLock);
        while (m_workerBusy && waitedMs < 200) {
            m_queueCond.waitRelative(m_queueLock, ms2ns(20));
            waitedMs += 20;
        }
        if (m_workerBusy)
            ALOGW("%s: dispatch worker still busy after %dms — returning anyway",
                  __FUNCTION__, waitedMs);
    }

    m_flushing = false;
    ALOGI("%s: flush complete (%zu requests aborted, waited %dms)",
          __FUNCTION__, abortedCount, waitedMs);
    return 0;
}

void G800FCamera2Device::dump(int /*fd*/)
{
}

void G800FCamera2Device::copyPreviewToStream(G800FExynosCameraBuffer *camBuf,
                                             buffer_handle_t outBuf,
                                             int width, int height)
{
    if (camBuf == NULL || outBuf == NULL)
        return;

    // SCP produces NV21M (2 planes: Y + interleaved VU).
    // The buffer is passed directly as NV21 to gralloc — no RGBA
    // conversion. We copy NV21M → NV21 (single-plane) directly.
    //
    // The Exynos 3470 gralloc supports NV21 (HAL_PIXEL_FORMAT_YCrCb_420_SP)
    // natively — HWC/Mali can render it directly as a texture.

    const gralloc_module_t *grmod = NULL;
    hw_module_t const *hwmod = NULL;
    if (hw_get_module(GRALLOC_HARDWARE_MODULE_ID, &hwmod) != 0) {
        ALOGE("%s: cannot get gralloc module", __FUNCTION__);
        return;
    }
    grmod = reinterpret_cast<const gralloc_module_t*>(hwmod);

    void *vaddr = NULL;
    int err = grmod->lock(grmod, outBuf,
                          GRALLOC_USAGE_SW_WRITE_OFTEN,
                          0, 0, width, height, &vaddr);
    if (err != 0) {
        ALOGE("%s: gralloc_lock failed: %d", __FUNCTION__, err);
        return;
    }

    private_handle_t *hnd = (private_handle_t *)outBuf;
    int dstFormat = hnd->format;
    int dstStride = hnd->stride;

    // Safety fallback
    if ((dstFormat == 0 || dstStride == 0 || dstStride < width) && m_previewStream != NULL) {
        dstFormat = m_previewStream->format;
        if (dstStride <= 0 || dstStride < width)
            dstStride = width;
    }

    // Derive Y/Cb/Cr plane pointers from the private_handle_t directly.
    // The standard gralloc0 lock() API only returns a single pointer (Y plane
    // base) via *vaddr.  Chroma planes are computed from hnd->base1/base2
    // (multi-plane) or from offsets within hnd->base (single-plane).
    void *dstY  = vaddr;
    void *dstC1 = NULL;
    void *dstC2 = NULL;
    grallocGetPlanePtrs(hnd, dstY, dstC1, dstC2);

    // Source: SCP NV21M output
    //   plane 0: Y (scpStride bytes per row)
    //   plane 1: interleaved VU (scpStride bytes per row)
    // getPreviewWidth() returns the SCP stride (1200), not the
    // app-requested size. This is the crucial fix for distortion.
    uint8_t *srcY  = reinterpret_cast<uint8_t*>(camBuf->planeVaddr(0));
    uint8_t *srcVU = reinterpret_cast<uint8_t*>(camBuf->planeVaddr(1));
    int srcW = m_pipeEngine ? m_pipeEngine->getPreviewWidth() : 1200;
    int srcH = m_pipeEngine ? m_pipeEngine->getPreviewHeight() : 576;
    int srcYStride = srcW;       // SCP Y stride = scpW (1200)
    int srcCStride = srcW;       // SCP chroma stride = scpW (VU interleaved)

    if (dstY == NULL || srcY == NULL) {
        ALOGE("%s: null Y plane (dst=%p src=%p)", __FUNCTION__, dstY, srcY);
        grmod->unlock(grmod, outBuf);
        return;
    }

    // Source dimensions: SCP produces srcW×srcH (e.g. 1920×1080).
    // Destination dimensions: width×height (app-requested size).
    // Aspect ratio adjustment: center-crop the source to the target aspect ratio,
    // so the image is not stretched (e.g. 16:9 → 4:3).
    int chromaRows = height / 2;
    int chromaCols = width / 2;

    int cropLeft = 0, cropTop = 0, cropW = srcW, cropH = srcH;
    if (srcW > 0 && srcH > 0 && width > 0 && height > 0) {
        int srcRatio = srcW * 1000 / srcH;
        int dstRatio = width * 1000 / height;
        if (srcRatio != dstRatio) {
            if (srcRatio > dstRatio) {
                cropW = (int)((long)srcH * width / height);
                cropLeft = (srcW - cropW) / 2;
            } else {
                cropH = (int)((long)srcW * height / width);
                cropTop = (srcH - cropH) / 2;
            }
            //ALOGD("%s: aspect crop %dx%d → %dx%d at (%d,%d) for dst %dx%d",
            //      __FUNCTION__, srcW, srcH, cropW, cropH, cropLeft, cropTop, width, height);
        }
    }

    //ALOGD("%s: src=%dx%d (stride=%d, crop=%dx%d at %d,%d) dst=%dx%d (stride=%d, fmt=0x%x)",
    //      __FUNCTION__, srcW, srcH, srcYStride, cropW, cropH, cropLeft, cropTop,
    //      width, height, dstStride, dstFormat);

    if (dstFormat == HAL_PIXEL_FORMAT_YCRCB_420_SP) {
        // NV21 single-plane — nearest-neighbor scaling (both X and Y).
        // IMPORTANT: horizontal must also be scaled (not memcpy-crop!),
        // otherwise when srcW > width (e.g. 1920→640) only the left side
        // is copied and the image is clipped.
        // Aspect ratio crop: source is first cropped to cropW×cropH,
        // then scaled to width×height.
        uint8_t *dst = (uint8_t*)dstY;
        for (int row = 0; row < height; row++) {
            int srcRow = cropTop + (int)((long)row * cropH / height);
            if (srcRow >= srcH) srcRow = srcH - 1;
            uint8_t *srcRowPtr = srcY + srcRow * srcYStride;
            uint8_t *dstRowPtr = dst + row * dstStride;
            if (cropW == width && cropLeft == 0) {
                memcpy(dstRowPtr, srcRowPtr, width);
            } else {
                for (int col = 0; col < width; col++) {
                    int srcCol = cropLeft + (int)((long)col * cropW / width);
                    if (srcCol >= srcW) srcCol = srcW - 1;
                    dstRowPtr[col] = srcRowPtr[srcCol];
                }
            }
        }
        uint8_t *dstC = dst + dstStride * height;
        for (int row = 0; row < chromaRows; row++) {
            int srcRow = (cropTop >> 1) + (int)((long)row * (cropH >> 1) / chromaRows);
            if (srcRow >= srcH / 2) srcRow = srcH / 2 - 1;
            uint8_t *srcRowPtr = srcVU + srcRow * srcCStride;
            uint8_t *dstRowPtr = dstC + row * dstStride;
            if (cropW == width && cropLeft == 0) {
                memcpy(dstRowPtr, srcRowPtr, width);
            } else {
                for (int col = 0; col < chromaCols; col++) {
                    int srcCol = (cropLeft >> 1) + (int)((long)col * (cropW >> 1) / chromaCols);
                    if (srcCol >= srcW / 2) srcCol = srcW / 2 - 1;
                    dstRowPtr[2 * col]     = srcRowPtr[2 * srcCol];     // V
                    dstRowPtr[2 * col + 1] = srcRowPtr[2 * srcCol + 1]; // U
                }
            }
        }
        //ALOGD("%s: NV21M -> NV21 (single-plane, scaled %dx%d -> %dx%d)",
        //      __FUNCTION__, cropW, cropH, width, height);
    } else if (dstC2 != NULL) {
        // 3-plane format (YV12 / YCbCr_420_888) — nearest-neighbor scaling
        uint8_t *dstYp = (uint8_t*)dstY;
        for (int row = 0; row < height; row++) {
            int srcRow = cropTop + (int)((long)row * cropH / height);
            if (srcRow >= srcH) srcRow = srcH - 1;
            uint8_t *srcRowPtr = srcY + srcRow * srcYStride;
            uint8_t *dstRowPtr = dstYp + row * dstStride;
            if (cropW == width && cropLeft == 0) {
                memcpy(dstRowPtr, srcRowPtr, width);
            } else {
                for (int col = 0; col < width; col++) {
                    int srcCol = cropLeft + (int)((long)col * cropW / width);
                    if (srcCol >= srcW) srcCol = srcW - 1;
                    dstRowPtr[col] = srcRowPtr[srcCol];
                }
            }
        }
        int dstCStride = ALIGN(dstStride / 2, 16);
        uint8_t *dstCb = (uint8_t*)dstC1;
        uint8_t *dstCr = (uint8_t*)dstC2;
        for (int row = 0; row < chromaRows; row++) {
            int srcRow = (cropTop >> 1) + (int)((long)row * (cropH >> 1) / chromaRows);
            if (srcRow >= srcH / 2) srcRow = srcH / 2 - 1;
            uint8_t *srcRowPtr = srcVU + srcRow * srcCStride;
            for (int col = 0; col < chromaCols; col++) {
                int srcCol = (cropLeft >> 1) + (int)((long)col * (cropW >> 1) / chromaCols);
                if (srcCol >= srcW / 2) srcCol = srcW / 2 - 1;
                dstCr[row * dstCStride + col] = srcRowPtr[2 * srcCol];     // V
                dstCb[row * dstCStride + col] = srcRowPtr[2 * srcCol + 1]; // U
            }
        }
        //ALOGD("%s: NV21M -> YV12 (3-plane, scaled %dx%d -> %dx%d)",
        //      __FUNCTION__, cropW, cropH, width, height);
    } else if (dstC1 != NULL) {
        // 2-plane format (NV12M or NV21M) — nearest-neighbor scaling
        uint8_t *dstYp = (uint8_t*)dstY;
        for (int row = 0; row < height; row++) {
            int srcRow = cropTop + (int)((long)row * cropH / height);
            if (srcRow >= srcH) srcRow = srcH - 1;
            uint8_t *srcRowPtr = srcY + srcRow * srcYStride;
            uint8_t *dstRowPtr = dstYp + row * dstStride;
            if (cropW == width && cropLeft == 0) {
                memcpy(dstRowPtr, srcRowPtr, width);
            } else {
                for (int col = 0; col < width; col++) {
                    int srcCol = cropLeft + (int)((long)col * cropW / width);
                    if (srcCol >= srcW) srcCol = srcW - 1;
                    dstRowPtr[col] = srcRowPtr[srcCol];
                }
            }
        }
        if (dstFormat == HAL_PIXEL_FORMAT_EXYNOS_YCrCb_420_SP_M ||
            dstFormat == HAL_PIXEL_FORMAT_EXYNOS_YCrCb_420_SP_M_FULL) {
            uint8_t *dstVU = (uint8_t*)dstC1;
            for (int row = 0; row < chromaRows; row++) {
                int srcRow = (cropTop >> 1) + (int)((long)row * (cropH >> 1) / chromaRows);
                if (srcRow >= srcH / 2) srcRow = srcH / 2 - 1;
                uint8_t *srcRowPtr = srcVU + srcRow * srcCStride;
                uint8_t *dstRowPtr = dstVU + row * dstStride;
                if (cropW == width && cropLeft == 0) {
                    memcpy(dstRowPtr, srcRowPtr, width);
                } else {
                    for (int col = 0; col < chromaCols; col++) {
                        int srcCol = (cropLeft >> 1) + (int)((long)col * (cropW >> 1) / chromaCols);
                        if (srcCol >= srcW / 2) srcCol = srcW / 2 - 1;
                        dstRowPtr[2 * col]     = srcRowPtr[2 * srcCol];     // V
                        dstRowPtr[2 * col + 1] = srcRowPtr[2 * srcCol + 1]; // U
                    }
                }
            }
            //ALOGD("%s: NV21M -> NV21M (direct, scaled %dx%d -> %dx%d)",
            //      __FUNCTION__, cropW, cropH, width, height);
        } else {
            uint8_t *dstUV = (uint8_t*)dstC1;
            for (int row = 0; row < chromaRows; row++) {
                int srcRow = (cropTop >> 1) + (int)((long)row * (cropH >> 1) / chromaRows);
                if (srcRow >= srcH / 2) srcRow = srcH / 2 - 1;
                uint8_t *srcRowPtr = srcVU + srcRow * srcCStride;
                uint8_t *dstRowPtr = dstUV + row * dstStride;
                for (int col = 0; col < chromaCols; col++) {
                    int srcCol = (cropLeft >> 1) + (int)((long)col * (cropW >> 1) / chromaCols);
                    if (srcCol >= srcW / 2) srcCol = srcW / 2 - 1;
                    dstRowPtr[2 * col]     = srcRowPtr[2 * srcCol + 1]; // U
                    dstRowPtr[2 * col + 1] = srcRowPtr[2 * srcCol];     // V
                }
            }
            //ALOGD("%s: NV21M -> NV12M (swapped, scaled %dx%d -> %dx%d)",
            //      __FUNCTION__, cropW, cropH, width, height);
        }
    } else if (dstFormat == HAL_PIXEL_FORMAT_RGBA_8888 ||
               dstFormat == HAL_PIXEL_FORMAT_RGBX_8888) {
        // RGBA fallback — only when the framework forces RGBA.
        uint8_t *dst = reinterpret_cast<uint8_t*>(dstY);
        int dstRowBytes = dstStride * 4;
        for (int row = 0; row < height; row++) {
            int srcRow = cropTop + (int)((long)row * cropH / height);
            if (srcRow >= srcH) srcRow = srcH - 1;
            for (int col = 0; col < width; col++) {
                int srcCol = cropLeft + (int)((long)col * cropW / width);
                if (srcCol >= srcW) srcCol = srcW - 1;
                int y = srcY[srcRow * srcYStride + srcCol];
                int uvIdx = (srcRow >> 1) * srcCStride + (srcCol & ~1);
                int v = srcVU[uvIdx] - 128;
                int u = srcVU[uvIdx + 1] - 128;
                int r = y + ((180 * v + 64) >> 7);
                int g = y - ((44 * u + 91 * v + 64) >> 7);
                int b = y + ((227 * u + 64) >> 7);
                uint8_t *px = dst + row * dstRowBytes + col * 4;
                px[0] = (uint8_t)(r < 0 ? 0 : (r > 255 ? 255 : r));
                px[1] = (uint8_t)(g < 0 ? 0 : (g > 255 ? 255 : g));
                px[2] = (uint8_t)(b < 0 ? 0 : (b > 255 ? 255 : b));
                px[3] = 0xff;
            }
        }
        //ALOGD("%s: NV21M -> RGBA (scalar fallback, scaled %dx%d -> %dx%d)",
        //      __FUNCTION__, cropW, cropH, width, height);
    } else {
        ALOGW("%s: unsupported destination format 0x%x", __FUNCTION__, dstFormat);
    }

    grmod->unlock(grmod, outBuf);
}

void G800FCamera2Device::copyPreviewToStreamFromData(void* srcYp, void* srcVUp,
                                                      int srcW, int srcH,
                                                      buffer_handle_t outBuf,
                                                      int width, int height)
{
    // Same logic as copyPreviewToStream, but the source is a raw
    // data copy (previewData from G800FFrame) instead of a V4L2 buffer.
    // Used when the SCP buffer was immediately requeued (conformant).
    if (srcYp == NULL || srcVUp == NULL || outBuf == NULL)
        return;

    const gralloc_module_t *grmod = NULL;
    hw_module_t const *hwmod = NULL;
    if (hw_get_module(GRALLOC_HARDWARE_MODULE_ID, &hwmod) != 0) {
        ALOGE("%s: cannot get gralloc module", __FUNCTION__);
        return;
    }
    grmod = reinterpret_cast<const gralloc_module_t*>(hwmod);

    void *vaddr = NULL;
    int err = grmod->lock(grmod, outBuf,
                          GRALLOC_USAGE_SW_WRITE_OFTEN,
                          0, 0, width, height, &vaddr);
    if (err != 0) {
        ALOGE("%s: gralloc_lock failed: %d", __FUNCTION__, err);
        return;
    }

    private_handle_t *hnd = (private_handle_t *)outBuf;
    int dstFormat = hnd->format;
    int dstStride = hnd->stride;
    // DIAG: Log all dimensions and strides to diagnose diagonal stripe corruption.
    //ALOGI("%s: srcW=%d srcH=%d streamW=%d streamH=%d hnd{fmt=0x%x stride=%d w=%d h=%d}",
    //      __FUNCTION__, srcW, srcH, width, height, dstFormat, dstStride,
    //      hnd->width, hnd->height);
    if ((dstFormat == 0 || dstStride == 0 || dstStride < width) && m_previewStream != NULL) {
        dstFormat = m_previewStream->format;
        if (dstStride <= 0 || dstStride < width)
            dstStride = width;
        //ALOGI("%s: dstStride overridden to %d (hnd->stride=%d < width=%d)",
        //      __FUNCTION__, dstStride, hnd->stride, width);
    }

    // Derive Y/Cb/Cr plane pointers from the private_handle_t directly.
    // See grallocGetPlanePtrs() for why we don't use vaddr[1]/vaddr[2].
    void *dstY  = vaddr;
    void *dstC1 = NULL;
    void *dstC2 = NULL;
    grallocGetPlanePtrs(hnd, dstY, dstC1, dstC2);

    uint8_t *srcY  = reinterpret_cast<uint8_t*>(srcYp);
    uint8_t *srcVU = reinterpret_cast<uint8_t*>(srcVUp);
    int srcYStride = srcW;
    int srcCStride = srcW;

    if (dstY == NULL || srcY == NULL) {
        ALOGE("%s: null Y plane (dst=%p src=%p)", __FUNCTION__, dstY, srcY);
        grmod->unlock(grmod, outBuf);
        return;
    }

    // SCP firmware outputs 1200x576, app may request 960x720.
    // Use nearest-neighbor vertical scaling: for each destination row,
    // compute the corresponding source row. Crop columns if srcW > width.
    int chromaRows = height / 2;
    int chromaCols = width / 2;

    if (dstFormat == HAL_PIXEL_FORMAT_RGBA_8888 ||
        dstFormat == HAL_PIXEL_FORMAT_RGBX_8888) {
        uint8_t *dst = reinterpret_cast<uint8_t*>(dstY);
        int dstRowBytes = dstStride * 4;
        // DIAG: Log conversion parameters
        //ALOGI("%s: RGBA conv srcYStride=%d srcCStride=%d dstStride=%d dstRowBytes=%d"
        //      " w=%d h=%d chromaRows=%d",
        //      __FUNCTION__, srcYStride, srcCStride, dstStride, dstRowBytes,
        //      width, height, chromaRows);

#ifdef __ARM_NEON__
        // NEON NV21→RGBA conversion.
        //
        // BUGFIX (diagonal stripes): the old version loaded 8 VU pairs (16 bytes
        // = 16 columns) with vld2_u8, but only processed 8 Y pixels (col+=8).
        // Each VU pair covers 2 horizontal pixels, but the code applied
        // each V/U to only 1 pixel → chroma shift → diagonal stripes.
        //
        // Fix: process 16 Y pixels per iteration (col+=16) with 8 VU
        // pairs. vzip_u8 duplicates each V/U for 2 consecutive pixels.
        // 960/16 = 60 → width is divisible by 16.
        //
        // SCP firmware outputs 1200x576, app may request 960x720.
        // Nearest-neighbor vertical scaling: srcRow = row * srcH / height.
        // Columns are cropped (width <= srcW).
        for (int row = 0; row < height; row += 2) {
            int srcRow0 = (int)((long)row * srcH / height);
            int srcRow1 = (int)((long)(row + 1) * srcH / height);
            if (srcRow0 >= srcH) srcRow0 = srcH - 1;
            if (srcRow1 >= srcH) srcRow1 = srcH - 1;
            int srcChromaRow = srcRow0 >> 1;
            for (int col = 0; col < width; col += 16) {
                const uint8_t *yRow0 = srcY + srcRow0 * srcYStride + col;
                const uint8_t *yRow1 = srcY + srcRow1 * srcYStride + col;
                const uint8_t *vuRow = srcVU + srcChromaRow * srcCStride + col;
                uint8_t *dRow0 = dst + row * dstRowBytes + col * 4;
                uint8_t *dRow1 = dst + (row + 1) * dstRowBytes + col * 4;

                // Load 8 VU pairs (16 bytes = 16 pixel columns)
                uint8x8x2_t vu = vld2_u8(vuRow);
                // Kernel SCP DMA uses DMA_OUTPUT_ORDER_CbCr = UV.
                // val[0] = Cb (U), val[1] = Cr (V)
                uint8x8_t uVec = vu.val[0];  // U for cols 0,2,4,6,8,10,12,14
                uint8x8_t vVec = vu.val[1];  // V for cols 0,2,4,6,8,10,12,14

                // Duplicate each V/U for 2 consecutive pixels
                // vzip_u8(uVec, uVec) → [U0,U0, U2,U2, U4,U4, U6,U6] | [U8,U8, U10,U10, U12,U12, U14,U14]
                uint8x8x2_t uDup = vzip_u8(uVec, uVec);
                uint8x8x2_t vDup = vzip_u8(vVec, vVec);

                // Two halves: 8 pixels each
                for (int half = 0; half < 2; half++) {
                    uint8x8_t uH = uDup.val[half];
                    uint8x8_t vH = vDup.val[half];

                    int16x8_t vS = vreinterpretq_s16_u16(vmovl_u8(vH));
                    int16x8_t uS = vreinterpretq_s16_u16(vmovl_u8(uH));
                    vS = vsubq_s16(vS, vdupq_n_s16(128));
                    uS = vsubq_s16(uS, vdupq_n_s16(128));

                    int16x8_t rChroma = vmulq_n_s16(vS, 180);
                    int16x8_t gChromaV = vmulq_n_s16(vS, 91);
                    int16x8_t gChromaU = vmulq_n_s16(uS, 44);
                    int16x8_t bChroma = vmulq_n_s16(uS, 227);

                    for (int rIdx = 0; rIdx < 2; rIdx++) {
                        const uint8_t *yPtr = (rIdx == 0) ? yRow0 : yRow1;
                        uint8_t *dPtr = (rIdx == 0) ? dRow0 : dRow1;
                        // For half=1: Y pixels 8-15, dest 8 pixels further
                        if (half == 1) {
                            yPtr += 8;
                            dPtr += 32;  // 8 pixels * 4 bytes
                        }

                        uint8x8_t yVec = vld1_u8(yPtr);
                        int16x8_t yS = vreinterpretq_s16_u16(vmovl_u8(yVec));

                        int16x8_t rS = vaddq_s16(yS,
                                                  vshrq_n_s16(vaddq_s16(rChroma,
                                                                        vdupq_n_s16(64)), 7));
                        int16x8_t gS = vsubq_s16(yS,
                                                  vshrq_n_s16(vaddq_s16(vaddq_s16(gChromaU,
                                                                                  gChromaV),
                                                                        vdupq_n_s16(64)), 7));
                        int16x8_t bS = vaddq_s16(yS,
                                                  vshrq_n_s16(vaddq_s16(bChroma,
                                                                        vdupq_n_s16(64)), 7));

                        uint8x8_t rU = vqmovun_s16(rS);
                        uint8x8_t gU = vqmovun_s16(gS);
                        uint8x8_t bU = vqmovun_s16(bS);

                        uint8x8x4_t rgba;
                        rgba.val[0] = rU;  // R
                        rgba.val[1] = gU;  // G
                        rgba.val[2] = bU;  // B
                        rgba.val[3] = vdup_n_u8(255);  // A
                        vst4_u8(dPtr, rgba);
                    }
                }
            }
        }
#else
        // Scalar fallback — processes 2x2 pixel blocks correctly.
        // BUGFIX: the old version wrote only 1 pixel per iteration instead of 2.
        // Nearest-neighbor vertical scaling as in the NEON path.
        for (int row = 0; row < height; row += 2) {
            int srcRow0 = (int)((long)row * srcH / height);
            int srcRow1 = (int)((long)(row + 1) * srcH / height);
            if (srcRow0 >= srcH) srcRow0 = srcH - 1;
            if (srcRow1 >= srcH) srcRow1 = srcH - 1;
            int srcChromaRow = srcRow0 >> 1;
            for (int col = 0; col < width; col += 2) {
                int yIdx = srcRow0 * srcYStride + col;
                int yIdx2 = srcRow1 * srcYStride + col;
                int vuIdx = srcChromaRow * srcCStride + col;
                // Kernel SCP DMA: CbCr = UV order
                int u = srcVU[vuIdx] - 128;      // Cb = U
                int v = srcVU[vuIdx + 1] - 128;  // Cr = V
                int y0 = srcY[yIdx];
                int y1 = srcY[yIdx + 1];              // col+1, same row
                int y2 = srcY[yIdx2];                 // col, srcRow1
                int y3 = srcY[yIdx2 + 1];             // col+1, srcRow1
                int r, g, b;
                uint8_t *dRow0 = (uint8_t*)dst + row * dstRowBytes + col * 4;
                uint8_t *dRow1 = (uint8_t*)dst + (row + 1) * dstRowBytes + col * 4;
                // 2x2 block: all 4 pixels share the same (U,V)
                r = y0 + (180 * v + 64) >> 7; g = y0 - (44 * u + 91 * v + 64) >> 7; b = y0 + (227 * u + 64) >> 7;
                dRow0[0] = (uint8_t)clamp(r); dRow0[1] = (uint8_t)clamp(g); dRow0[2] = (uint8_t)clamp(b); dRow0[3] = 255;
                r = y1 + (180 * v + 64) >> 7; g = y1 - (44 * u + 91 * v + 64) >> 7; b = y1 + (227 * u + 64) >> 7;
                dRow0[4] = (uint8_t)clamp(r); dRow0[5] = (uint8_t)clamp(g); dRow0[6] = (uint8_t)clamp(b); dRow0[7] = 255;
                r = y2 + (180 * v + 64) >> 7; g = y2 - (44 * u + 91 * v + 64) >> 7; b = y2 + (227 * u + 64) >> 7;
                dRow1[0] = (uint8_t)clamp(r); dRow1[1] = (uint8_t)clamp(g); dRow1[2] = (uint8_t)clamp(b); dRow1[3] = 255;
                r = y3 + (180 * v + 64) >> 7; g = y3 - (44 * u + 91 * v + 64) >> 7; b = y3 + (227 * u + 64) >> 7;
                dRow1[4] = (uint8_t)clamp(r); dRow1[5] = (uint8_t)clamp(g); dRow1[6] = (uint8_t)clamp(b); dRow1[7] = 255;
            }
        }
#endif
    } else if (dstFormat == HAL_PIXEL_FORMAT_YCRCB_420_SP ||
               dstFormat == 0x13 /* HAL_PIXEL_FORMAT_YCbCr_420_SP = NV12 */) {
        // NV21 single-plane — nearest-neighbor vertical scaling
        uint8_t *dst = (uint8_t*)dstY;
        for (int row = 0; row < height; row++) {
            int srcRow = (int)((long)row * srcH / height);
            if (srcRow >= srcH) srcRow = srcH - 1;
            memcpy(dst + row * dstStride, srcY + srcRow * srcYStride, width);
        }
        uint8_t *dstC = dst + dstStride * height;
        for (int row = 0; row < chromaRows; row++) {
            int srcRow = (int)((long)row * (srcH / 2) / chromaRows);
            if (srcRow >= srcH / 2) srcRow = srcH / 2 - 1;
            memcpy(dstC + row * dstStride, srcVU + srcRow * srcCStride, width);
        }
    } else if (dstC1 != NULL) {
        // Multi-plane YUV (NV21M / YV12) — nearest-neighbor vertical scaling
        uint8_t *dstYp = (uint8_t*)dstY;
        for (int row = 0; row < height; row++) {
            int srcRow = (int)((long)row * srcH / height);
            if (srcRow >= srcH) srcRow = srcH - 1;
            memcpy(dstYp + row * dstStride, srcY + srcRow * srcYStride, width);
        }
        if (dstC2 != NULL) {
            // YV12: separate Cb, Cr planes
            uint8_t *dstCb = (uint8_t*)dstC1;
            uint8_t *dstCr = (uint8_t*)dstC2;
            for (int row = 0; row < chromaRows; row++) {
                int srcRow = (int)((long)row * (srcH / 2) / chromaRows);
                if (srcRow >= srcH / 2) srcRow = srcH / 2 - 1;
                for (int col = 0; col < chromaCols; col++) {
                    dstCb[row * (dstStride / 2) + col] = srcVU[srcRow * srcCStride + col * 2 + 1];
                    dstCr[row * (dstStride / 2) + col] = srcVU[srcRow * srcCStride + col * 2];
                }
            }
        } else {
            // NV21M: interleaved VU
            uint8_t *dstVU = (uint8_t*)dstC1;
            for (int row = 0; row < chromaRows; row++) {
                int srcRow = (int)((long)row * (srcH / 2) / chromaRows);
                if (srcRow >= srcH / 2) srcRow = srcH / 2 - 1;
                memcpy(dstVU + row * dstStride, srcVU + srcRow * srcCStride, width);
            }
        }
    } else {
        ALOGW("%s: single-plane gralloc buffer, format=0x%x", __FUNCTION__, dstFormat);
    }

    grmod->unlock(grmod, outBuf);
}

// Static C ops wrappers.
#define DEV(d) (reinterpret_cast<G800FCamera2Device*>((d)->priv))

int G800FCamera2Device::s_initialize(const camera3_device_t *d,
                                     const camera3_callback_ops_t *callback_ops)
{
    if (d == NULL) return -EINVAL;
    return DEV(d)->initialize(callback_ops);
}

int G800FCamera2Device::s_configure_streams(const camera3_device_t *d,
                                            camera3_stream_configuration_t *stream_list)
{
    if (d == NULL) return -EINVAL;
    return DEV(d)->configureStreams(stream_list);
}

const camera_metadata_t* G800FCamera2Device::s_construct_default_request_settings(
        const camera3_device_t *d, int type)
{
    if (d == NULL) return NULL;
    return DEV(d)->constructDefaultRequestSettings(type);
}

int G800FCamera2Device::s_process_capture_request(const camera3_device_t *d,
                                                  camera3_capture_request_t *request)
{
    if (d == NULL) return -EINVAL;
    return DEV(d)->processCaptureRequest(request);
}

int G800FCamera2Device::s_flush(const camera3_device_t *d)
{
    if (d == NULL) return -EINVAL;
    return DEV(d)->flush();
}

void G800FCamera2Device::s_dump(const camera3_device_t *d, int fd)
{
    if (d == NULL) return;
    DEV(d)->dump(fd);
}

int G800FCamera2Device::s_close(hw_device_t *device)
{
    if (device == NULL)
        return -EINVAL;
    camera3_device_t *d = reinterpret_cast<camera3_device_t*>(device);
    G800FCamera2Device *dev = reinterpret_cast<G800FCamera2Device*>(d->priv);
    delete dev;
    return 0;
}

} // namespace android
