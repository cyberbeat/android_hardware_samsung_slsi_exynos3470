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

#define LOG_TAG "G800FPipeEngine"
#include <log/log.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "G800FPipeEngine.h"
#include "G800FFrameSelector.h"
#include <system/camera_metadata.h>
#include <videodev2_exynos_camera.h>
#include <videodev2_exynos_media.h>
#include <hardware/camera3.h>

// V4L2_CID_IS_SET_SETFILE is missing from the in-tree BSP headers.
// It must be sent in ISP::setupPipe() after s_input.
// Without it, the FIMC-IS firmware has no tuning parameters and cannot
// control sensor exposure → black frames → distorted preview.
#ifndef V4L2_CID_IS_SET_SETFILE
#define V4L2_CID_IS_SET_SETFILE (V4L2_CID_FIMC_IS_BASE + 51)
#endif

namespace android {

static uint32_t sampledByteMean(const void* data, size_t size, size_t byteStride)
{
    if (!data || size == 0 || byteStride == 0) return 0;
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    size_t step = ((size / byteStride) / 4096) * byteStride;
    if (step < byteStride) step = byteStride;
    uint64_t sum = 0;
    uint32_t count = 0;
    for (size_t i = 0; i < size; i += step) {
        sum += bytes[i];
        count++;
    }
    return count ? static_cast<uint32_t>(sum / count) : 0;
}

// Chroma means of a packed YUYV buffer (byte pattern Y0 Cb Y1 Cr).
// Neutral ≈128; green tint shows as Cb<128 && Cr<128.
static void sampledYuyvChromaMean(const void* data, size_t size,
                                  uint32_t* cbMean, uint32_t* crMean)
{
    *cbMean = *crMean = 0;
    if (!data || size < 4) return;
    const uint8_t* b = static_cast<const uint8_t*>(data);
    uint64_t cbSum = 0, crSum = 0;
    uint32_t cbCnt = 0, crCnt = 0;
    size_t step = ((size / 4) / 4096) * 4;   // ~4k samples, 4-aligned
    if (step < 4) step = 4;
    for (size_t i = 0; i + 3 < size; i += step) {
        cbSum += b[i + 1]; cbCnt++;
        crSum += b[i + 3]; crCnt++;
    }
    if (cbCnt) *cbMean = (uint32_t)(cbSum / cbCnt);
    if (crCnt) *crMean = (uint32_t)(crSum / crCnt);
}

// Bayer PACK12 channel sampling was removed: the Bayer-packing decode was
// abandoned, and reading the FLITE pixel plane's CPU vaddr in the reprocess
// path proved unsafe (DMABUF mapping not guaranteed readable → SIGSEGV).
// The reproc path only needs dmaBufFd (kernel-side DMA).

// ============================================================================
// Static members
// ============================================================================
Mutex G800FPipeEngine::s_instanceLock;
G800FPipeEngine* G800FPipeEngine::s_activeInstance = NULL;

// BDS table — S5K4H5 rear, 7 entries; here reduced to aspect-ratio + BDS size.
// getPreviewBdsSize() reads entry[7] and entry[8].
// Aspect-ratios from g_previewResolutionsRear (third column = BDS index):
//   1280x720 → 0 (16:9), 960x720 → 1 (4:3), 736x736 → 2 (1:1), ...
//
// HAL3 optimization: always use 16:9 BDS (index 0) for the rear camera.
// BDS selection by preview surface aspect ratio
// which causes a full pipeline restart (deinit+init, ~3s freeze) when
// the app switches between 4:3 and 16:9 surfaces (e.g. Open Camera's
// 320x240 thumbnail surface → 1280x720 video). With a fixed 16:9 BDS,
// reconfigureScp() is always a no-op (same BDS size), and the FIMC
// hardware scaler (G800FFimcScaler) handles aspect-crop + downscale
// to the app-requested surface size (e.g. 1920x1080 → 320x240).
// The BDS table is retained for reference and potential future use.
const G800FPipeEngine::BdsEntry G800FPipeEngine::BDS_TABLE[] = {
    { 16,  9, 1920, 1080 },  // Index 0: 16:9  (always used for rear camera)
    {  4,  3, 1440, 1080 },  // Index 1: 4:3   (reference only)
    {  1,  1, 1080, 1080 },  // Index 2: 1:1   (reference only)
    {  3,  2, 1616, 1080 },  // Index 3: 3:2   (reference only)
    {  5,  3, 1352, 1080 },  // Index 4: 5:3   (reference only)
    { 16, 10, 1800, 1080 },  // Index 5: 16:10 (reference only)
    { 11,  8, 1320, 1080 },  // Index 6: 11:8  (reference only)
};

// ctl defaults written into every frame's shot metadata.  Leaving them 0
// means *undefined* enum values (0 is not a valid value for these 1-based
// enums) and an all-zero color transform — the ISP firmware may then skip
// color correction entirely on reprocessed frames:
//   ctl.noise.mode=FAST(1) strength=5
//   ctl.color.mode=FAST(1)
//   ctl.color.transform=identity 3x3
//   ctl.tonemap.mode=FAST(1)          (the curve field self-overlaps to ~0,
//                                      FAST ignores it)
//   ctl.edge.mode=FAST(1) strength=5
//   ctl.aa.mode=AUTO(2)
//   ctl.aa.aeRegions={0,0,0,0,1000}
//   ctl.aa.aeAntibandingMode=AUTO(4)
static void applyDefaultCtlValues(struct camera2_ctl *ctl)
{
    ctl->noise.mode     = PROCESSING_MODE_FAST;
    ctl->noise.strength = 5;
    ctl->color.mode     = COLORCORRECTION_MODE_FAST;
    ctl->color.transform[0] = 1.0f;
    ctl->color.transform[4] = 1.0f;
    ctl->color.transform[8] = 1.0f;
    ctl->tonemap.mode   = TONEMAP_MODE_FAST;
    ctl->edge.mode      = PROCESSING_MODE_FAST;
    ctl->edge.strength  = 5;
    ctl->aa.mode        = AA_CONTROL_AUTO;
    ctl->aa.aeRegions[0] = 0;
    ctl->aa.aeRegions[1] = 0;
    ctl->aa.aeRegions[2] = 0;
    ctl->aa.aeRegions[3] = 0;
    ctl->aa.aeRegions[4] = 1000;
    ctl->aa.aeAntibandingMode = AA_AE_ANTIBANDING_AUTO;
}

int G800FPipeEngine::getPreviewBdsSize(int previewW, int previewH,
                                        int *bdsW, int *bdsH)
{
    // Always return 16:9 BDS (index 0). This prevents pipeline restarts
    // when the app switches between 4:3 and 16:9 preview surfaces.
    // The FIMC hardware scaler handles aspect-crop + downscale to the
    // app-requested surface size.
    *bdsW = BDS_TABLE[0].bdsW;
    *bdsH = BDS_TABLE[0].bdsH;
    ALOGI("%s: preview %dx%d → BDS %dx%d (fixed 16:9, FIMC handles aspect-crop)",
          __FUNCTION__, previewW, previewH, *bdsW, *bdsH);
    return 0;
}

// ============================================================================
// Sensor ID helpers
// ============================================================================
int G800FPipeEngine::packSensorIdFLITE(int cameraId) {
    int moduleId = (cameraId == 0) ? SENSOR_ID_S5K4H5 : SENSOR_ID_S5K6B2;
    return (0 << 28) | moduleId;
}
int G800FPipeEngine::packSensorIdISP(int cameraId) {
    int moduleId = (cameraId == 0) ? SENSOR_ID_S5K4H5 : SENSOR_ID_S5K6B2;
    int ssxVindex = (cameraId == 0) ? 0 : 1; // SS0/SS1
    return (0 << 28) | (ssxVindex << 16) | (12 << 8) | moduleId; // 3A0P=12
}
int G800FPipeEngine::packSensorIdSCC(int cameraId) {
    int moduleId = (cameraId == 0) ? SENSOR_ID_S5K4H5 : SENSOR_ID_S5K6B2;
    int ssxVindex = (cameraId == 0) ? 0 : 1;
    return (0 << 28) | (ssxVindex << 16) | (34 << 8) | moduleId; // SCC=34
}
int G800FPipeEngine::packSensorIdSCP(int cameraId) {
    int moduleId = (cameraId == 0) ? SENSOR_ID_S5K4H5 : SENSOR_ID_S5K6B2;
    int ssxVindex = (cameraId == 0) ? 0 : 1;
    return (0 << 28) | (ssxVindex << 16) | (37 << 8) | moduleId; // SCP=37
}
int G800FPipeEngine::packSensorIdISPReproc(int cameraId) {
    int moduleId = (cameraId == 0) ? SENSOR_ID_S5K4H5 : SENSOR_ID_S5K6B2;
    int ssxVindex = (cameraId == 0) ? 0 : 1;
    return (1 << 28) | (ssxVindex << 16) | (12 << 8) | moduleId;
}
int G800FPipeEngine::packSensorIdSCCReproc(int cameraId) {
    int moduleId = (cameraId == 0) ? SENSOR_ID_S5K4H5 : SENSOR_ID_S5K6B2;
    int ssxVindex = (cameraId == 0) ? 0 : 1;
    return (1 << 28) | (ssxVindex << 16) | (34 << 8) | moduleId;
}

// ============================================================================
// Node group helpers
// ============================================================================
void G800FPipeEngine::setNodeGroup(camera2_shot_ext* shot, int index, int vid,
                                   int request, int x, int y, int w, int h) {
    // Simple version: input == output crop.
    setNodeGroup(shot, index, vid, request, x, y, w, h, x, y, w, h);
}

void G800FPipeEngine::setNodeGroup(camera2_shot_ext* shot, int index, int vid,
                                   int request,
                                   int ix, int iy, int iw, int ih,
                                   int ox, int oy, int ow, int oh) {
    // Per kernel isp_tag/scc_tag/scp_tag:
    //
    //   For the leader (index == -1), vid, request, input.cropRegion
    //   and output.cropRegion MUST be set.  The kernel isp_tag() reads
    //   node->output.cropRegion and only falls back to isp_param->otf_output
    //   when IS_NULL_COORD (all 0) is set.  Without leader crops,
    //   the FIMC-IS firmware produces SCC/SCP at only ~1 fps.
    //
    //   leader.input.crop  = bayerCrop  (sensor/FLITE size)
    //   leader.output.crop = bds        (ISP OTF output size)
    //
    //   For capture nodes (index >= 0):
    //   capture[N].input.crop  = ISP output size (from leader.output)
    //   capture[N].output.crop = respective node size (SCP=preview, SCC=picture)
    if (!shot) return;
    camera2_node_group& ng = shot->node_group;
    if (index == -1) {
        ng.leader.vid     = vid;
        ng.leader.request = request;
        ng.leader.input.cropRegion[0]  = ix;
        ng.leader.input.cropRegion[1]  = iy;
        ng.leader.input.cropRegion[2]  = iw;
        ng.leader.input.cropRegion[3]  = ih;
        ng.leader.output.cropRegion[0] = ox;
        ng.leader.output.cropRegion[1] = oy;
        ng.leader.output.cropRegion[2] = ow;
        ng.leader.output.cropRegion[3] = oh;
    } else if (index >= 0 && index < (int)(sizeof(ng.capture) / sizeof(ng.capture[0]))) {
        ng.capture[index].vid     = vid;
        ng.capture[index].request = request;
        ng.capture[index].input.cropRegion[0]  = ix;
        ng.capture[index].input.cropRegion[1]  = iy;
        ng.capture[index].input.cropRegion[2]  = iw;
        ng.capture[index].input.cropRegion[3]  = ih;
        ng.capture[index].output.cropRegion[0] = ox;
        ng.capture[index].output.cropRegion[1] = oy;
        ng.capture[index].output.cropRegion[2] = ow;
        ng.capture[index].output.cropRegion[3] = oh;
    }
}

// ============================================================================
// G800FFlitePipe
// ============================================================================
G800FFlitePipe::G800FFlitePipe()
    : G800FPipe(PIPE_FLITE, "G800FFlitePipe"),
      m_shotExt(NULL), m_shotExtCount(0), m_aeflashMode(0),
      m_reprocRedirectQ(NULL), m_frameSelector(NULL)
{
    memset(m_fliteInUse, 0, sizeof(m_fliteInUse));
}

G800FFlitePipe::~G800FFlitePipe()
{
}

status_t G800FFlitePipe::create(int videoNodeNum, int sensorId)
{
    m_videoNodeNum = videoNodeNum;
    m_sensorId = sensorId;
    status_t err = m_node.create(videoNodeNum);
    if (err != NO_ERROR) return err;
    err = m_node.open();
    if (err != NO_ERROR) return err;
    err = m_node.setInput(sensorId);
    ALOGI("%s: FLITE node %d opened, sensorId=0x%x", __FUNCTION__, videoNodeNum, sensorId);
    return err;
}

status_t G800FFlitePipe::setupPipe(int w, int h, int pixFmt,
                                   int numPlanes, int numBuffers,
                                   v4l2_buf_type bufType,
                                   v4l2_memory memory)
{
    status_t err = m_node.setSize(w, h);
    if (err != NO_ERROR) return err;
    err = m_node.setColorFormat(pixFmt, numPlanes);
    if (err != NO_ERROR) return err;
    err = m_node.setBufferType(numPlanes, bufType, memory);
    if (err != NO_ERROR) return err;
    err = m_node.setFormat();
    if (err != NO_ERROR) return err;
    err = m_node.reqBuffers(numBuffers);
    if (err != NO_ERROR) return err;

    // DMABUF mode: ION-allocated buffers exported as dmabuf.
    // All nodes use V4L2_MEMORY_DMABUF.
    // DMABUF avoids vmap allocation in the kernel — USERPTR would lead to
    // "vmap allocation failed" after ~4 frames (vmap address space exhausted).
    // The dmabuf fds are passed to the ISP (plane 0 = Bayer).
    //
    // Buffer size calculation (SBGGR12 = 12 bit/pixel = 1.5 bytes/pixel):
    // The kernel rounds the width up to a multiple of 10 and
    // then multiplies by 8/5 (= 1.5):
    //   kernel_bpl = (w + 9) / 10 * 10 * 8 / 5
    //   kernel_size = kernel_bpl * h
    // For 3280 (Rear): (3280+9)/10*10*8/5 = 5248 → 5248*2458 = 12,899,584
    // For 1936 (Front): (1936+9)/10*10*8/5 = 3104 → 3104*1090 = 3,383,360
    // Without alignment, w*8/5 would produce a too-small dmabuf for widths
    // not divisible by 10 → EFAULT on QBUF.
    if (memory == V4L2_MEMORY_DMABUF) {
        int alignedW = (w + 9) / 10 * 10;
        size_t planeSizes[2] = { (size_t)(alignedW * 8 / 5) * h, G800FPipeEngine::SPARE_SIZE };
        err = allocBuffersDmabuf(numBuffers, numPlanes, planeSizes);
        if (err != NO_ERROR) return err;
    } else {
        err = allocBuffersMmap(numBuffers, numPlanes);
        if (err != NO_ERROR) return err;
    }

    m_numPlanes = numPlanes;
    ALOGI("%s: FLITE configured %dx%d fmt=0x%x planes=%d bufs=%d mem=%d",
          __FUNCTION__, w, h, pixFmt, numPlanes, numBuffers, (int)memory);
    return NO_ERROR;
}

status_t G800FFlitePipe::queueInitialBuffers()
{
    for (int i = 0; i < m_numBuffers; i++) {
        status_t err = qBufFlite(i);
        if (err != NO_ERROR) {
            ALOGE("%s: FLITE qBuf %d failed: %d", __FUNCTION__, i, err);
            return err;
        }
    }
    ALOGI("%s: queued %d FLITE buffers", __FUNCTION__, m_numBuffers);
    return NO_ERROR;
}

status_t G800FFlitePipe::qBufFlite(int index)
{
    if (index < 0 || index >= m_numBuffers) return BAD_VALUE;
    G800FExynosCameraBuffer* buf = m_buffers[index];
    if (!buf) return BAD_VALUE;

    // DMABUF mode: pass dmabuf fds per plane.
    v4l2_plane planes[VIDEO_MAX_PLANES];
    memset(planes, 0, sizeof(planes));
    bool isDmabuf = (buf->dmaBufFd(0) >= 0);
    for (int p = 0; p < 2; p++) {
        planes[p].bytesused = buf->planeSize(p);
        planes[p].length    = buf->planeSize(p);
        if (isDmabuf)
            planes[p].m.fd = buf->dmaBufFd(p);
    }

    v4l2_buffer vbuf;
    memset(&vbuf, 0, sizeof(vbuf));
    vbuf.index  = index;
    vbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    vbuf.memory = isDmabuf ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
    vbuf.length = 2;
    vbuf.m.planes = planes;

    return m_node.qBuf(&vbuf);
}

status_t G800FFlitePipe::requeueBuffer(int index)
{
    if (index < 0 || index >= m_numBuffers) return BAD_VALUE;
    if (m_fliteInUse[index]) {
        ALOGW("%s: FLITE buffer %d still in use, not requeuing", __FUNCTION__, index);
        return ALREADY_EXISTS;
    }
    return qBufFlite(index);
}

status_t G800FFlitePipe::sensorStreamOn()
{
    // FLITE->sensorStream(true) with value 1 (blocking):
    //   setControl(0x9a100e, on ? 1 : 0)
    //
    // Value 1 = SSTREAM=1, INSTANT=0, NOBLOCK=0 (blocking mode).
    // The kernel calls fimc_is_sensor_front_start() synchronously and
    // waits for the HIC_STREAM_ON firmware response (up to 3s).
    //
    // (0x10000001 = noblock=1 is the instant-capture/AE-stabilization
    // mode, NOT the normal preview start.)
    return m_node.setControl(0x9a100e, 1);
}

status_t G800FFlitePipe::sensorStreamOff()
{
    return m_node.setControl(0x9a100e, 0);
}

void G800FFlitePipe::setAeFlashMode(int mode)
{
    m_aeflashMode = mode;
    // Write to all FLITE shot_ext buffers
    for (int i = 0; i < m_shotExtCount && m_shotExt; i++) {
        if (m_shotExt[i]) {
            m_shotExt[i]->shot.ctl.aa.aeflashMode = (enum aa_ae_flashmode)mode;
        }
    }
}

status_t G800FFlitePipe::m_getBuffer()
{
    // Blocking dequeue: m_node->getBuffer() → exynos_v4l2_dqbuf().
    //   NO poll()! The fd is O_RDWR (not O_NONBLOCK), so
    //   exynos_v4l2_dqbuf() blocks until the kernel delivers a buffer.
    //   STREAMOFF unblocks the wait on stop.
    //
    // The previous implementation used poll() + dqbuf with a timeout.
    // That led to POLLERR when no buffers were queued (buffer
    // starvation when all FLITE buffers are in the ISP pipeline).
    // This problem does not occur when not using poll().
    v4l2_buffer vbuf;
    v4l2_plane planes[VIDEO_MAX_PLANES];
    memset(planes, 0, sizeof(planes));
    memset(&vbuf, 0, sizeof(vbuf));
    vbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    // Memory mode dynamic: DMABUF if buffers have dmabuf fds, otherwise MMAP.
    G800FExynosCameraBuffer* probeBuf = (m_numBuffers > 0) ? m_buffers[0] : NULL;
    vbuf.memory = (probeBuf && probeBuf->dmaBufFd(0) >= 0)
        ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
    vbuf.length = 2;
    vbuf.m.planes = planes;

    status_t err = m_node.dqBuf(&vbuf);
    if (err != NO_ERROR) {
        // -EAGAIN = buffer starvation (queue empty), pass through for retry.
        // Other errors = real problems (STREAMOFF, fd closed, etc.)
        if (err != -EAGAIN) {
            ALOGE("%s: FLITE dqBuf failed: %d", __FUNCTION__, err);
        }
        return err;
    }

    int idx = vbuf.index;
    if (idx < 0 || idx >= m_numBuffers) {
        ALOGE("%s: FLITE dqBuf invalid index %d", __FUNCTION__, idx);
        return BAD_VALUE;
    }

    // Read metadata from shot_ext
    camera2_shot_ext* shot = (m_shotExt && idx < m_shotExtCount) ? m_shotExt[idx] : NULL;
    uint32_t frameCount = shot ? shot->shot.dm.request.frameCount : 0;
    // Sensor timestamp from the kernel (CLOCK_MONOTONIC in ns):
    // Kernel sets dm.sensor.timeStamp = fimc_is_get_timestamp() in
    // fimc-is-device-sensor.c:661 and fimc-is-groupmgr.c:1638.
    // This is the exact capture time of the frame at the sensor.
    // Used as ANDROID_SENSOR_TIMESTAMP in the HAL3 result (AV-Sync!).
    uint64_t timestamp = shot ? shot->shot.dm.sensor.timeStamp : 0;

    // Debug: log FLITE output (dm = what the sensor actually delivered)
    if (shot) {
        //int64_t nowNs = systemTime(SYSTEM_TIME_MONOTONIC);
        //int64_t fliteDeltaMs = (nowNs - (int64_t)shot->shot.dm.sensor.timeStamp) / 1000000;
        //int fliteToIspQSize = m_outputFrameQ ? m_outputFrameQ->size() : -1;
        //ALOGI("%s: FLITE-OUT idx=%d fcount=%u ts=%llu now=%llu delta=%lldms fliteToIspQ=%d | "
        //      "dm: expTime=%lluns frameDur=%lluns sens=%d anGain=%d | "
        //      "aa: aeState=%d aeFlash=%d afState=%d awbState=%d | "
        //      "free=%d req=%d proc=%d complete=%d",
        //      __FUNCTION__, idx, frameCount,
        //      (unsigned long long)shot->shot.dm.sensor.timeStamp,
        //      (unsigned long long)nowNs, (long long)fliteDeltaMs,
        //      fliteToIspQSize,
        //      (unsigned long long)shot->shot.dm.sensor.exposureTime,
        //      (unsigned long long)shot->shot.dm.sensor.frameDuration,
        //      shot->shot.dm.sensor.sensitivity,
        //      shot->shot.dm.sensor.analogGain,
        //      shot->shot.dm.aa.aeState, shot->shot.dm.aa.aeflashMode,
        //      shot->shot.dm.aa.afState, shot->shot.dm.aa.awbState,
        //      shot->free_cnt, shot->request_cnt,
        //      shot->process_cnt, shot->complete_cnt);
    } else {
        //ALOGI("%s: FLITE-OUT idx=%d fcount=%u (no shot_ext)", __FUNCTION__, idx, frameCount);
    }

    // Advance flash state machine per frame.
    // advanceFlashSequence() performs START→ON→METERING→READY transitions
    // based on frame count and AE state. The request worker polls
    // isFlashSequenceReady() and then calls armFlashCapture().
    if (G800FPipeEngine::s_activeInstance) {
        G800FPipeEngine::s_activeInstance->advanceFlashSequence(frameCount);
    }

    // Create frame
    G800FFrame* frame = allocFrame(idx);
    if (!frame) {
        ALOGE("%s: FLITE allocFrame failed, requeuing buffer", __FUNCTION__);
        qBufFlite(idx);
        return NO_MEMORY;
    }
    frame->meta = shot;
    frame->metaSize = sizeof(camera2_shot_ext);
    frame->frameCount = frameCount;
    frame->timestamp = timestamp;
    frame->aeflashMode = m_aeflashMode;
    // Snapshot the SENSOR-node dm/udm now — the ISP dequeue path later
    // overwrites fliteShot->shot.dm with the ISP dm (same meta plane).
    // Reprocessing feeds the bayer frame's own shot to ISP[1] — this
    // snapshot is the equivalent.
    if (shot) {
        memcpy(&frame->sensorDm, &shot->shot.dm, sizeof(frame->sensorDm));
        memcpy(&frame->sensorUdm, &shot->shot.udm, sizeof(frame->sensorUdm));
        frame->hasSensorMeta = true;
    }

    // Check if this frame should be redirected to reprocessing
    if (m_reprocRedirectQ && m_frameSelector) {
        // Flash capture gate: for flash capture, only accept frames
        // AFTER armFlashCapture() has been called (m_mainFlashArmed=true).
        // Frames before that are preflash/metering frames and would result
        // in an overexposed photo (flash has not fired yet).
        // Frames after armFlashCapture() are the real main-flash frames.
        if (m_frameSelector->isFlashCapture() && !m_frameSelector->isMainFlashArmed()) {
            // Preflash frame — discard, requeue FLITE, preview continues
            recycleFrame(frame);
            qBufFlite(idx);
            //ALOGD("%s: FLITE frame %d → preflash skip, main flash not yet armed (fcount=%u)",
            //      __FUNCTION__, idx, frameCount);
            return NO_ERROR;
        }
        // Offer frame to FrameSelector.
        bool accepted = m_frameSelector->offerFrame(frame);
        if (accepted) {
            frame->captureCandidate = true;
            // Mark FLITE buffer as in-use (reprocessing will requeue it later)
            m_fliteInUse[idx] = true;
            m_reprocRedirectQ->push_back(frame);
            //ALOGI("%s: FLITE frame %d → FrameSelector accepted (fcount=%u)",
            //      __FUNCTION__, idx, frameCount);
            return NO_ERROR;
        } else {
            // Frame was rejected by the selector (e.g. flash skip)
            // → requeue FLITE, return frame to pool, normal preview continues
            recycleFrame(frame);
            qBufFlite(idx);
            //ALOGD("%s: FLITE frame %d → FrameSelector skipped (fcount=%u)",
            //      __FUNCTION__, idx, frameCount);
            return NO_ERROR;
        }
    }

    // Push to ISP pipe via output queue
    if (m_outputFrameQ) {
        // IMPORTANT: check queue capacity before pushing!
        // With the 1-in-flight ISP limit, ISP processes only 1 frame per
        // iteration.  If FLITE produces faster than ISP consumes,
        // the queue would consume all 8 FLITE buffers
        // → FLITE buffer starvation → pipeline dead.
        // Solution: if the queue is full, discard the frame and requeue the
        // FLITE buffer immediately.  This is OK for preview — a few lost
        // frames are better than a pipeline deadlock.
        if (m_outputFrameQ->size() >= m_outputFrameQ->getCapacity()) {
            // Queue full — discard frame, requeue FLITE buffer
            recycleFrame(frame);
            qBufFlite(idx);
            //ALOGD("%s: FLITE frame %d dropped (ISP inputQ full, fcount=%u)",
            //      __FUNCTION__, idx, frameCount);
            return NO_ERROR;
        }
        // Mark FLITE buffer as in-use (ISP pipe will requeue after ISP dequeue)
        m_fliteInUse[idx] = true;
        m_outputFrameQ->push_back(frame);
        //int qsz = m_outputFrameQ->size();
        //ALOGD("%s: FLITE frame %d → ISP inputQ (size=%d, fcount=%u)",
        //      __FUNCTION__, idx, qsz, frameCount);
    } else {
        // No downstream pipe — requeue immediately
        recycleFrame(frame);
        qBufFlite(idx);
    }

    return NO_ERROR;
}

status_t G800FFlitePipe::m_putBuffer()
{
    // FLITE is a capture node — m_putBuffer is a no-op here.
    // Buffers are requeued by the ISP pipe after ISP dequeue, or
    // by the reprocessing pipeline after ISP[1] dequeue.
    // Initial buffers are queued by queueInitialBuffers().
    return NO_ERROR;
}

// ============================================================================
// G800FIspPipe
// ============================================================================
G800FIspPipe::G800FIspPipe()
    : G800FPipe(PIPE_ISP, "G800FIspPipe"),
      m_shotExt(NULL), m_shotExtCount(0), m_flitePipe(NULL),
      m_ispInFlightCount(0),
      m_latestAfState(0), m_latestAfMode(0),
      m_latestAeState(0), m_latestAwbState(0),
      m_latestFlashReady(0), m_latestFiringStable(0), m_latestFlashOffReady(0),
      m_latestSensitivity(0), m_latestExposureTime(0),
      m_latestDmAeMode(0), m_latestDmAwbMode(0),
      m_latestFlashDecision(0), m_latestDmFlashMode(0),
      m_latestDmAeflashMode(0),
      m_flashReprocRedirectQ(NULL), m_flashTargetFcount(0), m_flashReprocTarget(0),
      m_memory(V4L2_MEMORY_MMAP)
{
    memset(m_ispInFlight, 0, sizeof(m_ispInFlight));
    memset(m_ispToFliteMap, -1, sizeof(m_ispToFliteMap));
    memset(m_ispFrameMap, 0, sizeof(m_ispFrameMap));
    for (int i = 0; i < 8; i++) m_frameMetaRing[i].frameCount = -1;
}

G800FIspPipe::~G800FIspPipe()
{
}

void G800FIspPipe::setFlashCaptureRedirect(G800FFrameQueue* reprocQ)
{
    Mutex::Autolock l(m_flashRedirectLock);
    m_flashReprocRedirectQ = reprocQ;
    /* New sequence: start phase 1 (wait for firingStable).
     * Prevents a stale m_flashTargetFcount from a
     * previous sequence from starting the new sequence in phase 2. */
    m_flashTargetFcount = 0;
    m_flashReprocTarget = 0;
}

void G800FIspPipe::clearFlashCaptureRedirect()
{
    {
        Mutex::Autolock l(m_flashRedirectLock);
        m_flashReprocRedirectQ = NULL;
        m_flashTargetFcount = 0;
    }
}

status_t G800FIspPipe::create(int videoNodeNum, int sensorId)
{
    m_videoNodeNum = videoNodeNum;
    m_sensorId = sensorId;
    status_t err = m_node.create(videoNodeNum);
    if (err != NO_ERROR) return err;
    err = m_node.open();
    if (err != NO_ERROR) return err;
    // ISP s_input is deferred to setupPipe
    ALOGI("%s: ISP node %d opened (s_input deferred)", __FUNCTION__, videoNodeNum);
    return NO_ERROR;
}

status_t G800FIspPipe::setupPipe(int w, int h, int pixFmt,
                                 int numPlanes, int numBuffers,
                                 v4l2_buf_type bufType,
                                 v4l2_memory memory)
{
    m_memory = memory;  // remember for qBuf/dqBuf

    // s_input first (ischain init)
    status_t err = m_node.setInput(m_sensorId);
    if (err != NO_ERROR) {
        ALOGE("%s: ISP s_input(0x%x) failed: %d", __FUNCTION__, m_sensorId, err);
        return err;
    }

    // Load ISP setfile (tuning parameters).  The ISP setfile is loaded via
    // setControl(V4L2_CID_IS_SET_SETFILE, setfile) in ISP::setupPipe()
    // after s_input.  Without this, the FIMC-IS firmware has no tuning
    // parameters and cannot control sensor exposure/gain, producing
    // black frames with all-zero metadata (expTime=0, aeState=0).
    // Setfile 0 = ISS_SUB_SCENARIO_STILL_PREVIEW (normal preview).
    err = m_node.setControl(V4L2_CID_IS_SET_SETFILE, 0);
    if (err != NO_ERROR) {
        ALOGW("%s: V4L2_CID_IS_SET_SETFILE(0) failed: %d (non-fatal)",
              __FUNCTION__, err);
    } else {
        ALOGI("%s: ISP setfile 0 (STILL_PREVIEW) loaded", __FUNCTION__);
    }

    err = m_node.setSize(w, h);
    if (err != NO_ERROR) return err;
    err = m_node.setColorFormat(pixFmt, numPlanes);
    if (err != NO_ERROR) return err;
    err = m_node.setBufferType(numPlanes, bufType, memory);
    if (err != NO_ERROR) return err;
    err = m_node.setFormat();
    if (err != NO_ERROR) return err;
    err = m_node.reqBuffers(numBuffers);
    if (err != NO_ERROR) return err;

    if (memory == V4L2_MEMORY_DMABUF) {
        // DMABUF mode: ISP reads Bayer via VDMA1 from plane 0.
        // Plane 0 = FLITE buffer's Bayer dmabuf fd (passed at qbuf time),
        // Plane 1 = ISP's own ION-allocated SPARE dmabuf (shot_ext metadata).
        // DMABUF avoids vmap allocation in the kernel (USERPTR → vmap exhaustion
        // after ~4 frames, "vmap allocation for size 12939264 failed").
        int alignedW = (w + 9) / 10 * 10;
        size_t bayerSize = (size_t)(alignedW * 8 / 5) * h;
        size_t planeSizes[2] = { bayerSize, G800FPipeEngine::SPARE_SIZE };
        err = allocBuffersDmabuf(numBuffers, numPlanes, planeSizes);
        if (err != NO_ERROR) return err;
        ALOGI("%s: ISP DMABUF mode — ION buffers allocated (bayer=%zu, spare=%zu)",
              __FUNCTION__, bayerSize, (size_t)G800FPipeEngine::SPARE_SIZE);
    } else if (memory == V4L2_MEMORY_USERPTR) {
        // USERPTR mode (legacy): ISP reads Bayer data via VDMA1 from plane 0.
        // Plane 0 = FLITE buffer's Bayer plane (passed at qbuf time),
        // Plane 1 = ISP's own ION-allocated SPARE plane (shot_ext metadata).
        int alignedW = (w + 9) / 10 * 10;
        size_t bayerSize = (size_t)(alignedW * 8 / 5) * h;
        size_t planeSizes[2] = { bayerSize, G800FPipeEngine::SPARE_SIZE };
        err = allocBuffers(numBuffers, numPlanes, planeSizes);
        if (err != NO_ERROR) return err;
        ALOGI("%s: ISP USERPTR mode — ION buffers allocated (bayer=%zu, spare=%zu)",
              __FUNCTION__, bayerSize, (size_t)G800FPipeEngine::SPARE_SIZE);
    } else {
        // MMAP mode: kernel allocates buffers (has kvaddr/dvaddr for shot_ext).
        err = allocBuffersMmap(numBuffers, numPlanes);
        if (err != NO_ERROR) return err;
    }

    m_numPlanes = numPlanes;
    ALOGI("%s: ISP configured %dx%d fmt=0x%x planes=%d bufs=%d mem=%d",
          __FUNCTION__, w, h, pixFmt, numPlanes, numBuffers, memory);
    return NO_ERROR;
}

status_t G800FIspPipe::queueInitialBuffers()
{
    // preparePipes does FLITE->prepare(), SCP->prepare(), SCC->prepare()
    // but NOT ISP->prepare()!
    //
    // Kernel reason (fimc-is-video-isp.c:381-384):
    //   ISP qbuf checks test_bit(FIMC_IS_QUEUE_STREAM_ON, &queue->state)
    //   and returns -EINVAL if STREAM is OFF.
    //   FLITE/SCC/SCP do NOT have this check — they allow qbuf before STREAMON.
    //
    // ISP qbuf happens only in the thread loop (m_putBuffer) AFTER startPipes
    // (STREAMON). The ISP thread fetches frames from the input queue and does
    // qbuf with shot_ext metadata.
    ALOGI("%s: ISP does not queue initial buffers (kernel requires STREAMON first)", __FUNCTION__);
    return NO_ERROR;
}

status_t G800FIspPipe::qBufIsp(int ispIndex, int fliteIndex, G800FFrame* frame)
{
    if (!m_flitePipe) {
        ALOGE("%s: no FLITE pipe set", __FUNCTION__);
        return INVALID_OPERATION;
    }
    G800FExynosCameraBuffer* fliteBuf = m_flitePipe->getBuffer(fliteIndex);
    G800FExynosCameraBuffer* ispBuf   = getBuffer(ispIndex);
    if (!fliteBuf || !ispBuf) return BAD_VALUE;

    // Copy shot_ext from FLITE to ISP (3 memcpy operations)
    camera2_shot_ext* fliteShot = m_flitePipe->getShotExt(fliteIndex);
    camera2_shot_ext* ispShot = (m_shotExt && ispIndex < m_shotExtCount)
        ? m_shotExt[ispIndex] : NULL;
    if (fliteShot && ispShot) {
        memcpy(&ispShot->shot.ctl, &fliteShot->shot.ctl, sizeof(fliteShot->shot.ctl));
        memcpy(&ispShot->shot.dm,  &fliteShot->shot.dm,  sizeof(fliteShot->shot.dm));
        memcpy(&ispShot->node_group, &fliteShot->node_group, sizeof(fliteShot->node_group));
        // Individual field copies
        ispShot->setfile = fliteShot->setfile;
        ispShot->drc_bypass = fliteShot->drc_bypass;
        ispShot->dis_bypass = fliteShot->dis_bypass;
        ispShot->dnr_bypass = fliteShot->dnr_bypass;
        ispShot->fd_bypass = fliteShot->fd_bypass;
        ispShot->shot.dm.request.frameCount = fliteShot->shot.dm.request.frameCount;
        ispShot->shot.ctl.request.frameCount = fliteShot->shot.ctl.request.frameCount;
        ispShot->shot.magicNumber = SHOT_MAGIC_NUMBER;
    }

    // Build v4l2_buffer.
    // DMABUF mode: plane 0 = FLITE Bayer dmabuf fd (VDMA1 input),
    //                    plane 1 = ISP's ION-allocated SPARE dmabuf fd.
    // USERPTR mode (legacy): plane 0 = FLITE vaddr, plane 1 = ISP SPARE vaddr.
    // MMAP mode:    plane 0 = ISP's own MMAP buffer, plane 1 = ISP's SPARE.
    v4l2_plane planes[VIDEO_MAX_PLANES];
    memset(planes, 0, sizeof(planes));

    if (m_memory == V4L2_MEMORY_DMABUF) {
        // Plane 0: FLITE Bayer dmabuf — ISP reads via VDMA1 (DMA-input mode)
        planes[0].m.fd = fliteBuf->dmaBufFd(0);
        planes[0].bytesused = fliteBuf->planeSize(0);
        planes[0].length    = fliteBuf->planeSize(0);
        // Plane 1: ISP metadata (shot_ext) from ION-allocated SPARE dmabuf
        planes[1].m.fd = ispBuf->dmaBufFd(1);
        planes[1].bytesused = ispBuf->planeSize(1);
        planes[1].length    = ispBuf->planeSize(1);
    } else if (m_memory == V4L2_MEMORY_USERPTR) {
        // Plane 0: FLITE Bayer data — ISP reads it via VDMA1 (DMA-input mode)
        planes[0].m.userptr = reinterpret_cast<unsigned long>(fliteBuf->planeVaddr(0));
        planes[0].bytesused = fliteBuf->planeSize(0);
        planes[0].length    = fliteBuf->planeSize(0);
        // Plane 1: ISP metadata (shot_ext) from ION-allocated SPARE plane
        planes[1].m.userptr = reinterpret_cast<unsigned long>(ispBuf->planeVaddr(1));
        planes[1].bytesused = ispBuf->planeSize(1);
        planes[1].length    = ispBuf->planeSize(1);
    } else {
        // MMAP mode: kernel knows buffer addresses from querybuf
        planes[0].bytesused = ispBuf->planeSize(0);
        planes[0].length    = ispBuf->planeSize(0);
        planes[1].bytesused = ispBuf->planeSize(1);
        planes[1].length    = ispBuf->planeSize(1);
    }

    v4l2_buffer vbuf;
    memset(&vbuf, 0, sizeof(vbuf));
    vbuf.index  = ispIndex;
    vbuf.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    vbuf.memory = m_memory;
    vbuf.length = 2;
    vbuf.m.planes = planes;

    status_t err = m_node.qBuf(&vbuf);
    if (err != NO_ERROR) {
        ALOGE("%s: ISP qBuf %d (flite=%d) failed: %d", __FUNCTION__, ispIndex, fliteIndex, err);
        return err;
    }

    // Track in-flight
    Mutex::Autolock l(m_inFlightLock);
    m_ispInFlight[ispIndex] = true;
    m_ispInFlightCount++;
    m_ispToFliteMap[ispIndex] = fliteIndex;

    // Debug: log ISP input configuration (shot_ext.ctl + node_group)
    //if (ispShot) {
    //    struct camera2_node_group *ng = &ispShot->node_group;
    //    //ALOGI("%s: ISP-IN ispIdx=%d fliteIdx=%d fcount=%u | "
        //      "ctl: expTime=%lluns frameDur=%lluns sens=%d | "
        //      "aa: aeMode=%d aeFlash=%d afMode=%d awbMode=%d scene=%d "
        //      "aeFps=[%d,%d] afTrigger=%d | "
        //      "lens: focusDist=%d | "
        //      "ng: leader vid=%d req=%d in=[%d,%d,%d,%d] out=[%d,%d,%d,%d] | "
        //      "cap0(SCC) vid=%d req=%d in=[%d,%d,%d,%d] out=[%d,%d,%d,%d] | "
        //      "cap1(SCP) vid=%d req=%d in=[%d,%d,%d,%d] out=[%d,%d,%d,%d] | "
        //      "setfile=%d drc=%d dis=%d dnr=%d fd=%d | "
        //      "inFlight=%d",
        //      __FUNCTION__, ispIndex, fliteIndex,
        //      ispShot->shot.dm.request.frameCount,
        //      (unsigned long long)ispShot->shot.ctl.sensor.exposureTime,
        //      (unsigned long long)ispShot->shot.ctl.sensor.frameDuration,
        //      ispShot->shot.ctl.sensor.sensitivity,
        //      ispShot->shot.ctl.aa.aeMode, ispShot->shot.ctl.aa.aeflashMode,
        //      ispShot->shot.ctl.aa.afMode, ispShot->shot.ctl.aa.awbMode,
        //      ispShot->shot.ctl.aa.sceneMode,
        //      ispShot->shot.ctl.aa.aeTargetFpsRange[0],
        //      ispShot->shot.ctl.aa.aeTargetFpsRange[1],
        //      ispShot->shot.ctl.aa.afTrigger,
        //      ispShot->shot.ctl.lens.focusDistance,
        //      ng->leader.vid, ng->leader.request,
        //      ng->leader.input.cropRegion[0], ng->leader.input.cropRegion[1],
        //      ng->leader.input.cropRegion[2], ng->leader.input.cropRegion[3],
        //      ng->leader.output.cropRegion[0], ng->leader.output.cropRegion[1],
        //      ng->leader.output.cropRegion[2], ng->leader.output.cropRegion[3],
        //      ng->capture[0].vid, ng->capture[0].request,
        //      ng->capture[0].input.cropRegion[0], ng->capture[0].input.cropRegion[1],
        //      ng->capture[0].input.cropRegion[2], ng->capture[0].input.cropRegion[3],
        //      ng->capture[0].output.cropRegion[0], ng->capture[0].output.cropRegion[1],
        //      ng->capture[0].output.cropRegion[2], ng->capture[0].output.cropRegion[3],
        //      ng->capture[1].vid, ng->capture[1].request,
        //      ng->capture[1].input.cropRegion[0], ng->capture[1].input.cropRegion[1],
        //      ng->capture[1].input.cropRegion[2], ng->capture[1].input.cropRegion[3],
        //      ng->capture[1].output.cropRegion[0], ng->capture[1].output.cropRegion[1],
        //      ng->capture[1].output.cropRegion[2], ng->capture[1].output.cropRegion[3],
        //      ispShot->setfile, ispShot->drc_bypass, ispShot->dis_bypass,
        //      ispShot->dnr_bypass, ispShot->fd_bypass,
        //      m_ispInFlightCount);
    //}

    return NO_ERROR;
}

status_t G800FIspPipe::m_getBuffer()
{
    // Check if there are in-flight buffers
    {
        Mutex::Autolock l(m_inFlightLock);
        if (m_ispInFlightCount <= 0) {
            //ALOGD("%s: ISP no in-flight buffers (count=0) → EAGAIN", __FUNCTION__);
            return -EAGAIN;
        }
    }

    // Blocking dequeue: getBuffer() → exynos_v4l2_dqbuf().
    //   NO poll()! Blocks until ISP is done.
    v4l2_buffer vbuf;
    v4l2_plane planes[VIDEO_MAX_PLANES];
    memset(planes, 0, sizeof(planes));
    memset(&vbuf, 0, sizeof(vbuf));
    vbuf.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    vbuf.memory = m_memory;
    vbuf.length = 2;
    vbuf.m.planes = planes;

    status_t err = m_node.dqBuf(&vbuf);
    if (err != NO_ERROR) {
        if (err != -EAGAIN) {
            ALOGE("%s: ISP dqBuf failed: %d", __FUNCTION__, err);
        }
        return err;
    }

    int ispIdx = vbuf.index;
    int fliteIdx = -1;
    {
        Mutex::Autolock l(m_inFlightLock);
        m_ispInFlight[ispIdx] = false;
        m_ispInFlightCount--;
        fliteIdx = m_ispToFliteMap[ispIdx];
        m_ispToFliteMap[ispIdx] = -1;
    }

    // m_numOfRunningFrame-- after m_getBuffer.  This
    // controls the drain mode in the threadLoop: at < 2 the input queue
    // is fully drained, at >= 2 only one m_putBuffer() per iteration.
    // Without this update, m_numOfRunningFrame stays at 0 → drain mode
    // always active → ISP gets flooded with all FLITE frames at once →
    // inFlight rises to 8 → DQBUF EINVAL → pipeline freeze.
    if (m_numOfRunningFrame > 0)
        m_numOfRunningFrame--;

    // Read 3A state from ISP dm
    camera2_shot_ext* shot = (m_shotExt && ispIdx < m_shotExtCount)
        ? m_shotExt[ispIdx] : NULL;
    if (shot) {
        // AF state transitions — shows whether the firmware's AF engine
        // actually scans (AA_AFSTATE: 1=INACTIVE 2=PASSIVE_SCAN
        // 3=ACTIVE_SCAN 4=ACQUIRED 5=FAILED).
        if (shot->shot.dm.aa.afState != m_latestAfState)
            ALOGI("AFSTATE %d -> %d (afMode=%d fcount=%u)",
                  m_latestAfState, shot->shot.dm.aa.afState,
                  shot->shot.dm.aa.afMode, shot->shot.dm.request.frameCount);
        m_latestAfState     = shot->shot.dm.aa.afState;
        m_latestAfMode      = shot->shot.dm.aa.afMode;
        m_latestAeState     = shot->shot.dm.aa.aeState;
        m_latestAwbState    = shot->shot.dm.aa.awbState;
        m_latestFlashReady   = shot->shot.dm.flash.flashReady;
        m_latestFiringStable = shot->shot.dm.flash.firingStable;
        m_latestFlashOffReady = shot->shot.dm.flash.flashOffReady;
        m_latestSensitivity  = shot->shot.dm.sensor.sensitivity;
        m_latestExposureTime = shot->shot.dm.sensor.exposureTime;
        // dm-echoed control values: what the firmware actually applied
        // (diagnosing green-tint: does the ISP hold locked/flash WB gains?)
        m_latestDmAeMode      = shot->shot.dm.aa.aeMode;
        m_latestDmAwbMode     = shot->shot.dm.aa.awbMode;
        m_latestDmAeflashMode = shot->shot.dm.aa.aeflashMode;
        m_latestFlashDecision = shot->shot.dm.flash.decision;
        m_latestDmFlashMode   = shot->shot.dm.flash.flashMode;
        /* per-frame diagnostic log disabled.
        if (m_postFlashLogFrames > 0) {
            m_postFlashLogFrames--;
            ALOGI("POSTFLASH-DM fcount=%u: aeState=%d awbState=%d afState=%d "
                  "aeMode=%d awbMode=%d aeFlash=%d flashMode=%d decision=%d "
                  "exp=%lldns iso=%u firingStable=%d",
                  shot->shot.dm.request.frameCount,
                  m_latestAeState, m_latestAwbState, m_latestAfState,
                  m_latestDmAeMode, m_latestDmAwbMode, m_latestDmAeflashMode,
                  m_latestDmFlashMode, m_latestFlashDecision,
                  (long long)m_latestExposureTime, m_latestSensitivity,
                  m_latestFiringStable);
        }
        */

        /* per-frame diagnostic log disabled.
        if (G800FPipeEngine::s_activeInstance &&
            G800FPipeEngine::s_activeInstance->isFlashSequenceActive()) {
            ALOGI("ISP-UDM fcount=%u: awb[vsLen=%u nz=%d g11=%u g12=%u g13=%u "
                  "g14=%u g15=%u] int.vs2[100..103]=%u %u %u %u",
                  shot->shot.dm.request.frameCount,
                  shot->shot.udm.awb.vsLength,
                  udmNzCount(shot->shot.udm.awb.vendorSpecific),
                  shot->shot.udm.awb.vendorSpecific[11],
                  shot->shot.udm.awb.vendorSpecific[12],
                  shot->shot.udm.awb.vendorSpecific[13],
                  shot->shot.udm.awb.vendorSpecific[14],
                  shot->shot.udm.awb.vendorSpecific[15],
                  shot->shot.udm.internal.vendorSpecific2[100],
                  shot->shot.udm.internal.vendorSpecific2[101],
                  shot->shot.udm.internal.vendorSpecific2[102],
                  shot->shot.udm.internal.vendorSpecific2[103]);
        }
        */

        camera2_shot_ext* fliteShot = (m_flitePipe && fliteIdx >= 0)
            ? m_flitePipe->getShotExt(fliteIdx) : NULL;
        if (fliteShot) {
            memcpy(&fliteShot->shot.dm, &shot->shot.dm, sizeof(shot->shot.dm));
        }
        // Publish this frame's firmware results (dm + udm + uctl — incl.
        // computed WB gains, exposure and flash metering) into the meta
        // ring.
        // Decoupled from the flite buffer, which gets recycled while the
        // reproc thread still needs this frame's metadata.
        IspFrameMeta& slot =
            m_frameMetaRing[shot->shot.dm.request.frameCount & 7];
        slot.dm   = shot->shot.dm;
        slot.uctl = shot->shot.uctl;
        slot.udm  = shot->shot.udm;
        __sync_synchronize();
        slot.frameCount = (int32_t)shot->shot.dm.request.frameCount;
    }

    // Get the frame from m_ispFrameMap — the same frame that
    // m_putBuffer stored here.  The frame flows as a single object
    // through the pipeline.  We enrich it with ISP metadata and
    // pass it on to SCP (via m_outputFrameQ = m_ispToScpQ).
    G800FFrame* frame = NULL;
    {
        Mutex::Autolock l(m_inFlightLock);
        frame = m_ispFrameMap[ispIdx];
        m_ispFrameMap[ispIdx] = NULL;
    }

    if (!frame) {
        // Fallback: if no frame is in the map (e.g. after pipeline reset),
        // allocate a new one.  This is a safety net, not the regular path.
        ALOGW("%s: ISP no frame in map for idx=%d, allocating new", __FUNCTION__, ispIdx);
        frame = allocFrame(ispIdx);
        if (!frame) {
            ALOGE("%s: ISP allocFrame fallback failed", __FUNCTION__);
            return NO_MEMORY;
        }
    }

    // Enrich frame with ISP metadata
    frame->meta = shot;
    frame->metaSize = sizeof(camera2_shot_ext);
    frame->frameCount = shot ? shot->shot.dm.request.frameCount : 0;
    // Propagate sensor timestamp from kernel (for AV-Sync).
    // Used as ANDROID_SENSOR_TIMESTAMP in handleRequest.
    frame->timestamp = shot ? shot->shot.dm.sensor.timeStamp : 0;
    frame->afState = m_latestAfState;
    frame->aeState = m_latestAeState;
    frame->awbState = m_latestAwbState;
    frame->sensitivity = m_latestSensitivity;
    frame->exposureTime = m_latestExposureTime;

    G800FFrameQueue* flashRedirectQ = NULL;
    G800FFrame* redirectFrame = NULL;
    {
        Mutex::Autolock l(m_flashRedirectLock);
        if (m_flashReprocRedirectQ && shot) {
            if (m_flashTargetFcount == 0) {
                /* Phase 1: wait for firingStable==1 (in MAIN_ON the
                 * wire carries ON(4), not CAPTURE); the capture target
                 * is the frame right after it. */
                if (shot->shot.dm.flash.firingStable == 1) {
                    m_flashTargetFcount = frame->frameCount + 1;
                    /* Release firingStable frame normally (preview).
                     * Fall through to the normal path below. */
                }
            } else {
                /* Phase 2: wait for frame with frameCount >= target.
                 * All earlier Bayer frames are discarded until
                 * bufferFcount == waitFcount. */
                if (frame->frameCount >= m_flashTargetFcount) {
                    flashRedirectQ = m_flashReprocRedirectQ;
                    m_flashReprocRedirectQ = NULL;
                    redirectFrame = frame;
                    /* Remember the confirmed target for the warm path — readable
                     * without waiting for the slow reproc output. */
                    m_flashReprocTarget = frame->frameCount;
                    if (frame->frameCount > m_flashTargetFcount) {
                        ALOGW("%s: flash target fcount=%u missed, using fcount=%u",
                              __FUNCTION__, m_flashTargetFcount,
                              frame->frameCount);
                    }
                    m_flashTargetFcount = 0;
                }
                /* Release frames before the target normally (preview). */
            }
        }
    }

    if (flashRedirectQ && redirectFrame) {
        redirectFrame->captureCandidate = true;
        status_t redirectErr = flashRedirectQ->push_back(redirectFrame);
        if (redirectErr == NO_ERROR) {
            //ALOGI("%s: flash target fcount=%u redirected to reproc (fliteIdx=%d)",
            //      __FUNCTION__, redirectFrame->frameCount,
            //      redirectFrame->bufferIndex);
            return NO_ERROR;
        }
        ALOGE("%s: flash redirect failed: %d (fcount=%u fliteIdx=%d)",
              __FUNCTION__, redirectErr, redirectFrame->frameCount,
              redirectFrame->bufferIndex);
        int redirectIdx = redirectFrame->bufferIndex;
        if (redirectIdx >= 0 && m_flitePipe) {
            m_flitePipe->clearFliteInUse(redirectIdx);
            m_flitePipe->requeueBuffer(redirectIdx);
        }
        recycleFrame(redirectFrame);
        return NO_ERROR;
    }

    if (fliteIdx >= 0 && m_flitePipe) {
        m_flitePipe->clearFliteInUse(fliteIdx);
        m_flitePipe->requeueBuffer(fliteIdx);
    }

    if (m_outputFrameQ) {
        // Overflow protection: if the output queue is full, properly
        // release the oldest frame (recycleFrame).  This prevents
        // frame leaks when the consumer has no further requests
        // (e.g. after close() but before stopPreview()).
        if (m_outputFrameQ->size() >= m_outputFrameQ->getCapacity()) {
            G800FFrame* old = NULL;
            if (m_outputFrameQ->try_pop_front(&old) == NO_ERROR && old) {
                //ALOGD("%s: ISP outputQ full, recycling oldest frame", __FUNCTION__);
                recycleFrame(old);
            }
        }
        m_outputFrameQ->push_back(frame);
        //ALOGD("%s: ISP frame → outputQ (size=%d)", __FUNCTION__, m_outputFrameQ->size());
    } else {
        recycleFrame(frame);
    }
    // m_outputFrameQ (m_ispToScpQ) is consumed by G800FScpPipe::m_getBuffer
    // — SCP pops the same frame and adds preview data.
    // The frame flows on as a single object (ISP → SCP → HAL3).

    return NO_ERROR;
}

status_t G800FIspPipe::m_putBuffer()
{
    // Pop a frame from the input queue (from FLITE pipe)
    if (!m_inputFrameQ) return NO_ERROR;

    G800FFrame* frame = NULL;
    status_t err = m_inputFrameQ->try_pop_front(&frame);
    if (err != NO_ERROR || !frame) {
        //ALOGD("%s: ISP inputQ empty → EAGAIN", __FUNCTION__);
        return -EAGAIN;
    }

    // Use ISP buffer with same index as FLITE buffer.
    // CRITICAL: The Samsung FIMC-IS kernel (fimc-is-video.c line 557)
    // rejects qbuf if the buffer address changes for the same index:
    //   "buffer %d plane %d is changed(%08X != %08X)"
    // With USERPTR, plane 0 is the FLITE buffer's vaddr.  If we use a
    // different FLITE buffer for the same ISP index, the address changes
    // and the kernel returns -EINVAL after ~200 frames.
    // Fix: 1:1 mapping ISP buffer N ↔ FLITE buffer N.  Since we have
    // 8 of each, ISP buffer N always gets FLITE buffer N's address.
    int ispIdx = frame->bufferIndex;
    if (ispIdx < 0 || ispIdx >= m_numBuffers) {
        ALOGE("%s: invalid fliteIdx=%d (numBuffers=%d)", __FUNCTION__, ispIdx, m_numBuffers);
        recycleFrame(frame);
        return BAD_VALUE;
    }
    {
        Mutex::Autolock l(m_inFlightLock);
        if (m_ispInFlight[ispIdx]) {
            // ISP buffer for this FLITE index is already in-flight.
            // Push frame back to input queue and return EAGAIN.
            m_inputFrameQ->push_front(frame);
            //ALOGD("%s: ISP buf %d in-flight (fliteIdx=%d) → EAGAIN",
            //      __FUNCTION__, ispIdx, frame->bufferIndex);
            return -EAGAIN;
        }
    }

    // Queue ISP with FLITE Bayer
    err = qBufIsp(ispIdx, frame->bufferIndex, frame);
    if (err != NO_ERROR) {
        ALOGE("%s: ISP qBuf failed: %d (ispIdx=%d fliteIdx=%d)", __FUNCTION__, err, ispIdx, frame->bufferIndex);
        m_inputFrameQ->push_front(frame);
        return err;
    }

    //ALOGD("%s: ISP qBuf ispIdx=%d fliteIdx=%d → inFlight=%d",
    //      __FUNCTION__, ispIdx, frame->bufferIndex, m_ispInFlightCount);

    // Do NOT recycle the frame — the frame flows on as a single
    // object.  We store the pointer in m_ispFrameMap so that
    // m_getBuffer can retrieve it later once the ISP buffer has been
    // fully processed.  The frame then carries the ISP metadata and is
    // passed on to SCP.
    {
        Mutex::Autolock l(m_inFlightLock);
        m_ispFrameMap[ispIdx] = frame;
    }

    // m_numOfRunningFrame++ after m_putBuffer.  Together with the decrement
    // in m_getBuffer, this controls the drain mode in the threadLoop:
    //   m_numOfRunningFrame < 2 → drain input queue (all frames)
    //   m_numOfRunningFrame >= 2 → only one m_putBuffer() per iteration
    // This limits the number of simultaneously in-flight ISP buffers and
    // prevents the pipeline freeze (inFlight→8 → DQBUF EINVAL).
    m_numOfRunningFrame++;

    return NO_ERROR;
}

// Recycle all frames still stuck in m_ispFrameMap.
// Called by PipeEngine::stopPreview().
void G800FIspPipe::recycleStuckFrames()
{
    Mutex::Autolock l(m_inFlightLock);
    for (int i = 0; i < 8; i++) {
        if (m_ispFrameMap[i]) {
            ALOGD("%s: recycling stuck ISP frame idx=%d", __FUNCTION__, i);
            G800FFrame* stuck = m_ispFrameMap[i];
            m_ispFrameMap[i] = NULL;
            // recycleFrame without lock (we already hold m_inFlightLock,
            // but recycleFrame uses the shared pool lock — no
            // deadlock risk since they are different locks).
            G800FPipe::recycleFrame(stuck);
        }
    }
}

// ============================================================================
// G800FScpPipe
// ============================================================================
G800FScpPipe::G800FScpPipe()
    : G800FPipe(PIPE_SCP, "G800FScpPipe"),
      m_stream(NULL), m_streamCount(0),
      m_previewW(0), m_previewH(0)
{
    memset(m_scpInUse, 0, sizeof(m_scpInUse));
    m_previewQ.setCapacity(4);
}

G800FScpPipe::~G800FScpPipe()
{
}

status_t G800FScpPipe::create(int videoNodeNum, int sensorId)
{
    m_videoNodeNum = videoNodeNum;
    m_sensorId = sensorId;
    status_t err = m_node.create(videoNodeNum);
    if (err != NO_ERROR) return err;
    err = m_node.open();
    if (err != NO_ERROR) return err;
    err = m_node.setInput(sensorId);
    ALOGI("%s: SCP node %d opened, sensorId=0x%x", __FUNCTION__, videoNodeNum, sensorId);
    return err;
}

status_t G800FScpPipe::setupPipe(int w, int h, int pixFmt,
                                 int numPlanes, int numBuffers,
                                 v4l2_buf_type bufType,
                                 v4l2_memory memory)
{
    status_t err = m_node.setSize(w, h);
    if (err != NO_ERROR) return err;
    err = m_node.setColorFormat(pixFmt, numPlanes);
    if (err != NO_ERROR) return err;
    err = m_node.setBufferType(numPlanes, bufType, memory);
    if (err != NO_ERROR) return err;
    err = m_node.setFormat();
    if (err != NO_ERROR) return err;

    if (memory == V4L2_MEMORY_DMABUF) {
        // DMABUF mode: userspace allocates non-cacheable ION buffers.
        // The kernel's vb2 DMABUF path performs cache sync on each
        // qbuf/dqbuf — fixing the FIMC-IS SCP cache coherency issue.
        // Plane sizes come from G_FMT (sizeimage) after setFormat().
        //
        // IMPORTANT: buffers must be allocated with 16-byte-aligned height
        // because the FIMC M2M scaler requires 16-byte alignment.
        // The SCP outputs 1080 lines, but the FIMC expects 1088.
        // vb2_ion_attach_dmabuf() checks: dbuf->size >= sizeimage(1088).
        // Without this alignment adjustment, QBUF fails with EFAULT.
        unsigned int hAligned = (unsigned int)((h + 0xf) & ~0xf);
        size_t planeSizes[VIDEO_MAX_PLANES];
        for (int p = 0; p < numPlanes; p++) {
            unsigned int sz = m_node.getPlaneSizeImage(p);
            if (sz == 0) {
                // Fallback: calculate from dimensions
                if (p == 0)      sz = (size_t)(w * h);          // Y
                else if (p == 1) sz = (size_t)(w * h / 2);      // VU
                else             sz = 32768;                    // spare
                ALOGW("%s: SCP plane %d sizeimage=0, using fallback %u",
                      __FUNCTION__, p, sz);
            }
            // Force at least the 16-byte-aligned size so the
            // FIMC M2M scaler accepts the DMABUF (vb2_ion_attach_dmabuf
            // checks dbuf->size >= sizeimage).
            if (p == 0) {
                unsigned int alignedSz = (size_t)(w * hAligned);
                if (sz < alignedSz) sz = alignedSz;
            } else if (p == 1) {
                unsigned int alignedSz = (size_t)(w * hAligned / 2);
                if (sz < alignedSz) sz = alignedSz;
            }
            planeSizes[p] = sz;
        }
        err = m_node.reqBuffers(numBuffers);
        if (err != NO_ERROR) return err;
        err = allocBuffersDmabuf(numBuffers, numPlanes, planeSizes);
        if (err != NO_ERROR) return err;
    } else {
        // MMAP mode: kernel allocates buffers
        err = m_node.reqBuffers(numBuffers);
        if (err != NO_ERROR) return err;
        err = allocBuffersMmap(numBuffers, numPlanes);
        if (err != NO_ERROR) return err;
    }

    m_numPlanes = numPlanes;
    ALOGI("%s: SCP configured %dx%d fmt=0x%x planes=%d bufs=%d mem=%d",
          __FUNCTION__, w, h, pixFmt, numPlanes, numBuffers, memory);
    return NO_ERROR;
}

status_t G800FScpPipe::queueInitialBuffers()
{
    bool isDmabuf = (m_buffers[0] && m_buffers[0]->dmaBufFd(0) >= 0);
    for (int i = 0; i < m_numBuffers; i++) {
        G800FExynosCameraBuffer* buf = m_buffers[i];
        v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(planes, 0, sizeof(planes));
        for (int p = 0; p < m_numPlanes; p++) {
            planes[p].bytesused = buf->planeSize(p);
            planes[p].length    = buf->planeSize(p);
            if (isDmabuf)
                planes[p].m.fd = buf->dmaBufFd(p);
        }
        v4l2_buffer vbuf;
        memset(&vbuf, 0, sizeof(vbuf));
        vbuf.index  = i;
        vbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        vbuf.memory = isDmabuf ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
        vbuf.length = m_numPlanes;
        vbuf.m.planes = planes;
        status_t err = m_node.qBuf(&vbuf);
        if (err != NO_ERROR) {
            ALOGE("%s: SCP qBuf %d failed: %d (mem=%d)",
                  __FUNCTION__, i, err, vbuf.memory);
            return err;
        }
    }
    ALOGI("%s: queued %d SCP buffers (mem=%s)",
          __FUNCTION__, m_numBuffers, isDmabuf ? "DMABUF" : "MMAP");
    return NO_ERROR;
}

status_t G800FScpPipe::getPreviewFrame(G800FFrame** frame)
{
    status_t err = m_previewQ.try_pop_front(frame);
    if (err == NO_ERROR && *frame) {
        //int64_t nowNs = systemTime(SYSTEM_TIME_MONOTONIC);
        //int64_t deltaMs = (nowNs - (int64_t)(*frame)->timestamp) / 1000000;
        //ALOGI("%s: previewQ pop fcount=%u ts=%llu now=%llu delta=%lldms previewQ=%d/%d",
        //      __FUNCTION__, (*frame)->frameCount,
        //      (unsigned long long)(*frame)->timestamp,
        //      (unsigned long long)nowNs, (long long)deltaMs,
        //      m_previewQ.size(), m_previewQ.getCapacity());
    }
    return err;
}

// Requeue a SCP V4L2 buffer by index (used by m_getBuffer after data copy).
void G800FScpPipe::requeueScpBuffer(int idx)
{
    if (idx < 0 || idx >= m_numBuffers) return;
    G800FExynosCameraBuffer* buf = m_buffers[idx];
    if (!buf) return;
    bool isDmabuf = (buf->dmaBufFd(0) >= 0);
    v4l2_plane planes[VIDEO_MAX_PLANES];
    memset(planes, 0, sizeof(planes));
    for (int p = 0; p < m_numPlanes; p++) {
        planes[p].bytesused = buf->planeSize(p);
        planes[p].length    = buf->planeSize(p);
        if (isDmabuf)
            planes[p].m.fd = buf->dmaBufFd(p);
    }
    v4l2_buffer vbuf;
    memset(&vbuf, 0, sizeof(vbuf));
    vbuf.index  = idx;
    vbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    vbuf.memory = isDmabuf ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
    vbuf.length = m_numPlanes;
    vbuf.m.planes = planes;
    m_node.qBuf(&vbuf);
    Mutex::Autolock l(m_inUseLock);
    m_scpInUse[idx] = false;
}

void G800FScpPipe::releasePreviewFrame(G800FFrame* frame)
{
    if (!frame) return;
    // Direct V4L2 path: the V4L2 buffer was NOT requeued in m_getBuffer.
    // Here we return the V4L2 buffer, then recycle the frame.
    if (frame->ownsScpBuffer && frame->scpBufferIndex >= 0) {
        requeueScpBuffer(frame->scpBufferIndex);
        frame->ownsScpBuffer = false;
        frame->scpBufferIndex = -1;
        frame->buffer = NULL;  // don't access after requeue
    }
    // recycleFrame() releases ownsPreviewData memory (if any).
    recycleFrame(frame);
}

status_t G800FScpPipe::m_getBuffer()
{
    // Dequeue model:
    //   1. DQ from the SCP V4L2 node  (V4L2 FIRST, then input queue!)
    //   2. Requeue V4L2 buffer immediately (no buffer starvation!)
    //   3. Copy Y+VU data into own malloc buffer
    //   4. Pop frame from m_inputFrameQ (= m_ispToScpQ, pushed by ISP)
    //      — the same frame that flowed through FLITE → ISP
    //   5. Add preview data to the frame
    //   6. Push frame into preview queue (with ISP metadata + preview data)
    //
    // IMPORTANT: V4L2 dqbuf MUST happen before input-queue pop!  Otherwise
    // the FIMC-IS pipeline stalls: the kernel needs free SCP buffers
    // to process new ISP frames.  If SCP doesn't dequeue →
    // no free SCP buffers → ISP blocks → 1 fps instead of 30 fps.
    //
    // If the input queue is empty (ISP not ready yet), we still
    // dequeue, copy the data, requeue the V4L2 buffer and
    // store the data in a "pending" frame that gets associated on the next
    // ISP frame.  Fallback: allocate a new frame.

    // 1. DQ from the SCP V4L2 node
    bool isDmabuf = (m_buffers[0] && m_buffers[0]->dmaBufFd(0) >= 0);
    v4l2_buffer vbuf;
    v4l2_plane planes[VIDEO_MAX_PLANES];
    memset(planes, 0, sizeof(planes));
    memset(&vbuf, 0, sizeof(vbuf));
    vbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    vbuf.memory = isDmabuf ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
    vbuf.length = m_numPlanes;
    vbuf.m.planes = planes;

    status_t err = m_node.dqBuf(&vbuf);
    if (err != NO_ERROR) {
        if (err != -EAGAIN) {
            ALOGE("%s: SCP dqBuf failed: %d (mem=%d)",
                  __FUNCTION__, err, vbuf.memory);
        }
        return err;
    }

    int idx = vbuf.index;
    if (idx < 0 || idx >= m_numBuffers) return BAD_VALUE;

    G800FExynosCameraBuffer* buf = m_buffers[idx];
    if (!buf) {
        ALOGE("%s: SCP buffer %d is NULL", __FUNCTION__, idx);
        requeueScpBuffer(idx);
        return BAD_VALUE;
    }

    // NOTE: Cache sync (msync/__builtin___clear_cache) was tested and
    // brought no improvement — the 64-pixel stripes have a different
    // cause than CPU cache coherency.  See TODO.md for further analysis.

    // Determine SCP buffer size.
    // m_previewW/H is set via setPreviewSize() to the SCP V4L2 buffer size
    // (= BDS size, e.g. 1920x1080). We copy the COMPLETE
    // SCP buffer and pass this size as previewStride/previewWidth/
    // previewHeight to the frame. copyPreviewToStreamFromData then scales
    // from scpBufSize to the target stream size (e.g. 1280x720 preview or
    // 1920x1080 video).
    int pw = m_previewW;
    int ph = m_previewH;
    if (pw <= 0 || ph <= 0) {
        ALOGE("%s: SCP preview size not set (%dx%d)", __FUNCTION__, pw, ph);
        requeueScpBuffer(idx);
        return BAD_VALUE;
    }

    // Y + VU plane sizes (NV21M: Y = w*h, VU = w*(h/2))
    size_t ySize  = (size_t)(pw * ph);
    size_t vuSize = (size_t)(pw * ph / 2);

    // Direct V4L2 path: NO malloc, NO memcpy.
    // The SCP V4L2 buffer stays dequeued until releasePreviewFrame().
    // servePreviewRequest reads directly from buf->planeVaddr(0/1).
    // copyPreviewToStream scales directly from V4L2 → gralloc in one pass.
    // This eliminates 3MB malloc + 3MB memcpy per frame.
    uint8_t* srcY  = (uint8_t*)buf->planeVaddr(0);
    uint8_t* srcVU = (uint8_t*)buf->planeVaddr(1);
    if (!srcY || !srcVU) {
        ALOGE("%s: SCP plane vaddr NULL (y=%p vu=%p)", __FUNCTION__, srcY, srcVU);
        requeueScpBuffer(idx);
        return BAD_VALUE;
    }

    // Kernel-negotiated stride from VIDIOC_G_FMT (stored in the Node after
    // setFormat).  The stride comes from the gralloc buffer manager
    // (getBufStride → this+0xcf4); HAL3 gets it directly from the V4L2 node.
    // If G_FMT does not provide bytesperline (0), fall back to planeSize/height
    // — that is the old heuristic.
    unsigned int bplY  = m_node.getBytesPerLine(0);
    unsigned int bplVU = m_node.getBytesPerLine(1);
    size_t plane0Len = buf->planeSize(0);
    size_t plane1Len = buf->planeSize(1);
    int kernelStrideY  = (bplY  != 0) ? (int)bplY  : (ph > 0) ? (int)(plane0Len / (size_t)ph) : pw;
    int halfH = ph / 2;
    int kernelStrideVU = (bplVU != 0) ? (int)bplVU : (halfH > 0) ? (int)(plane1Len / (size_t)halfH) : pw;
    // Direct V4L2 path: NO memcpy.  copyPreviewToStream reads directly
    // from the V4L2-mapped buffer with the kernel stride.  This eliminates
    // the 3MB memcpy and the malloc allocation per frame.

    // V4L2 buffer is NOT requeued immediately — deferred until
    // releasePreviewFrame().  With 9 SCP buffers this is safe:
    // servePreviewRequest takes < 5ms, the pipeline has enough free buffers.
    // requeueScpBuffer(idx);  ← removed, see releasePreviewFrame()

    // 4. Pop frame from input queue (m_ispToScpQ).
    //    The frame already carries the ISP metadata (shot_ext).
    G800FFrame* frame = NULL;
    if (m_inputFrameQ) {
        m_inputFrameQ->try_pop_front(&frame);
    }

    if (!frame) {
        // Fallback: input queue empty (ISP not ready yet).
        // We have dequeued V4L2 — the pipeline will continue once the
        // frame is released.  Allocate a new frame.
        // The HAL3 layer may then query global latest values.
        frame = allocFrame(idx);
        if (!frame) {
            ALOGE("%s: SCP allocFrame fallback failed", __FUNCTION__);
            requeueScpBuffer(idx);
            return NO_MEMORY;
        }
        //ALOGD("%s: SCP idx=%d ← no ISP frame, using fallback", __FUNCTION__, idx);
    }

    // 5. Direct V4L2 path: frame references the SCP V4L2 buffer directly.
    //    NO malloc, NO memcpy — copyPreviewToStream reads from
    //    buf->planeVaddr(0/1) with kernel stride.
    //    V4L2 buffer is requeued in releasePreviewFrame().
    frame->buffer = buf;
    if (m_stream && idx < m_streamCount) {
        if (!frame->meta) {
            frame->meta = m_stream[idx];
            frame->metaSize = sizeof(camera2_stream);
        }
    }
    frame->previewData[0] = NULL;
    frame->previewData[1] = NULL;
    frame->previewDataSize[0] = ySize;
    frame->previewDataSize[1] = vuSize;
    frame->previewStride[0] = kernelStrideY;
    frame->previewStride[1] = kernelStrideVU;
    frame->previewWidth = pw;
    frame->previewHeight = ph;
    frame->ownsPreviewData = false;
    frame->scpBufferIndex = idx;
    frame->ownsScpBuffer = true;

    //ALOGD("%s: SCP dq idx=%d ← frame (fcount=%u, af=%d, ae=%d, awb=%d)",
    //      __FUNCTION__, idx, frame->frameCount,
    //      frame->afState, frame->aeState, frame->awbState);

    // Per-stage timestamp diagnostics: SCP output
    {
        //int64_t nowNs = systemTime(SYSTEM_TIME_MONOTONIC);
        //int64_t scpDeltaMs = (nowNs - (int64_t)frame->timestamp) / 1000000;
        //int ispToScpQSize = m_inputFrameQ ? m_inputFrameQ->size() : -1;
        //ALOGI("%s: SCP-TS idx=%d fcount=%u ts=%llu now=%llu delta=%lldms ispToScpQ=%d previewQ=%d/%d",
        //      __FUNCTION__, idx, frame->frameCount,
        //      (unsigned long long)frame->timestamp,
        //      (unsigned long long)nowNs, (long long)scpDeltaMs,
        //      ispToScpQSize, m_previewQ.size(), m_previewQ.getCapacity());
    }

    // Preview queue overflow protection
    if (m_previewQ.size() >= m_previewQ.getCapacity()) {
        G800FFrame* old = NULL;
        if (m_previewQ.try_pop_front(&old) == NO_ERROR && old) {
            //ALOGD("%s: SCP previewQ full, releasing oldest frame", __FUNCTION__);
            releasePreviewFrame(old);
        }
    }

    // 6. Push to preview queue for HAL3 layer
    m_previewQ.push_back(frame);
    //ALOGD("%s: SCP dq idx=%d → copied+requeued → previewQ (size=%d)",
    //      __FUNCTION__, idx, m_previewQ.size());

    return NO_ERROR;
}

status_t G800FScpPipe::m_putBuffer()
{
    // SCP is a capture node — buffers are requeued by releasePreviewFrame()
    return NO_ERROR;
}

// ============================================================================
// G800FSccPipe
// ============================================================================
G800FSccPipe::G800FSccPipe()
    : G800FPipe(PIPE_SCC, "G800FSccPipe"),
      m_stream(NULL), m_streamCount(0)
{
    memset(m_sccInUse, 0, sizeof(m_sccInUse));
    m_captureQ.setCapacity(4);
}

G800FSccPipe::~G800FSccPipe()
{
}

status_t G800FSccPipe::create(int videoNodeNum, int sensorId)
{
    m_videoNodeNum = videoNodeNum;
    m_sensorId = sensorId;
    status_t err = m_node.create(videoNodeNum);
    if (err != NO_ERROR) return err;
    err = m_node.open();
    if (err != NO_ERROR) return err;
    err = m_node.setInput(sensorId);
    ALOGI("%s: SCC node %d opened, sensorId=0x%x", __FUNCTION__, videoNodeNum, sensorId);
    return err;
}

status_t G800FSccPipe::setupPipe(int w, int h, int pixFmt,
                                 int numPlanes, int numBuffers,
                                 v4l2_buf_type bufType,
                                 v4l2_memory memory)
{
    status_t err = m_node.setSize(w, h);
    if (err != NO_ERROR) return err;
    err = m_node.setColorFormat(pixFmt, numPlanes);
    if (err != NO_ERROR) return err;
    err = m_node.setBufferType(numPlanes, bufType, memory);
    if (err != NO_ERROR) return err;
    err = m_node.setFormat();
    if (err != NO_ERROR) return err;
    err = m_node.reqBuffers(numBuffers);
    if (err != NO_ERROR) return err;

    // MMAP mode: kernel allocates buffers (has kvaddr/dvaddr for stream metadata).
    err = allocBuffersMmap(numBuffers, numPlanes);
    if (err != NO_ERROR) return err;

    m_numPlanes = numPlanes;
    ALOGI("%s: SCC configured %dx%d fmt=0x%x planes=%d bufs=%d",
          __FUNCTION__, w, h, pixFmt, numPlanes, numBuffers);
    return NO_ERROR;
}

status_t G800FSccPipe::queueInitialBuffers()
{
    for (int i = 0; i < m_numBuffers; i++) {
        G800FExynosCameraBuffer* buf = m_buffers[i];
        v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(planes, 0, sizeof(planes));
        for (int p = 0; p < m_numPlanes; p++) {
            planes[p].bytesused = buf->planeSize(p);
            planes[p].length    = buf->planeSize(p);
        }
        v4l2_buffer vbuf;
        memset(&vbuf, 0, sizeof(vbuf));
        vbuf.index  = i;
        vbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        vbuf.memory = V4L2_MEMORY_MMAP;
        vbuf.length = m_numPlanes;
        vbuf.m.planes = planes;
        status_t err = m_node.qBuf(&vbuf);
        if (err != NO_ERROR) return err;
    }
    ALOGI("%s: queued %d SCC buffers", __FUNCTION__, m_numBuffers);
    return NO_ERROR;
}

status_t G800FSccPipe::getCaptureFrame(G800FFrame** frame, int timeoutMs)
{
    return m_captureQ.pop_front(frame, timeoutMs);
}

void G800FSccPipe::releaseCaptureFrame(G800FFrame* frame)
{
    if (!frame) return;
    int idx = frame->bufferIndex;
    if (idx >= 0 && idx < m_numBuffers) {
        G800FExynosCameraBuffer* buf = m_buffers[idx];
        v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(planes, 0, sizeof(planes));
        for (int p = 0; p < m_numPlanes; p++) {
            planes[p].bytesused = buf->planeSize(p);
            planes[p].length    = buf->planeSize(p);
        }
        v4l2_buffer vbuf;
        memset(&vbuf, 0, sizeof(vbuf));
        vbuf.index  = idx;
        vbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        vbuf.memory = V4L2_MEMORY_MMAP;
        vbuf.length = m_numPlanes;
        vbuf.m.planes = planes;
        m_node.qBuf(&vbuf);
        Mutex::Autolock l(m_inUseLock);
        m_sccInUse[idx] = false;
    }
    recycleFrame(frame);
}

status_t G800FSccPipe::m_getBuffer()
{
    // Blocking dqbuf, no poll().
    v4l2_buffer vbuf;
    v4l2_plane planes[VIDEO_MAX_PLANES];
    memset(planes, 0, sizeof(planes));
    memset(&vbuf, 0, sizeof(vbuf));
    vbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    vbuf.memory = V4L2_MEMORY_MMAP;
    vbuf.length = m_numPlanes;
    vbuf.m.planes = planes;

    status_t err = m_node.dqBuf(&vbuf);
    if (err != NO_ERROR) {
        if (err != -EAGAIN) {
            ALOGE("%s: SCC dqBuf failed: %d", __FUNCTION__, err);
        }
        return err;
    }

    int idx = vbuf.index;
    if (idx < 0 || idx >= m_numBuffers) return BAD_VALUE;

    G800FFrame* frame = allocFrame(idx);
    if (!frame) return NO_MEMORY;
    frame->meta = (m_stream && idx < m_streamCount) ? m_stream[idx] : NULL;
    frame->metaSize = sizeof(camera2_stream);
    // The SCC output stream carries the FLITE fcount of its input frame in
    // camera2_stream.fcount — used to match the warm-path still to the
    // flash redirect target.
    frame->frameCount = frame->meta ? ((camera2_stream*)frame->meta)->fcount : 0;

    /* Bounded ring: each held frame pins one of the few SCC V4L2 buffers.
     * Keep only the most recent frames — the still target (firingStable+1)
     * is produced at the END of the lit window, so evicting the oldest keeps
     * the relevant frames while always leaving one buffer free for the pipe
     * to keep streaming.  On overflow, requeue the evicted frame's buffer. */
    while (m_captureQ.push_back(frame) != NO_ERROR) {
        G800FFrame* old = NULL;
        if (m_captureQ.try_pop_front(&old) != NO_ERROR || !old) {
            /* queue wedged — drop this frame's buffer back to the pool */
            releaseCaptureFrame(frame);
            return NO_ERROR;
        }
        releaseCaptureFrame(old);
    }
    return NO_ERROR;
}

status_t G800FSccPipe::m_putBuffer()
{
    // SCC is a capture node — buffers are requeued by releaseCaptureFrame()
    return NO_ERROR;
}

// ============================================================================
// G800FPipeEngine
// ============================================================================
G800FPipeEngine::G800FPipeEngine(int cameraId)
    : m_cameraId(cameraId), m_initialized(false), m_previewRunning(false),
      m_reprocessing(false), m_reprocNodesReady(false), m_reprocBuffersReady(false),
      m_reprocPipelineStreaming(false),
      m_reprocSetupInProgress(false),
      m_previewW(0), m_previewH(0),
      m_flitePipe(NULL), m_ispPipe(NULL), m_scpPipe(NULL), m_sccPipe(NULL),
      m_frameSelector(NULL),
      m_flashFrameSkipCount(0),
      m_flashSeqState(FLASH_SEQ_IDLE),
      m_flashSeqStateFrames(0),
      m_flashSeqCaptureFrames(0),
      m_flashSeqStateEnter(0),
      m_flashSeqBegin(0),
      m_flashReadyMain(false),
      m_flashState(AA_FLASHMODE_OFF),
      m_lastReqAeMode(AA_AEMODE_CENTER),
      m_lastReqAwbMode(AA_AWBMODE_WB_AUTO),
      m_lastReqAeFlashMode(AA_FLASHMODE_OFF),
      m_lastReqAfMode(ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE),
      m_lastReqIspAfMode(AA_AFMODE_CONTINUOUS_PICTURE),
      m_lastReqSceneMode(AA_SCENE_MODE_PREVIEW),
      m_lastReqAeExpComp(0),
      m_lastReqSensorExposure(0),
      m_lastReqSensorFrameDur(0),
      m_lastReqSensorSensitivity(0),
      m_lastReqFocusDistance(0),
      m_afKickPending(false),
      m_reprocThreadRunning(false)
{
    memset(m_fliteShotExt, 0, sizeof(m_fliteShotExt));
    memset(m_ispShotExt, 0, sizeof(m_ispShotExt));
    m_lastReqAeFps[0] = 15; m_lastReqAeFps[1] = 30;
    memset(m_lastReqAfRegions, 0, sizeof(m_lastReqAfRegions));
    m_lastReqAfRegions[4] = 1000;
    memset(m_scpStream, 0, sizeof(m_scpStream));
    memset(m_sccStream, 0, sizeof(m_sccStream));
    memset(m_reprocIspBuffers, 0, sizeof(m_reprocIspBuffers));
    memset(m_reprocSccBuffers, 0, sizeof(m_reprocSccBuffers));
    memset(m_reprocIspShotExt, 0, sizeof(m_reprocIspShotExt));
    memset(m_reprocSccStream, 0, sizeof(m_reprocSccStream));
}

G800FPipeEngine::~G800FPipeEngine()
{
    deinit();
}

status_t G800FPipeEngine::init()
{
    Mutex::Autolock l(s_instanceLock);
    if (s_activeInstance && s_activeInstance != this) {
        ALOGE("%s: another instance is already active", __FUNCTION__);
        return ALREADY_EXISTS;
    }
    s_activeInstance = this;

    // Create pipes
    m_flitePipe = new G800FFlitePipe();
    m_ispPipe   = new G800FIspPipe();
    m_scpPipe   = new G800FScpPipe();
    m_sccPipe   = new G800FSccPipe();

    // Create FrameSelector (Bayer frame selection)
    m_frameSelector = new G800FFrameSelector();

    // Create FLITE pipe
    int fliteNode = (m_cameraId == 0) ? NODE_FLITE_REAR : NODE_FLITE_FRONT;
    status_t err = m_flitePipe->create(fliteNode, packSensorIdFLITE(m_cameraId));
    if (err != NO_ERROR) goto fail;

    // Create ISP pipe
    err = m_ispPipe->create(NODE_ISP, packSensorIdISP(m_cameraId));
    if (err != NO_ERROR) goto fail;

    // Create SCP pipe
    err = m_scpPipe->create(NODE_SCP, packSensorIdSCP(m_cameraId));
    if (err != NO_ERROR) goto fail;

    // Create SCC pipe
    err = m_sccPipe->create(NODE_SCC, packSensorIdSCC(m_cameraId));
    if (err != NO_ERROR) goto fail;

    // Connect pipes via frame queues
    m_flitePipe->setOutputFrameQ(&m_fliteToIspQ);
    m_ispPipe->setInputFrameQ(&m_fliteToIspQ);
    m_ispPipe->setOutputFrameQ(&m_ispToScpQ); // ISP → SCP (preview)
    m_ispPipe->setFlitePipe(m_flitePipe.get());
    // SCP receives frames from ISP via m_ispToScpQ.
    // The frame flows as a single object through the pipeline.
    m_scpPipe->setInputFrameQ(&m_ispToScpQ);

    // IMPORTANT: limit FLITE→ISP queue to 2 (1 in-flight + 1 pending).
    // With the 1-in-flight ISP limit, ISP processes only 1 frame per
    // iteration.  If FLITE produces faster than ISP consumes, the
    // queue would consume all 8 FLITE buffers → FLITE buffer starvation →
    // "dqbuf can not be executed without qbuf(0)" → pipeline dead.
    // With capacity 2: FLITE discards excess frames (requeue),
    // ISP works at its own rate, FLITE buffers remain available.
    m_fliteToIspQ.setCapacity(2);

    // Shared frame pool — all pipes use the same pool.
    // The frame is allocated by FLITE, flows through ISP → SCP → HAL3
    // and is returned by releasePreviewFrame.
    m_flitePipe->setSharedFramePool(&m_sharedFramePool, &m_sharedFramePoolLock);
    m_ispPipe->setSharedFramePool(&m_sharedFramePool, &m_sharedFramePoolLock);
    m_scpPipe->setSharedFramePool(&m_sharedFramePool, &m_sharedFramePoolLock);

    // Connect FrameSelector to FLITE pipe
    m_flitePipe->setFrameSelector(m_frameSelector);

    // Note: SCC is fed via OTF from ISP, not via a frame queue.
    // The SCC pipe runs independently, dequeuing from the V4L2 node.

    m_initialized = true;
    ALOGI("%s: pipe engine initialized (camera %d)", __FUNCTION__, m_cameraId);
    return NO_ERROR;

fail:
    ALOGE("%s: init failed: %d", __FUNCTION__, err);
    deinit();
    return err;
}

void G800FPipeEngine::deinit()
{
    ALOGI("%s: called (previewRunning=%d, initialized=%d)", __FUNCTION__, (int)m_previewRunning.load(), (int)m_initialized.load());
    // CRITICAL (sensor protection): ALWAYS turn off flash, regardless of
    // m_reprocessing. If the flash sequence is active but reprocessing
    // has not started yet (e.g. error during beginFlashSequence),
    // the flash would otherwise stay on.
    if (m_flashSeqState != FLASH_SEQ_IDLE) {
        ALOGW("%s: Flash sequence still active (state=%s), forcing endFlashSequence()",
              __FUNCTION__, flashSeqStateName(m_flashSeqState));
        endFlashSequence();  // sets aeflashMode=OFF + releaseFlash() sysfs 'U'
    }

    if (m_reprocessing) finishStillCapture();

    // CRITICAL sensor protection: stop preview before destroying pipes.
    // stopPreview() does sensorStreamOff() (V4L2 CID 0x9a100e = 0) and
    // stop() on all pipes (VIDIOC_STREAMOFF + thread join).
    // This safely shuts down the image sensors (FLITE/ISP/SCC/SCP).
    if (m_previewRunning) stopPreview();

    // Destroy pipes: stop() + close() + releaseBuffers() for each V4L2 node.
    // Order: FLITE (sensor frontend) first, then ISP, then SCP/SCC.
    // sp<> releases the object when the last reference disappears.
    ALOGI("%s: destroying pipes", __FUNCTION__);
    if (m_flitePipe != NULL) { m_flitePipe->destroy(); m_flitePipe.clear(); }
    ALOGI("%s: FLITE pipe destroyed", __FUNCTION__);
    if (m_ispPipe != NULL)   { m_ispPipe->destroy();   m_ispPipe.clear(); }
    ALOGI("%s: ISP pipe destroyed", __FUNCTION__);
    if (m_scpPipe != NULL)   { m_scpPipe->destroy();   m_scpPipe.clear(); }
    ALOGI("%s: SCP pipe destroyed", __FUNCTION__);
    if (m_sccPipe != NULL)   { m_sccPipe->destroy();   m_sccPipe.clear(); }
    ALOGI("%s: SCC pipe destroyed", __FUNCTION__);

    if (m_frameSelector) { delete m_frameSelector; m_frameSelector = NULL; }

    Mutex::Autolock l(s_instanceLock);
    if (s_activeInstance == this) {
        s_activeInstance = NULL;
    }
    m_initialized = false;
}

status_t G800FPipeEngine::startPreview(int previewW, int previewH)
{
    if (!m_initialized) return NO_INIT;
    if (m_previewRunning) return ALREADY_EXISTS;

    m_previewW = previewW;
    m_previewH = previewH;

    // m_previewW/H is overwritten below with the BDS size (SCP buffer size).
    // The initial value here is only temporary and is replaced by the BDS block
    // further below. getPreviewWidth()/getPreviewHeight() then returns
    // the BDS size, which is used by copyPreviewToStream and JPEG fallback paths
    // as the SCP source size.

    // FLITE, ISP and SCC run CONTINUOUSLY at full-res.
    // Only SCP is configured to preview size.
    // No setSize on capture — FrameSelector selects FLITE Bayer asynchronously.
    //
    // Front camera (S5K6B2/S5K8B1, 2MP) has smaller sensor dimensions:
    //   maxSensor = 1920x1080, margin = 16x10 → FLITE/ISP = 1936x1090
    //   SCC = 1920x1080
    //   bdsPreviewEnabled = 0 (no BDS scaling)
    // Rear camera (S5K4H5, 8MP):
    //   maxSensor = 3264x2448, margin = 16x10 → FLITE/ISP = 3280x2458
    //   SCC = 3264x2448
    //   bdsPreviewEnabled = 1 (BDS scaling active)
    int fliteW, fliteH, ispW, ispH, sccW, sccH;
    if (m_cameraId == 0) {
        fliteW = FLITE_W; fliteH = FLITE_H;  // 3280x2458
        ispW   = ISP_W;   ispH   = ISP_H;    // 3280x2458
        sccW   = SCC_W;   sccH   = SCC_H;    // 3264x2448
    } else {
        fliteW = FLITE_FRONT_W; fliteH = FLITE_FRONT_H;  // 1936x1090
        ispW   = ISP_FRONT_W;   ispH   = ISP_FRONT_H;    // 1936x1090
        sccW   = SCC_FRONT_W;   sccH   = SCC_FRONT_H;    // 1920x1080
    }

    // Calculate BDS size (g_previewBdsTable)
    // For 960x720 (4:3) → BDS = 1440x1080
    // For 1280x720 (16:9) → BDS = 1920x1080
    // leader.output = min(bayerCrop, bds) — ISP scales to BDS size,
    // SCC/SCP scale further from there.
    // Declared before the goto statements, otherwise C++ gives jump-bypass errors.
    int bdsW = 0, bdsH = 0;
    // SCP is set to the requested preview size (e.g. 960x720), NV21M.
    // The firmware scales the ISP output to this size.
    //
    // HAL3 optimization: start SCP at BDS size instead of preview size.
    // BDS = ISP output size (e.g. 1920x1080 for 16:9). If later a
    // video stream with 1920x1080 is added, reconfigureScp is a no-op
    // (same size) and preview is not interrupted. Preview 1280x720
    // is software downscaled from 1920x1080 (copyPreviewToStreamFromData).
    // m_previewW/H is set below to the SCP buffer size (= BDS),
    // since getPreviewWidth()/getPreviewHeight() is used by copyPreviewToStream and
    // JPEG fallback paths as the SCP source size.
    int scpW = previewW, scpH = previewH;
    // Front camera: bdsPreviewEnabled = 0 (no BDS table).
    // ISP output = sensor size (1920x1080), no BDS scaling.
    // Rear camera: bdsPreviewEnabled = 1, BDS table is used.
    if (m_cameraId != 0) {
        bdsW = SCC_FRONT_W;  // 1920
        bdsH = SCC_FRONT_H;  // 1080
        ALOGI("%s: Front camera — BDS disabled, using sensor size %dx%d", __FUNCTION__, bdsW, bdsH);
    } else {
        getPreviewBdsSize(previewW, previewH, &bdsW, &bdsH);
        ALOGI("%s: BDS for preview %dx%d = %dx%d", __FUNCTION__,
              previewW, previewH, bdsW, bdsH);
    }

    // Configure SCP to BDS size (not to preview size).
    // m_previewW/H = SCP V4L2 buffer size (= BDS size), used by
    // getPreviewWidth()/getPreviewHeight() for copyPreviewToStream and
    // JPEG fallback paths. handleScpBuffer (G800FScpPipe) gets the
    // same size via setPreviewSize(scpW, scpH).
    scpW = bdsW;
    scpH = bdsH;
    m_previewW = scpW;
    m_previewH = scpH;

    ALOGI("%s: FLITE=%dx%d ISP=%dx%d SCC=%dx%d SCP=%dx%d (preview=%dx%d, continuous)",
          __FUNCTION__, fliteW, fliteH, ispW, ispH, sccW, sccH,
          scpW, scpH, previewW, previewH);

    // Setup FLITE pipe (full-res Bayer)
    // V4L2_MEMORY_DMABUF for all nodes.
    // DMABUF avoids kernel vmap exhaustion that occurs with USERPTR after ~4 frames
    // ("vmap allocation for size 12939264 failed").
    status_t err = m_flitePipe->setupPipe(fliteW, fliteH,
                                          V4L2_PIX_FMT_SBGGR12,
                                          FLITE_PLANES, NUM_FLITE_BUFFERS,
                                          V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                                          V4L2_MEMORY_DMABUF);
    ALOGI("%s: FLITE setupPipe err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;

    // Setup ISP pipe (full-res Bayer input)
    // V4L2_MEMORY_DMABUF.  ISP reads Bayer via VDMA1 from
    // plane 0 (FLITE dmabuf fd), shot_ext metadata from plane 1 (ISP dmabuf).
    err = m_ispPipe->setupPipe(ispW, ispH,
                               V4L2_PIX_FMT_SBGGR12,
                               ISP_PLANES, NUM_ISP_BUFFERS,
                               V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                               V4L2_MEMORY_DMABUF);
    ALOGI("%s: ISP setupPipe err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;

    // Setup SCP pipe (preview resolution)
    // SCP is configured to the requested preview size.
    // SCP setSize(960, 720) with NV21M is accepted and
    // output by the firmware. No more 1200x576 hardcoding.
    err = m_scpPipe->setupPipe(scpW, scpH,
                               V4L2_PIX_FMT_NV21M,
                               SCP_PLANES, NUM_SCP_BUFFERS,
                               V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                               V4L2_MEMORY_DMABUF);
    m_scpPipe->setPreviewSize(scpW, scpH);
    ALOGI("%s: SCP setupPipe err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;

    // Setup SCC pipe (full-res YUV capture — runs continuously)
    // SCC uses YUYV (packed, 2 planes: YUYV + SPARE), NOT NV21M!
    // (SCC m_format[0x56595559=YUYV] m_planes[2])
    err = m_sccPipe->setupPipe(sccW, sccH,
                               V4L2_PIX_FMT_YUYV,
                               SCC_PLANES, NUM_SCC_BUFFERS,
                               V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
    ALOGI("%s: SCC setupPipe err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;

    // Initialize shot_ext metadata for FLITE and ISP
    // BDS size was calculated above (bdsW, bdsH).

    for (int i = 0; i < NUM_FLITE_BUFFERS; i++) {
        m_fliteShotExt[i] = reinterpret_cast<camera2_shot_ext*>(
            m_flitePipe->getBuffer(i)->planeVaddr(1));
        memset(m_fliteShotExt[i], 0, sizeof(camera2_shot_ext));
        m_fliteShotExt[i]->shot.magicNumber = SHOT_MAGIC_NUMBER;
        m_fliteShotExt[i]->setfile = 0; // preview setfile
        m_fliteShotExt[i]->drc_bypass = 1;
        m_fliteShotExt[i]->dis_bypass = 1;
        m_fliteShotExt[i]->dnr_bypass = 1;
        m_fliteShotExt[i]->fd_bypass = 1;

        // CRITICAL: Initialize aa.* enum fields to valid values.
        // memset(0) leaves them at 0, but FIMC-IS enums are 1-based
        // (AA_FLASHMODE_OFF=1, AA_AEMODE_OFF=1, etc.).  Value 0 is
        // undefined and may cause the firmware to misinterpret the
        // shot_ext — e.g. aeflashMode=0 triggers an internal "auto"
        // flash decision after ~2s, which fires FlashCommand 7 (main
        // flash) without a preceding pre-flash, causing the firmware
        // error "Preflash does not called before!!" and crashing the
        // FLITE pipeline (POLLERR → preview never starts).
        m_fliteShotExt[i]->shot.ctl.aa.aeflashMode = AA_FLASHMODE_OFF;
        // Session default: metering-mode=center -> AA_AEMODE_CENTER
        m_fliteShotExt[i]->shot.ctl.aa.aeMode      = AA_AEMODE_CENTER;
        m_fliteShotExt[i]->shot.ctl.aa.awbMode     = AA_AWBMODE_WB_AUTO;
        m_fliteShotExt[i]->shot.ctl.aa.captureIntent = AA_CAPTURE_INTENT_PREVIEW;
        m_fliteShotExt[i]->shot.ctl.flash.flashMode = CAM2_FLASH_MODE_OFF;
        // aeExpCompensation = ev + 5, ev=0 → 5.
        m_fliteShotExt[i]->shot.ctl.aa.aeExpCompensation = 5;

        applyDefaultCtlValues(&m_fliteShotExt[i]->shot.ctl);

        // Auto-focus mode: rear camera (ID 0) supports continuous-picture AF,
        // front camera (ID 1) is fixed-focus.
        // The FIMC-IS firmware reads shot.ctl.aa.afMode to control the AF
        // actuator.  afTrigger=1 triggers AF start.  Without afTrigger=1,
        // the firmware may
        // not activate the AF actuator.
        if (m_cameraId == 0) {
            m_fliteShotExt[i]->shot.ctl.aa.afMode = AA_AFMODE_CONTINUOUS_PICTURE;
            m_fliteShotExt[i]->shot.ctl.aa.afTrigger = 1;
            m_fliteShotExt[i]->shot.ctl.aa.afRegions[0] = 0;
            m_fliteShotExt[i]->shot.ctl.aa.afRegions[1] = 0;
            m_fliteShotExt[i]->shot.ctl.aa.afRegions[2] = 0;
            m_fliteShotExt[i]->shot.ctl.aa.afRegions[3] = 0;
            m_fliteShotExt[i]->shot.ctl.aa.afRegions[4] = 1000;
        } else {
            m_fliteShotExt[i]->shot.ctl.aa.afMode = AA_AFMODE_OFF;
        }
        // Session-start AF kick: make the FIRST applyRequestSettings after
        // this startPreview emit an afTrigger=1 pulse regardless of whether
        // the requested mode differs from the cache.
        // Fixes: after session 2+ the firmware never scanned because the
        // buffer-init trigger was overwritten before the FW consumed it.
        m_afKickPending = (m_cameraId == 0);

        // Node group: leader=ISP, capture[0]=SCC, capture[1]=SCP
        // (from the kernel node-group contract + BDS table):
        //   bayerCrop = getPreviewBayerCropSize(dstRect) = {0, 0, 3264, 2448}
        //     (from BDS table entry[5,6] = zoomRefW×zoomRefH, at zoom=0)
        //   bds = getPreviewBdsSize() = {0, 0, 1440, 1080}
        //     (from BDS table entry[7,8] = previewBdsW×previewBdsH)
        //   leader.input.crop  = bayerCrop = 3264x2448 (NOT FLITE 3280x2458!)
        //   leader.output.crop = min(bayerCrop.w, bds.w) x min(bayerCrop.h, bds.h)
        //     = min(3264, 1440) x min(2448, 1080) = 1440x1080
        //   capture[0]=SCC: input=leader.output, output=pictureW×pictureH
        //   capture[1]=SCP: input=leader.output, output=previewW×previewH
        //
        // CRITICAL: leader.input must be 3264x2448 (SCC/Bayer crop),
        //   NOT 3280x2458 (FLITE with padding).  The FLITE size
        //   includes sensor padding (16px wider, 10px taller) that
        //   is not part of the active image area.  We use
        //   bayerCrop from the BDS table, which corresponds to the SCC size.
        int bayerCropW = sccW;  // 3264 — from BDS table entry[5,6] at zoom=0
        int bayerCropH = sccH;  // 2448
        int leaderOutW = (bayerCropW < bdsW) ? bayerCropW : bdsW;
        int leaderOutH = (bayerCropH < bdsH) ? bayerCropH : bdsH;
        setNodeGroup(m_fliteShotExt[i], -1, 30/*ISP*/, 1,
                     0, 0, bayerCropW, bayerCropH,  /* leader.input  = bayerCrop (3264x2448) */
                     0, 0, leaderOutW, leaderOutH); /* leader.output = min(bayerCrop, bds) */
        // capture[0] = SCC (picture)
        // Node-group layout:
        //   In normal preview mode, capture[0].request == 0!
        //   The ISP hardware does NOT output SCC frames (3264x2448 YUYV = 15.2 MB
        //   per frame).  This is the main difference from our previous
        //   implementation, which set request=1 and thus forced the ISP
        //   hardware to write an additional 15.2 MB per frame.
        //   At 30fps that would be 456 MB/s in addition to the BDS/SCP load — that
        //   exceeds the Exynos 3470 memory bandwidth and explains the 1fps.
        //
        //   SCC is only enabled per-frame (request=1) when a picture
        //   capture is pending.  The reprocessing node-group marks
        //   writing frame->field_0x54 into capture[0].request.
        //
        //   The SCC thread simply blocks in dqbuf until a frame arrives.
        //   That is OK — it does not consume CPU.
        setNodeGroup(m_fliteShotExt[i], 0, 34/*SCC*/, 0,  // request=0!
                     0, 0, leaderOutW, leaderOutH,  /* capture[0].input  = leader.output */
                     0, 0, sccW, sccH);             /* capture[0].output = picture        */
        // capture[1] = SCP (preview)
        // Output crop = firmware's actual SCP output size (1200x576), not the
        // app's requested preview size. The firmware ignores our output crop
        // for the DMA dimensions but uses it for scaler configuration.
        setNodeGroup(m_fliteShotExt[i], 1, 37/*SCP*/, 1,
                     0, 0, leaderOutW, leaderOutH,      /* capture[1].input  = leader.output */
                     0, 0, scpW, scpH);                /* capture[1].output = SCP firmware size */
    }
    m_flitePipe->setShotExt(m_fliteShotExt, NUM_FLITE_BUFFERS);

    // Parameter control block: same defaults as the per-buffer shots.
    // applyRequestSettings() keeps it updated with the request-derived
    // values; the flash state machine never touches it.  Source for the
    // reprocessing shot's ctl.
    memset(&m_paramsCtl, 0, sizeof(m_paramsCtl));
    m_paramsCtl.aa.aeflashMode = AA_FLASHMODE_OFF;
    m_paramsCtl.aa.aeMode      = AA_AEMODE_CENTER;   // session default metering=center
    m_paramsCtl.aa.awbMode     = AA_AWBMODE_WB_AUTO;
    m_paramsCtl.aa.captureIntent = AA_CAPTURE_INTENT_PREVIEW;
    m_paramsCtl.flash.flashMode = CAM2_FLASH_MODE_OFF;
    m_paramsCtl.aa.aeExpCompensation = 5;
    applyDefaultCtlValues(&m_paramsCtl);
    if (m_cameraId == 0) {
        m_paramsCtl.aa.afMode = AA_AFMODE_CONTINUOUS_PICTURE;
        m_paramsCtl.aa.afTrigger = 1;
        m_paramsCtl.aa.afRegions[4] = 1000;
    } else {
        m_paramsCtl.aa.afMode = AA_AFMODE_OFF;
    }

    for (int i = 0; i < NUM_ISP_BUFFERS; i++) {
        m_ispShotExt[i] = reinterpret_cast<camera2_shot_ext*>(
            m_ispPipe->getBuffer(i)->planeVaddr(1));
        memset(m_ispShotExt[i], 0, sizeof(camera2_shot_ext));
        m_ispShotExt[i]->shot.magicNumber = SHOT_MAGIC_NUMBER;
        m_ispShotExt[i]->setfile = 0;
        m_ispShotExt[i]->drc_bypass = 1;
        m_ispShotExt[i]->dis_bypass = 1;
        m_ispShotExt[i]->dnr_bypass = 1;
        m_ispShotExt[i]->fd_bypass = 1;

        // CRITICAL: Initialize aa.* enum fields to valid values (1-based).
        // See FLITE shot_ext comment above for details.
        m_ispShotExt[i]->shot.ctl.aa.aeflashMode = AA_FLASHMODE_OFF;
        m_ispShotExt[i]->shot.ctl.aa.aeMode      = AA_AEMODE_CENTER;
        m_ispShotExt[i]->shot.ctl.aa.awbMode     = AA_AWBMODE_WB_AUTO;
        m_ispShotExt[i]->shot.ctl.aa.captureIntent = AA_CAPTURE_INTENT_PREVIEW;
        m_ispShotExt[i]->shot.ctl.flash.flashMode = CAM2_FLASH_MODE_OFF;
        // aeExpCompensation = ev + 5, ev=0 → 5.
        m_ispShotExt[i]->shot.ctl.aa.aeExpCompensation = 5;

        applyDefaultCtlValues(&m_ispShotExt[i]->shot.ctl);

        // Auto-focus mode (same as FLITE shot_ext above).
        // The ISP firmware reads ctl.aa.afMode from the ISP shot_ext
        // to control the AF actuator on the rear camera sensor.
        if (m_cameraId == 0) {
            m_ispShotExt[i]->shot.ctl.aa.afMode = AA_AFMODE_CONTINUOUS_PICTURE;
            m_ispShotExt[i]->shot.ctl.aa.afTrigger = 1;
            m_ispShotExt[i]->shot.ctl.aa.afRegions[0] = 0;
            m_ispShotExt[i]->shot.ctl.aa.afRegions[1] = 0;
            m_ispShotExt[i]->shot.ctl.aa.afRegions[2] = 0;
            m_ispShotExt[i]->shot.ctl.aa.afRegions[3] = 0;
            m_ispShotExt[i]->shot.ctl.aa.afRegions[4] = 1000;
        } else {
            m_ispShotExt[i]->shot.ctl.aa.afMode = AA_AFMODE_OFF;
        }
    }
    m_ispPipe->setShotExt(m_ispShotExt, NUM_ISP_BUFFERS);

    // Initialize camera2_stream metadata for SCP/SCC
    for (int i = 0; i < NUM_SCP_BUFFERS; i++) {
        m_scpStream[i] = reinterpret_cast<camera2_stream*>(
            m_scpPipe->getBuffer(i)->planeVaddr(2));
        memset(m_scpStream[i], 0, sizeof(camera2_stream));
    }
    m_scpPipe->setStreamMeta(m_scpStream, NUM_SCP_BUFFERS);

    for (int i = 0; i < NUM_SCC_BUFFERS; i++) {
        // SCC: YUYV = 2 planes (YUYV data + SPARE). SPARE = plane 1.
        m_sccStream[i] = reinterpret_cast<camera2_stream*>(
            m_sccPipe->getBuffer(i)->planeVaddr(1));
        memset(m_sccStream[i], 0, sizeof(camera2_stream));
    }
    m_sccPipe->setStreamMeta(m_sccStream, NUM_SCC_BUFFERS);

    // Queue initial buffers:
    // preparePipes does ISP->prepare() (qbuf ISP) and SCC->prepare() (qbuf SCC).
    // FLITE buffers are pushed in pushFrameToPipe (here: queueInitialBuffers).
    // SCP also gets initial buffers.
    err = m_flitePipe->queueInitialBuffers();
    ALOGI("%s: FLITE queueInitialBuffers err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;
    err = m_ispPipe->queueInitialBuffers();  // shot_ext metadata for firmware
    ALOGI("%s: ISP queueInitialBuffers err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;
    err = m_scpPipe->queueInitialBuffers();
    ALOGI("%s: SCP queueInitialBuffers err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;
    err = m_sccPipe->queueInitialBuffers();
    ALOGI("%s: SCC queueInitialBuffers err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;

    // Create reprocessing nodes (ISP[1] + SCC[1]) AND buffers in
    // a separate thread:
    // The thread runs async and does the picture setup in parallel with
    // preview start: nodes + buffers async at startPreview,
    // no join before STREAMON. The thread is joined at requestStillCapture()
    // (if still running) or at stopPreview().
    //
    // Advantage: preview start is not blocked by node setup.
    // The nodes are ready when requestStillCapture() comes (or shortly after).
    if (!m_reprocNodesReady && !m_reprocBuffersReady && !m_reprocSetupInProgress) {
        // Join previous thread if not already done (should not be necessary,
        // but defensive against duplicate startPreview calls).
        if (m_reprocSetupThread.joinable()) {
            m_reprocSetupThread.join();
        }
        m_reprocSetupInProgress = true;
        m_reprocSetupThread = std::thread([this]() {
            int reprocIspW, reprocIspH, reprocSccW, reprocSccH;
            if (m_cameraId == 0) {
                reprocIspW = FLITE_W; reprocIspH = FLITE_H;  // 3280x2458
                reprocSccW = SCC_W;   reprocSccH = SCC_H;    // 3264x2448
            } else {
                reprocIspW = FLITE_FRONT_W; reprocIspH = FLITE_FRONT_H;  // 1936x1090
                reprocSccW = SCC_FRONT_W;   reprocSccH = SCC_FRONT_H;   // 1920x1080
            }
            // 1. Open and configure nodes
            status_t reprocErr = setupReprocessingNodes(reprocIspW, reprocIspH,
                                                         reprocSccW, reprocSccH);
            if (reprocErr != NO_ERROR) {
                ALOGE("%s: async setupReprocessingNodes failed: %d (non-fatal)",
                      __FUNCTION__, reprocErr);
                m_reprocSetupInProgress = false;
                return;
            }
            m_reprocNodesReady = true;
            ALOGI("%s: reprocessing nodes pre-configured (async)",
                  __FUNCTION__);
            // 2. Allocate buffers
            reprocErr = allocReprocessingBuffers(reprocIspW, reprocIspH,
                                                  reprocSccW, reprocSccH);
            if (reprocErr != NO_ERROR) {
                ALOGE("%s: async allocReprocessingBuffers failed: %d (non-fatal)",
                      __FUNCTION__, reprocErr);
                m_reprocSetupInProgress = false;
                return;
            }
            m_reprocBuffersReady = true;
            ALOGI("%s: reprocessing buffers pre-allocated (async)",
                  __FUNCTION__);
            m_reprocSetupInProgress = false;
        });
        ALOGI("%s: reprocessing node+buffer setup started (async, no join before STREAMON)",
              __FUNCTION__);
    } else if (m_reprocSetupThread.joinable()) {
        // Thread still running from previous startPreview - do not join.
        ALOGI("%s: reproc setup thread still running from previous startPreview (not joining)",
              __FUNCTION__);
    }

    // Start streaming order:
    //   1. FLITE->start()  (STREAMON only, no thread)
    //   2. SCP->start()    (STREAMON only)
    //   3. SCC->start()    (STREAMON only)
    //   4. ISP->start()    (STREAMON only)
    //   5. FLITE->sensorStream(true)  (V4L2_CID_IS_S_STREAM = 1, blocking)
    //
    // Then:
    //   6. FLITE->startThread()
    //   7. ISP->startThread()
    //
    // The separation of STREAMON and thread start is important:
    // All nodes must be streaming before the sensor sends stream-on
    // to the firmware. Threads may only run when the pipeline is
    // fully configured and streaming.

    // FLITE setControl calls before STREAMON:
    //   setControl(V4L2_CID_IS_MIN_TARGET_FPS=0x9a1037, minFps=15)
    //   setControl(V4L2_CID_IS_MAX_TARGET_FPS=0x9a1038, maxFps=30)
    //   setControl(V4L2_CID_IS_SCENE_MODE=0x9a129b, sceneMode=IS_SCENE_MODE_NONE=1)
    // The values come from the camera2_shot metadata buffer:
    //   [shot+0x570] = min target fps, [shot+0x574] = max target fps,
    //   [shot+0x54c] = aa.sceneMode
    err = m_flitePipe->getNode()->setControl(0x9a1037, 15);   // V4L2_CID_IS_MIN_TARGET_FPS
    if (err != NO_ERROR) {
        ALOGW("%s: FLITE setControl(MIN_TARGET_FPS=15) failed: %d (continuing)", __FUNCTION__, err);
    }
    err = m_flitePipe->getNode()->setControl(0x9a1038, 30);   // V4L2_CID_IS_MAX_TARGET_FPS
    if (err != NO_ERROR) {
        ALOGW("%s: FLITE setControl(MAX_TARGET_FPS=30) failed: %d (continuing)", __FUNCTION__, err);
    }
    err = m_flitePipe->getNode()->setControl(0x9a129b, 1);    // V4L2_CID_IS_SCENE_MODE = IS_SCENE_MODE_NONE
    if (err != NO_ERROR) {
        ALOGW("%s: FLITE setControl(SCENE_MODE=NONE) failed: %d (continuing)", __FUNCTION__, err);
    }

    err = m_flitePipe->start();   // 1. FLITE STREAMON
    ALOGI("%s: FLITE start() err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;
    err = m_scpPipe->start();     // 2. SCP STREAMON
    ALOGI("%s: SCP start() err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;
    err = m_sccPipe->start();     // 3. SCC STREAMON
    ALOGI("%s: SCC start() err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;
    err = m_ispPipe->start();     // 4. ISP STREAMON
    ALOGI("%s: ISP start() err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;

    // 5. Sensor stream-on (blocking mode, value 1)
    err = m_flitePipe->sensorStreamOn();
    ALOGI("%s: sensorStreamOn err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) {
        ALOGE("%s: sensorStreamOn failed: %d", __FUNCTION__, err);
        goto fail;
    }

    // 6-8. Start worker threads (after sensorStreamOn)
    ALOGI("%s: starting worker threads (FLITE→SCP→SCC→ISP)", __FUNCTION__);
    err = m_flitePipe->startThread();
    ALOGI("%s: FLITE startThread err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;
    err = m_scpPipe->startThread();
    ALOGI("%s: SCP startThread err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;
    err = m_sccPipe->startThread();
    ALOGI("%s: SCC startThread err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;
    err = m_ispPipe->startThread();
    ALOGI("%s: ISP startThread err=%d", __FUNCTION__, err);
    if (err != NO_ERROR) goto fail;

    // setFrameSkipCount(8): frameSkip=8 lets AE/AWB converge
    // before frames are delivered.
    if (m_frameSelector) {
        m_frameSelector->reset();
        m_frameSelector->setFrameSkipCount(8);
        ALOGI("%s: setFrameSkipCount(8) for AE/AWB convergence", __FUNCTION__);
    }

    m_previewRunning = true;
    {
        int curFliteW = (m_cameraId == 0) ? FLITE_W : FLITE_FRONT_W;
        int curFliteH = (m_cameraId == 0) ? FLITE_H : FLITE_FRONT_H;
        ALOGI("%s: preview pipeline started (FLITE=%dx%d continuous)",
              __FUNCTION__, curFliteW, curFliteH);
    }
    return NO_ERROR;

fail:
    ALOGE("%s: startPreview failed: %d", __FUNCTION__, err);
    // Close reprocessing nodes if they were already created.
    // stopPreview() returns early if m_previewRunning==false, so
    // we must close the reproc nodes explicitly here.
    if (m_reprocNodesReady || m_reprocIspNode.isOpen() || m_reprocSccNode.isOpen()) {
        closeReprocessingNodes();
    }
    stopPreview();
    return err;
}

status_t G800FPipeEngine::stopPreview()
{
    ALOGI("%s: called (previewRunning=%d)", __FUNCTION__, (int)m_previewRunning.load());
    if (!m_previewRunning) return NO_ERROR;
    m_previewRunning = false;

    // CRITICAL sensor protection: turn off sensor stream FIRST (before stopping pipes).
    // sensorStreamOff() sends V4L2 CID 0x9a100e = 0 to FLITE, which
    // puts the sensor into idle mode. Errors are logged but we
    // continue anyway — the pipes must be stopped regardless.
    if (m_flitePipe) {
        status_t sErr = m_flitePipe->sensorStreamOff();
        if (sErr != NO_ERROR) {
            ALOGE("%s: sensorStreamOff failed: %d — continuing pipe stop",
                  __FUNCTION__, sErr);
        }
    }

    // Stop pipes (upstream first, then downstream).
    // Each stop() does: signalStop → STREAMOFF → thread join.
    // (signalStop BEFORE STREAMOFF prevents busy-loop after stream stop)
    ALOGI("%s: stopping pipes (FLITE→ISP→SCC→SCP)", __FUNCTION__);
    if (m_flitePipe) m_flitePipe->stop();
    ALOGI("%s: FLITE stopped, now ISP", __FUNCTION__);
    if (m_ispPipe)   m_ispPipe->stop();
    ALOGI("%s: ISP stopped, now SCC", __FUNCTION__);
    if (m_sccPipe)   m_sccPipe->stop();
    ALOGI("%s: SCC stopped, now SCP", __FUNCTION__);
    if (m_scpPipe)   m_scpPipe->stop();
    ALOGI("%s: SCP stopped, all pipes done", __FUNCTION__);

    // Release V4L2 buffers (REQBUFS(0)) on all pipes after STREAMOFF.
    // Without this, the next startPreview() reqbufs(N) fails with EBUSY
    // because the kernel still has N buffers allocated from this session.
    ALOGI("%s: releasing V4L2 buffers (REQBUFS(0))", __FUNCTION__);
    if (m_flitePipe) m_flitePipe->releaseBuffers();
    if (m_ispPipe)   m_ispPipe->releaseBuffers();
    if (m_sccPipe)   m_sccPipe->releaseBuffers();
    if (m_scpPipe)   m_scpPipe->releaseBuffers();
    ALOGI("%s: V4L2 buffers released", __FUNCTION__);

    // Invalidate shot_ext pointers — they point into pipe buffer memory
    // that is freed when the pipes are destroyed.
    memset(m_fliteShotExt, 0, sizeof(m_fliteShotExt));
    memset(m_ispShotExt, 0, sizeof(m_ispShotExt));

    // Recycle frames still stuck in m_ispFrameMap (ISP was
    // stopped before the buffer was processed).  Otherwise they
    // would be missing from the shared pool and the pipeline would not
    // be able to allocate frames on the next start.
    if (m_ispPipe) {
        m_ispPipe->recycleStuckFrames();
    }

    // Join the async setup thread (if not yet finished).
    // The thread was started in startPreview() and does node setup +
    // buffer allocation in parallel. At stopPreview it must be finished
    // before we free the nodes/buffers.
    if (m_reprocSetupThread.joinable()) {
        ALOGI("%s: joining reproc setup thread before cleanup", __FUNCTION__);
        m_reprocSetupThread.join();
    }

    // Stop reprocessing pipeline (STREAMOFF) and join thread.
    // Must happen BEFORE closeReprocessingNodes() (STREAMOFF before close).
    if (m_reprocessing) finishStillCapture();
    if (m_reprocPipelineStreaming) {
        stopReprocessingPipeline();
        m_reprocPipelineStreaming = false;
    }

    // Release reprocessing buffers.
    // The buffers were allocated async in startPreview(). They must be
    // released at preview stop so they can be re-allocated on the next
    // startPreview().
    if (m_reprocBuffersReady) {
        releaseReprocessingBuffers();
        m_reprocBuffersReady = false;
    }

    // Close reprocessing nodes.
    // The nodes were opened and configured async in startPreview().
    // They must be closed at preview stop so that s_fmt works again
    // on the next startPreview().
    if (m_reprocNodesReady || m_reprocIspNode.isOpen() || m_reprocSccNode.isOpen()) {
        closeReprocessingNodes();
        m_reprocNodesReady = false;
    }

    ALOGI("%s: preview pipeline stopped", __FUNCTION__);
    return NO_ERROR;
}

// ============================================================================
// reconfigureScp — SCP-only reconfiguration on resolution change
//
// Instead of stopping and restarting the entire pipeline (FLITE→ISP→SCC→SCP),
// only SCP is stopped and reconfigured.  FLITE, ISP and SCC
// continue running at full-res undisturbed.
//
// Steps:
//   1. Stop SCP thread + STREAMOFF (m_scpPipe->stop())
//   2. Release SCP kernel buffers (releaseBuffers on node)
//   3. Release SCP userspace buffers (releaseBuffers on pipe)
//   4. Reconfigure SCP: setSize → setColorFormat → setBufferType →
//      setFormat → reqBuffers → allocBuffersDmabuf
//   5. Reset m_scpStream[] metadata pointers (point into SCP buffer plane 2)
//   6. Update FLITE shot_ext capture[1].output.crop to new SCP size
//   7. Queue SCP initial buffers
//   8. SCP STREAMON + start thread
//
// Fallback: on error → stopPreview() + startPreview() as full restart.
// ============================================================================

status_t G800FPipeEngine::reconfigureScp(int newW, int newH)
{
    ALOGI("%s: reconfiguring SCP to %dx%d (current SCP=%dx%d, FLITE/ISP/SCC stay running)",
          __FUNCTION__, newW, newH, m_previewW, m_previewH);

    if (!m_previewRunning || !m_scpPipe) {
        ALOGE("%s: preview not running — cannot reconfigure", __FUNCTION__);
        return NO_INIT;
    }
    // Same-size check: newW/newH and m_previewW/H are both BDS sizes.
    // If equal, SCP is already correctly configured — no-op.
    if (newW == m_previewW && newH == m_previewH) {
        ALOGI("%s: same BDS size %dx%d — nothing to do", __FUNCTION__, newW, newH);
        return NO_ERROR;
    }

    // Reprocessing must not be running — SCP reconfigure during reprocessing
    // would confuse the reproc thread (it gets frames from ISP/FLITE, not
    // from SCP, but the buffer release could cause race conditions).
    if (m_reprocessing) {
        ALOGW("%s: reprocessing active — falling back to full stopPreview+startPreview",
              __FUNCTION__);
        stopPreview();
        return startPreview(newW, newH);
    }

    // BDS check BEFORE all V4L2 operations.  If the aspect ratio
    // changes (e.g. 4:3 → 16:9), the BDS size changes, which requires
    // an ISP reconfiguration.  In this case, do a full restart directly,
    // without trying the SCP-only reconfigure.  This avoids the
    // reqbufs(0)-after-STREAMOFF problem that occurs on BDS changes
    // (kernel keeps STREAM_ON bit → reqbufs(0) → EINVAL → corruption).
    int oldBdsW, oldBdsH, newBdsW, newBdsH;
    getPreviewBdsSize(m_previewW, m_previewH, &oldBdsW, &oldBdsH);
    getPreviewBdsSize(newW, newH, &newBdsW, &newBdsH);
    if (oldBdsW != newBdsW || oldBdsH != newBdsH) {
        ALOGI("%s: BDS changed (%dx%d → %dx%d) — full deinit+init required "
              "(close/reopen V4L2 nodes)",
              __FUNCTION__, oldBdsW, oldBdsH, newBdsW, newBdsH);
        // stop() closes the V4L2 node (m_node->close()),
        // initPipes() reopens it (m_node->open()).  Without close/reopen,
        // the FIMC-IS kernel retains an inconsistent ischain/subdev state,
        // which leads to a kernel hang during qbuf on the next startPreview.
        // deinit()+init() between resolution changes avoids that.
        deinit();
        status_t err = init();
        if (err != NO_ERROR) {
            ALOGE("%s: init() failed after deinit: %d", __FUNCTION__, err);
            return err;
        }
        return startPreview(newW, newH);
    }

    // 1. Stop SCP thread + STREAMOFF
    ALOGI("%s: stopping SCP pipe (thread + STREAMOFF)", __FUNCTION__);
    m_scpPipe->stop();

    // 2. Release SCP kernel buffers (REQBUFS(0))
    G800FExynosCameraNode* scpNode = m_scpPipe->getNode();
    if (scpNode == NULL) {
        ALOGE("%s: SCP node is NULL — falling back to full restart", __FUNCTION__);
        stopPreview();
        return startPreview(newW, newH);
    }
    scpNode->releaseBuffers();

    // 3. Release SCP userspace buffers
    m_scpPipe->releaseBuffers();

    // 4. Reconfigure SCP (same parameters as startPreview, new size)
    //    As in startPreview, SCP is configured to BDS size, not to
    //    the requested preview size. newW/newH is the framework size
    //    (e.g. 1280x720), but SCP gets BDS (e.g. 1920x1080).
    //    The BDS check above already confirmed that BDS has not changed
    //    (otherwise we would have gone the full-restart path), so
    //    newBdsW/newBdsH == oldBdsW/oldBdsH.
    int scpW = newBdsW, scpH = newBdsH;
    status_t err = m_scpPipe->setupPipe(scpW, scpH,
                                        V4L2_PIX_FMT_NV21M,
                                        SCP_PLANES, NUM_SCP_BUFFERS,
                                        V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                                        V4L2_MEMORY_DMABUF);
    if (err != NO_ERROR) {
        ALOGE("%s: SCP setupPipe(%dx%d) failed: %d — falling back to deinit+init",
              __FUNCTION__, scpW, scpH, err);
        // deinit()+init()+startPreview() instead of just
        // stopPreview()+startPreview().  If reconfigureScp fails
        // (e.g. reqbufs(0) EINVAL because buffers are still in flight), the
        // FIMC-IS subsystem state is inconsistent.  A simple stop+start
        // would then hang at SCC MMAP alloc (kernel hang).  deinit() closes
        // the V4L2 nodes completely, which resets the kernel state.
        deinit();
        status_t initErr = init();
        if (initErr != NO_ERROR) {
            ALOGE("%s: init() failed after deinit: %d", __FUNCTION__, initErr);
            return initErr;
        }
        return startPreview(newW, newH);
    }
    m_scpPipe->setPreviewSize(scpW, scpH);

    // 5. Reset m_scpStream[] metadata pointers (point into plane 2 of the
    //    newly allocated SCP buffers)
    for (int i = 0; i < NUM_SCP_BUFFERS; i++) {
        m_scpStream[i] = reinterpret_cast<camera2_stream*>(
            m_scpPipe->getBuffer(i)->planeVaddr(2));
        memset(m_scpStream[i], 0, sizeof(camera2_stream));
    }
    m_scpPipe->setStreamMeta(m_scpStream, NUM_SCP_BUFFERS);

    // 6. Update FLITE shot_ext capture[1].output.crop to new SCP size.
    //    The ISP firmware reads this crop region to configure the SCP scaler.
    //    Without an update, SCP would still scale to the old size.
    //
    //    BDS check was already performed at the beginning of reconfigureScp().
    //    If we get here, BDS is unchanged → leader.output stays the same.
    int curSccW = (m_cameraId == 0) ? SCC_W : SCC_FRONT_W;
    int curSccH = (m_cameraId == 0) ? SCC_H : SCC_FRONT_H;
    int leaderOutW = (curSccW < oldBdsW) ? curSccW : oldBdsW;
    int leaderOutH = (curSccH < oldBdsH) ? curSccH : oldBdsH;
    for (int i = 0; i < NUM_FLITE_BUFFERS; i++) {
        if (m_fliteShotExt[i]) {
            setNodeGroup(m_fliteShotExt[i], 1, 37/*SCP*/, 1,
                         0, 0, leaderOutW, leaderOutH,      /* input = leader.output (unchanged) */
                         0, 0, scpW, scpH);                 /* output = new SCP size */
        }
    }

    // 7. Queue SCP initial buffers
    err = m_scpPipe->queueInitialBuffers();
    if (err != NO_ERROR) {
        ALOGE("%s: SCP queueInitialBuffers failed: %d — falling back to full restart",
              __FUNCTION__, err);
        stopPreview();
        return startPreview(newW, newH);
    }

    // 8. SCP STREAMON + start thread
    err = m_scpPipe->start();
    if (err != NO_ERROR) {
        ALOGE("%s: SCP start() failed: %d — falling back to full restart",
              __FUNCTION__, err);
        stopPreview();
        return startPreview(newW, newH);
    }
    err = m_scpPipe->startThread();
    if (err != NO_ERROR) {
        ALOGE("%s: SCP startThread() failed: %d — falling back to full restart",
              __FUNCTION__, err);
        stopPreview();
        return startPreview(newW, newH);
    }

    // Update PipeEngine preview size
    m_previewW = scpW;
    m_previewH = scpH;

    ALOGI("%s: SCP reconfigured to %dx%d (FLITE/ISP/SCC untouched)",
          __FUNCTION__, scpW, scpH);
    return NO_ERROR;
}

status_t G800FPipeEngine::getPreviewFrame(G800FFrame** frame)
{
    if (!m_scpPipe) return NO_INIT;
    return m_scpPipe->getPreviewFrame(frame);
}

void G800FPipeEngine::releasePreviewFrame(G800FFrame* frame)
{
    if (m_scpPipe) m_scpPipe->releasePreviewFrame(frame);
}

status_t G800FPipeEngine::getCaptureFrame(G800FFrame** frame, int timeoutMs)
{
    if (!m_sccPipe) return NO_INIT;
    return m_sccPipe->getCaptureFrame(frame, timeoutMs);
}

void G800FPipeEngine::releaseCaptureFrame(G800FFrame* frame)
{
    if (m_sccPipe) m_sccPipe->releaseCaptureFrame(frame);
}

// Pop warm-path SCC[0] frames (lit frames routed through the warm preview
// ISP[0]) until one matches the still-target fcount — the frame the flash
// redirect selected (firingStable+1).  Non-matching frames are released back
// to the SCC buffer pool; if the target was never produced, the most recent
// lit frame is used as a fallback (all lit frames share the same flash AWB).
status_t G800FPipeEngine::getWarmCaptureFrame(G800FFrame** frame,
                                              int32_t targetFcount,
                                              int timeoutMs)
{
    *frame = NULL;
    if (!m_sccPipe) return NO_INIT;
    /* The still target is firingStable+1 — the exact frame the cold reproc
     * consumed (reprocFrame->frameCount).  Match it exactly when it arrives.
     * As a fallback keep the BRIGHTEST lit frame seen (the flash-lit frame
     * stands out with a high luma mean); a dark/ambient frame must never be
     * substituted — that is exactly the "photo is not the lit frame" bug. */
    G800FFrame* best = NULL;
    uint32_t bestY = 0;
    nsecs_t deadline = systemTime(SYSTEM_TIME_MONOTONIC)
                     + (nsecs_t)timeoutMs * 1000000LL;
    while (true) {
        int64_t remainNs = deadline - systemTime(SYSTEM_TIME_MONOTONIC);
        if (remainNs <= 0) break;
        G800FFrame* f = NULL;
        if (m_sccPipe->getCaptureFrame(&f, (int)(remainNs / 1000000)) != NO_ERROR
                || !f)
            break;
        uint32_t yMean = 0;
        if (f->buffer && f->buffer->planeVaddr(0))
            yMean = sampledByteMean(f->buffer->planeVaddr(0),
                                    f->buffer->planeSize(0), 2);
        // per-frame log disabled.
        //ALOGI("%s: warm SCC fcount=%u target=%d yMean=%u",
        //      __FUNCTION__, (unsigned)f->frameCount, targetFcount, yMean);
        if ((int32_t)f->frameCount == targetFcount) {
            if (best) m_sccPipe->releaseCaptureFrame(best);
            *frame = f;
            return NO_ERROR;
        }
        if (yMean > bestY) {
            if (best) m_sccPipe->releaseCaptureFrame(best);
            best = f;
            bestY = yMean;
        } else {
            m_sccPipe->releaseCaptureFrame(f);
        }
    }
    if (best) {
        ALOGI("%s: target fcount=%d not seen — using brightest lit frame "
              "fcount=%u yMean=%u", __FUNCTION__, targetFcount,
              (unsigned)best->frameCount, bestY);
        *frame = best;
        return NO_ERROR;
    }
    return TIMED_OUT;
}

uint32_t G800FPipeEngine::getFlashReprocTarget() const
{
    return m_ispPipe ? m_ispPipe->getFlashReprocTarget() : 0;
}

uint32_t G800FPipeEngine::waitFlashReprocTarget(int timeoutMs)
{
    /* The redirect target is set when the lit (firingStable+1) frame is pushed
     * to reproc — ~when it enters ISP[1], ~1s before the cold reproc output
     * arrives.  Poll it so the warm path does not have to block on the slow
     * reproc result just to learn which frame to match. */
    nsecs_t deadline = systemTime(SYSTEM_TIME_MONOTONIC)
                     + (nsecs_t)timeoutMs * 1000000LL;
    while (systemTime(SYSTEM_TIME_MONOTONIC) < deadline) {
        uint32_t t = getFlashReprocTarget();
        if (t != 0) return t;
        usleep(2000);
    }
    return 0;
}

bool G800FPipeEngine::needsReprocessing(int targetW, int targetH) const
{
    // Since FLITE runs continuously at full-res (3280x2458) and SCC
    // delivers full-res YUV (3264x2448), reprocessing is always needed
    // for still capture: the Bayer frame is selected from the FLITE
    // stream by the FrameSelector and processed asynchronously.
    return (targetW > 0 && targetH > 0);
}

int G800FPipeEngine::getLatestAfState() const
{
    return m_ispPipe ? m_ispPipe->getLatestAfState() : 0;
}
int G800FPipeEngine::getLatestAfMode() const
{
    return m_ispPipe ? m_ispPipe->getLatestAfMode() : 0;
}
int G800FPipeEngine::getLatestAeState() const
{
    return m_ispPipe ? m_ispPipe->getLatestAeState() : 0;
}
int G800FPipeEngine::getLatestAwbState() const
{
    return m_ispPipe ? m_ispPipe->getLatestAwbState() : 0;
}
int G800FPipeEngine::getLatestFlashReady() const
{
    return m_ispPipe ? m_ispPipe->getLatestFlashReady() : 0;
}
int G800FPipeEngine::getLatestFiringStable() const
{
    return m_ispPipe ? m_ispPipe->getLatestFiringStable() : 0;
}
int G800FPipeEngine::getLatestFlashOffReady() const
{
    return m_ispPipe ? m_ispPipe->getLatestFlashOffReady() : 0;
}

int G800FPipeEngine::getLatestSensitivity() const
{
    return m_ispPipe ? m_ispPipe->getLatestSensitivity() : 0;
}
int64_t G800FPipeEngine::getLatestExposureTime() const
{
    return m_ispPipe ? m_ispPipe->getLatestExposureTime() : 0;
}

void G800FPipeEngine::setAeFlashMode(int mode)
{
    if (m_flitePipe) m_flitePipe->setAeFlashMode(mode);
}

int G800FPipeEngine::getAeFlashMode() const
{
    return m_flitePipe ? m_flitePipe->getAeFlashMode() : 0;
}

// ============================================================================
// writeFlashSeqMetadata — writes the complete flash metadata tuple for a
// flash-sequence state into all FLITE shot_ext buffers (propagates to ISP
// via the per-frame shot.ctl memcpy).
//
// Flash tuples per state:
//
//   state        aeflash    flashMode  firingT/P    aeMode   awbMode
//   START        START(2)   OFF(1)     0/0          tmpl     LOCKED
//   ON/METER     ON(4)      TORCH(3)   10ms/0x3f    tmpl     LOCKED   ← preflash hold!
//   AF_WAIT      ON(4)      TORCH(3)   10ms/0x3f    LOCKED   LOCKED
//   READY        AUTO(5)    OFF(1)     0/0          cond     cond
//   CAPTURE      CAPTURE(6) SINGLE(2)  500µs/0x3f   tmpl     tmpl     ← real strobe
//   MAIN_WAIT    CAPTURE(6) SINGLE(2)  500µs/0x3f   tmpl     tmpl
//   cleanup      OFF(1)     OFF(1)     0/0          tmpl     tmpl
//
// tmpl = session AE/AWB template (m_lastReqAeMode/m_lastReqAwbMode).
//   - the main strobe is driven by ctl.flash.flashMode=SINGLE together
//     with aeflashMode=CAPTURE and a 500µs firingTime — NOT by ON(4);
//   - ON(4)+TORCH(3)+10ms is the *preflash-hold* tuple;
//   - the CAPTURE/SINGLE tuple is kept on every lit frame — there is
//     no separate "ON(4) lit-frame" phase.
//
// The tuple (aeflashMode + flashMode + aeMode + awbMode + firing) MUST be
// written atomically per state — see the crash warning in the state
// machine comment below: firing without aeMode=OFF hits a firmware assert.
// ============================================================================
void G800FPipeEngine::writeFlashSeqMetadata(int state)
{
    int      aeflashMode  = m_flashState;   // keep current by default
    int      aeMode       = -1;             // -1: do not touch
    int      awbMode      = -1;
    int      flashMode    = -1;             // ctl.flash.flashMode (1-based)
    uint32_t firingTime   = 0;
    uint32_t firingPower  = 0;
    bool     writeFiring  = false;

    // The ae/awb "template" values are the SESSION AE/WB modes
    // (center→CENTER, auto→WB_AUTO), i.e. RUNNING 3A — not OFF.
    // AE keeps running through the preflash hold so exposure converges
    // on the lit scene → preview stays properly exposed.  The template
    // is the request-derived m_lastReqAeMode/m_lastReqAwbMode.
    switch (state) {
    case FLASH_SEQ_START:
        // START(2) trigger.
        aeflashMode = AA_FLASHMODE_START;
        flashMode   = CAM2_FLASH_MODE_OFF;
        aeMode      = m_lastReqAeMode;    // template (running AE)
        awbMode     = AA_AWBMODE_LOCKED;
        writeFiring = true;
        break;
    case FLASH_SEQ_ON:
    case FLASH_SEQ_METERING:
        // AE_WAIT / preflash hold: ON(4)+TORCH(3)+10ms.
        // aeMode = template (running AE converges on lit scene),
        // awbMode = LOCKED — always written atomically with the firing
        // fields; a partial tuple (running AWB + firing) was what hit
        // the noiseIndDenum assert in the crash-era code.
        aeflashMode = AA_FLASHMODE_ON;
        flashMode   = CAM2_FLASH_MODE_TORCH;
        aeMode      = m_lastReqAeMode;
        awbMode     = AA_AWBMODE_LOCKED;
        firingTime  = 10000000;
        firingPower = 0x3f;
        writeFiring = true;
        break;
    case FLASH_SEQ_AF_WAIT:
        // AE_DONE/AF_WAIT: same preflash-hold tuple but
        // aeMode=LOCKED(2), awbMode=LOCKED(2).
        aeflashMode = AA_FLASHMODE_ON;
        flashMode   = CAM2_FLASH_MODE_TORCH;
        aeMode      = AA_AEMODE_LOCKED;
        awbMode     = AA_AWBMODE_LOCKED;
        firingTime  = 10000000;
        firingPower = 0x3f;
        writeFiring = true;
        break;
    case FLASH_SEQ_READY:
        // MAIN_READY: AUTO(5)+flash OFF+firing 0.
        // ae/awb are written conditionally: locked when the request
        // asks for a lock (AE_LOCK/AWB_LOCK folded into m_lastReq* at
        // applyRequestSettings), otherwise the session modes.
        aeflashMode = AA_FLASHMODE_AUTO;
        flashMode   = CAM2_FLASH_MODE_OFF;
        aeMode      = m_lastReqAeMode;
        awbMode     = m_lastReqAwbMode;
        writeFiring = true;
        break;
    case FLASH_SEQ_CAPTURE:
        // MAIN_FIRE/MAIN_ON — the REAL main-strobe tuple:
        // ae/awb = session template (same as READY).
        // Written from the trigger frame through all lit frames;
        // there is no ON(4) lit phase.  Exit condition:
        // flashOffReady==2 || firingStable==1 || >16 frames.
        aeflashMode = AA_FLASHMODE_CAPTURE;
        flashMode   = CAM2_FLASH_MODE_SINGLE;
        aeMode      = m_lastReqAeMode;
        awbMode     = m_lastReqAwbMode;
        firingTime  = 500000;             // 0x7a120 — 500µs strobe
        firingPower = 0x3f;
        writeFiring = true;
        break;
    case FLASH_SEQ_MAIN_WAIT:
        // Cleanup: OFF(1)+flash OFF+firing 0, ae/awb = template —
        // the kernel sees AUTO/CAPTURE→OFF and calls force_enable(false).
        aeflashMode = AA_FLASHMODE_OFF;
        flashMode   = CAM2_FLASH_MODE_OFF;
        aeMode      = m_lastReqAeMode;
        // DONE inherits this write (default case writes nothing);
        // IDLE at endFlashSequence() restores the request mode.
        awbMode     = m_lastReqAwbMode;
        writeFiring = true;
        break;
    case FLASH_SEQ_IDLE:
        // Sequence over — restore the request-derived 3A modes cached in
        // applyRequestSettings(): our repeating preview requests carry
        // NULL settings and cannot restore them.
        aeflashMode = AA_FLASHMODE_OFF;
        flashMode   = CAM2_FLASH_MODE_OFF;
        aeMode      = m_lastReqAeMode;
        awbMode     = m_lastReqAwbMode;
        writeFiring = true;
        break;
    default:
        // DONE: OFF(1) is held until endFlashSequence() writes IDLE.
        break;
    }

    m_flashState = aeflashMode;
    for (int i = 0; i < NUM_FLITE_BUFFERS; i++) {
        camera2_shot_ext* shot = m_fliteShotExt[i];
        if (!shot) continue;
        shot->shot.ctl.aa.aeflashMode = (enum aa_ae_flashmode)aeflashMode;
        if (aeMode >= 0)
            shot->shot.ctl.aa.aeMode = (enum aa_aemode)aeMode;
        if (awbMode >= 0)
            shot->shot.ctl.aa.awbMode = (enum aa_awbmode)awbMode;
        if (flashMode >= 0)
            shot->shot.ctl.flash.flashMode = (enum flash_mode)flashMode;
        if (writeFiring) {
            shot->shot.ctl.flash.firingTime  = firingTime;
            shot->shot.ctl.flash.firingPower = firingPower;
        }
    }

    // Warm path: on the CAPTURE-state
    // (flash-lit) frames, request a full-res SCC output from the WARM preview
    // ISP[0] (capture[0].request=1 + leader.output = bayerCrop) so the still is
    // produced by the instance whose AWB just converged on the lit scene —
    // instead of the cold ISP[1] reprocessing which cannot reproduce the
    // flash-weighted AWB state.
    // Arm for the whole flash-lit window: CAPTURE (strobe firing / ramping)
    // AND MAIN_WAIT.  The still target is firingStable+1 — it is queued to
    // the FLITE/ISP right at the CAPTURE→MAIN_WAIT transition, and its
    // node_group is copied from the shared flite template at qBufIsp.  If
    // request=1 is disarmed the instant MAIN_WAIT is entered, the in-flight
    // target frame's ISP shot picks up request=0 and produces no SCC[0] —
    // getWarmCaptureFrame then falls back to a wrong (non-lit) frame.
    int wantScc = (state == FLASH_SEQ_CAPTURE ||
                   state == FLASH_SEQ_MAIN_WAIT) ? 1 : 0;
    int bcw = getSccWidth(), bch = getSccHeight();       // 3264x2448
    int bdsW = 0, bdsH = 0;
    getPreviewBdsSize(m_previewW, m_previewH, &bdsW, &bdsH);
    int loW = wantScc ? bcw : ((bcw < bdsW) ? bcw : bdsW);
    int loH = wantScc ? bch : ((bch < bdsH) ? bch : bdsH);
    for (int i = 0; i < NUM_FLITE_BUFFERS; i++) {
        camera2_shot_ext* s = m_fliteShotExt[i];
        if (!s) continue;
        camera2_node_group& ng = s->node_group;
        ng.capture[0].request = wantScc;
        ng.leader.output.cropRegion[2]    = loW;
        ng.leader.output.cropRegion[3]    = loH;
        ng.capture[0].input.cropRegion[2] = loW;
        ng.capture[0].input.cropRegion[3] = loH;
        ng.capture[1].input.cropRegion[2] = loW;
        ng.capture[1].input.cropRegion[3] = loH;
    }
}

// ============================================================================
// Flash state machine
//
// Sequence: IDLE → START → ON → METERING → AF_WAIT → READY → CAPTURE
//           → MAIN_WAIT → DONE → IDLE
//
// Per-state metadata (templates = session AE/WB modes):
//
//   State     aeflash    flashMode  aeMode     awbMode    firingT/P
//   START     START(2)   OFF(1)     OFF(1)     LOCKED(2)  0 / 0
//   ON/METER  ON(4)      TORCH(3)   OFF(1)     LOCKED(2)  10ms / 0x3f
//   AF_WAIT   ON(4)      TORCH(3)   LOCKED(2)  LOCKED(2)  10ms / 0x3f
//   READY     AUTO(5)    OFF(1)     OFF(1)     LOCKED(2)  0 / 0
//   CAPTURE   CAPTURE(6) SINGLE(2)  OFF(1)     LOCKED(2)  500µs / 0x3f
//   MAIN_WAIT OFF(1)     OFF(1)     OFF(1)     LOCKED(2)  0 / 0
//   DONE/IDLE OFF(1)     OFF(1)     (request)  (request)  0 / 0
//
//   CAPTURE(6)+SINGLE(2)+500µs is written by armFlashCapture() when the
//   still request arrives — this IS the lit-frame tuple, kept on every
//   frame in CAPTURE/MAIN_WAIT (there is no ON(4) lit phase).
//   CAPTURE exit condition: flashOffReady==2 || firingStable==1 ||
//   >16 frames.
//   The kernel fires force_enable + gpio_flash_lock at the
//   AUTO→CAPTURE aeflashMode change; SINGLE(2) + 500µs firingTime is
//   what actually triggers the strobe in the firmware (with OFF(1) or
//   TORCH(3) the aeflash CAPTURE request is not a real strobe).
//
// CRITICAL: write the COMPLETE tuple atomically — never a partial one.
// The 2026-09-19 device crash (ASSERT 0 != noiseIndDenum,
// ISP_SettingsImageControl.c:779 → kernel panic at
// fimc-is-interface.c:2155) came from the naive "ON+firing without
// aeMode/awbMode override" mapping: firing fields were set while
// aeMode/awbMode/flashMode stayed at the preview request values
// (running AWB + firing is the likely assert trigger).
// The tuples keep aeMode = session AE (running) with awbMode=LOCKED
// during the torch preflash, and ae/awb = session modes during the
// real strobe — all written atomically per frame.
//
// applyRequestSettings() must NOT overwrite aeMode/awbMode/aeflashMode
// while the sequence is active — the state machine owns them.
//
// Frame-based transitions (advanceFlashSequence, per FLITE frame):
//   START → ON:      after 3 frames (FLASH_SEQ_START_FRAMES)
//   ON → METERING:   after 3 frames (FLASH_SEQ_ON_FRAMES)
//   METERING → AF_WAIT: on aeState INACTIVE/CONVERGED/LOCKED (min 300ms)
//                    or timeout (1500ms).  aeState==INACTIVE appears when
//                    aeMode=OFF drives the AE state inactive.
//   AF_WAIT → READY: on afState ACQUIRED/FAILED/INACTIVE (min 150ms)
//                    or timeout (1200ms) — wait for AF done with ~2s
//                    budget; afState==INACTIVE covers afMode=OFF.
//
// Caller-driven transitions:
//   READY → CAPTURE: armFlashCapture() — gated by waitFlashSequenceReady()
//                    on dm.flash.flashReady==2 (MAIN_READY: firmware
//                    computed the main-flash parameters) with a ~1.1 s
//                    timeout (~31 frames)
//   CAPTURE/DONE → IDLE: endFlashSequence() (with releaseFlash sysfs 'U')
//
// The FLITE thread calls advanceFlashSequence() per frame. The request worker
// polls isFlashSequenceReady() and serves preview requests in parallel.
// ============================================================================

const char* G800FPipeEngine::flashSeqStateName(int state)
{
    switch (state) {
    case FLASH_SEQ_IDLE:      return "IDLE";
    case FLASH_SEQ_START:     return "START";
    case FLASH_SEQ_ON:        return "ON";
    case FLASH_SEQ_METERING:  return "METERING";
    case FLASH_SEQ_AF_WAIT:   return "AF_WAIT";
    case FLASH_SEQ_READY:     return "READY";
    case FLASH_SEQ_CAPTURE:   return "CAPTURE";
    case FLASH_SEQ_MAIN_WAIT: return "MAIN_WAIT";
    case FLASH_SEQ_DONE:      return "DONE";
    default:                  return "?";
    }
}

void G800FPipeEngine::beginFlashSequence()
{
    Mutex::Autolock l(m_flashSeqLock);
    if (m_flashSeqState != FLASH_SEQ_IDLE) {
        ALOGW("FLASHSEQ: begin ignored — sequence already active in state %s",
              flashSeqStateName(m_flashSeqState));
        return;
    }
    nsecs_t now = systemTime(SYSTEM_TIME_MONOTONIC);
    m_flashSeqState         = FLASH_SEQ_START;
    m_flashSeqStateFrames   = 0;
    m_flashSeqCaptureFrames = 0;
    m_flashSeqStateEnter    = now;
    m_flashSeqBegin         = now;
    m_flashReadyMain        = false;
    // Early strobe critical-section lock: clears the FLED TA (charger)
    // status for the WHOLE sequence so the preflash torch works on a
    // charging device (see armFlashLock).
    armFlashLock();
    m_flashState            = AA_FLASHMODE_START;

    // aeflashMode=START → kernel calls rt5033_flash_force_enable(true)
    // → pre-flash LED on.  PREFLASH tuple: aeMode=OFF,
    // awbMode=LOCKED, firing=0/0 (firmware does internal flash metering).
    writeFlashSeqMetadata(FLASH_SEQ_START);
    ALOGI("FLASHSEQ: begin → START (preflash armed), aeState=%d",
          getLatestAeState());
}

void G800FPipeEngine::advanceFlashSequence(uint32_t frameCount)
{
    Mutex::Autolock l(m_flashSeqLock);
    if (m_flashSeqState == FLASH_SEQ_IDLE)
        return;

    m_flashSeqStateFrames++;
    nsecs_t now = systemTime(SYSTEM_TIME_MONOTONIC);
    int elapsedMs = (int)((now - m_flashSeqStateEnter) / 1000000);
    int totalMs   = (int)((now - m_flashSeqBegin) / 1000000);
    int aeState   = getLatestAeState();

    // MAIN_READY gate: latch the firmware's "main flash ready" flag
    // (dm.flash.flashReady==2) — set once the preflash metering result is
    // in.  A waiter blocked in waitFlashSequenceReady() wakes on the
    // broadcast; the gate itself is evaluated there.
    if (!m_flashReadyMain && getLatestFlashReady() == 2) {
        m_flashReadyMain = true;
        ALOGI("FLASHSEQ: flashReady==2 (main flash computed) fcount=%u "
              "state=%s totalMs=%d",
              frameCount, flashSeqStateName(m_flashSeqState), totalMs);
        if (m_flashSeqState == FLASH_SEQ_READY)
            m_flashSeqCond.broadcast();
    }

    // CRITICAL sensor protection watchdog: if the entire flash sequence runs
    // longer than FLASH_SEQ_TOTAL_TIMEOUT_MS (8s), force it off.
    // This prevents sensor overheating if the caller never calls armFlashCapture()
    // or endFlashSequence() (e.g. capture thread crash,
    // ANR, or framework bug). The watchdog runs in the FLITE thread and
    // is therefore independent of the capture thread state.
    if (totalMs >= FLASH_SEQ_TOTAL_TIMEOUT_MS) {
        ALOGE("FLASHSEQ: ⚠ TOTAL TIMEOUT %dms in state %s — FORCING OFF "
              "(fcount=%u aeState=%d) — sensor protection!",
              totalMs, flashSeqStateName(m_flashSeqState), frameCount, aeState);
        writeFlashSeqMetadata(FLASH_SEQ_IDLE);   // OFF + firing=0/0
        setAeFlashMode(AA_FLASHMODE_OFF);
        m_flashSeqState         = FLASH_SEQ_IDLE;
        m_flashSeqStateFrames   = 0;
        m_flashSeqCaptureFrames = 0;
        m_flashSeqStateEnter    = now;
        if (m_frameSelector) m_frameSelector->setMainFlashArmed(false);
        m_flashSeqCond.broadcast();
        releaseFlash();
        return;
    }

    int next = m_flashSeqState;
    const char* reason = NULL;

    switch (m_flashSeqState) {
    case FLASH_SEQ_START:
        if (m_flashSeqStateFrames >= FLASH_SEQ_START_FRAMES) {
            next = FLASH_SEQ_ON;
            reason = "START frames done";
        }
        break;
    case FLASH_SEQ_ON:
        if (m_flashSeqStateFrames >= FLASH_SEQ_ON_FRAMES) {
            next = FLASH_SEQ_METERING;
            reason = "ON frames done";
        }
        break;
    case FLASH_SEQ_METERING:
        // FIMC ae_state (1-based, fimc-is-metadata.h:699-706):
        // 1=INACTIVE 2=SEARCHING 3=CONVERGED 4=LOCKED 5=FLASH_REQUIRED 6=PRECAPTURE
        // AE_WAIT exits on aeState==INACTIVE — with aeMode=OFF the
        // firmware's regular AE goes inactive while its internal flash
        // metering runs.  Keep CONVERGED/LOCKED as fallback exits.
        if (elapsedMs >= FLASH_SEQ_METERING_MIN_MS &&
            (aeState == AE_STATE_INACTIVE || aeState == AE_STATE_CONVERGED ||
             aeState == AE_STATE_LOCKED)) {
            next = FLASH_SEQ_AF_WAIT;
            reason = "AE done";
        } else if (elapsedMs >= FLASH_SEQ_METERING_MAX_MS) {
            next = FLASH_SEQ_AF_WAIT;
            reason = "AE metering TIMEOUT";
        }
        break;
    case FLASH_SEQ_AF_WAIT:
        // AF_WAIT: AF scans under preflash light, exits on
        // flashStep==2 (AF activity done) with a ~2s budget.  We observe
        // dm.aa.afState instead: 4=ACQUIRED_FOCUS 5=FAILED_FOCUS are the
        // terminal scan states; INACTIVE(1) covers afMode=OFF (front cam,
        // fixed focus) so we don't stall the sequence.
        {
            int afState = getLatestAfState();
            if (elapsedMs >= FLASH_SEQ_AF_MIN_MS &&
                (afState == AA_AFSTATE_AF_ACQUIRED_FOCUS ||
                 afState == AA_AFSTATE_AF_FAILED_FOCUS ||
                 afState == AA_AFSTATE_INACTIVE)) {
                next = FLASH_SEQ_READY;
                reason = "AF done";
            } else if (elapsedMs >= FLASH_SEQ_AF_MAX_MS) {
                next = FLASH_SEQ_READY;
                reason = "AF wait TIMEOUT";
            }
        }
        break;
    case FLASH_SEQ_CAPTURE:
        // armFlashCapture already wrote the real main-strobe tuple
        // (CAPTURE(6)+SINGLE(2)+500µs/0x3f) — kept on every lit frame,
        // there is no ON(4) phase.
        // Exit condition: flashOffReady==2 ||
        // firingStable==1 || timeout >16 frames.
        if (getLatestFiringStable() == 1) {
            next = FLASH_SEQ_MAIN_WAIT;
            reason = "main flash firing stable";
        } else if (getLatestFlashOffReady() == 2) {
            next = FLASH_SEQ_MAIN_WAIT;
            reason = "main flash off ready";
        } else if (m_flashSeqStateFrames > 16) {
            next = FLASH_SEQ_MAIN_WAIT;
            reason = "main flash frame timeout";
        }
        break;
    case FLASH_SEQ_MAIN_WAIT:
        // P2: after 1 frame MAIN_WAIT → DONE (like pipeline CLEANUP).
        // Flash is now safely off (aeflashMode=OFF written in MAIN_WAIT —
        // cleanup tuple).
        if (m_flashSeqStateFrames >= 1) {
            next = FLASH_SEQ_DONE;
            reason = "main wait done";
        }
        break;
    default:
        // READY / DONE: caller-driven, not frame-based
        break;
    }

    if (m_flashSeqState == FLASH_SEQ_CAPTURE)
        m_flashSeqCaptureFrames++;

    if (next == m_flashSeqState)
        return;

    if (next == FLASH_SEQ_READY && m_ispPipe)
        m_ispPipe->resetLatestFiringStable();
    // Writes the full tuple for the new state (aeflashMode + aeMode +
    // awbMode + firing) and updates m_flashState.
    writeFlashSeqMetadata(next);
    int mode = m_flashState;

    ALOGI("FLASHSEQ: %s -> %s (%s) fcount=%u aeState=%d stateMs=%d totalMs=%d "
          "frames=%u aeflashMode=%d",
          flashSeqStateName(m_flashSeqState), flashSeqStateName(next),
          reason ? reason : "-", frameCount, aeState, elapsedMs, totalMs,
          m_flashSeqStateFrames, mode);

    m_flashSeqState       = next;
    m_flashSeqStateFrames = 0;
    m_flashSeqStateEnter  = now;
    if (next == FLASH_SEQ_READY)
        m_flashSeqCond.broadcast();
}

bool G800FPipeEngine::isFlashSequenceActive() const
{
    Mutex::Autolock l(m_flashSeqLock);
    return m_flashSeqState != FLASH_SEQ_IDLE;
}

bool G800FPipeEngine::isFlashSequenceReady() const
{
    Mutex::Autolock l(m_flashSeqLock);
    if (m_flashSeqState == FLASH_SEQ_CAPTURE ||
        m_flashSeqState == FLASH_SEQ_MAIN_WAIT ||
        m_flashSeqState == FLASH_SEQ_DONE)
        return true;
    if (m_flashSeqState != FLASH_SEQ_READY)
        return false;
    // MAIN_READY: the capture arm must wait until the firmware signals
    // dm.flash.flashReady==2 (main-flash parameters computed from
    // preflash metering) — bounded by a ~31-frame timeout so a missing
    // flag can't stall the capture forever.
    if (m_flashReadyMain)
        return true;
    return systemTime(SYSTEM_TIME_MONOTONIC) - m_flashSeqStateEnter >=
           (nsecs_t)FLASH_SEQ_READY_TIMEOUT_MS * 1000000LL;
}

bool G800FPipeEngine::waitFlashSequenceReady(int timeoutMs)
{
    Mutex::Autolock l(m_flashSeqLock);
    nsecs_t deadline = systemTime(SYSTEM_TIME_MONOTONIC) +
                       (nsecs_t)timeoutMs * 1000000LL;
    for (;;) {
        if (m_flashSeqState == FLASH_SEQ_CAPTURE ||
            m_flashSeqState == FLASH_SEQ_MAIN_WAIT ||
            m_flashSeqState == FLASH_SEQ_DONE)
            return true;
        if (m_flashSeqState == FLASH_SEQ_READY) {
            // MAIN_READY gate: flashReady==2 = firmware finished the
            // preflash metering and computed the main-flash parameters.
            // Firing CAPTURE earlier is a blind strobe (blown highlights
            // on bright scenes).  Timeout ~31 frames.
            nsecs_t now = systemTime(SYSTEM_TIME_MONOTONIC);
            if (m_flashReadyMain ||
                now - m_flashSeqStateEnter >=
                    (nsecs_t)FLASH_SEQ_READY_TIMEOUT_MS * 1000000LL)
                return true;
        }
        if (m_flashSeqState == FLASH_SEQ_IDLE)
            return false;   // watchdog or endFlashSequence()
        nsecs_t remaining = deadline - systemTime(SYSTEM_TIME_MONOTONIC);
        if (remaining <= 0)
            return false;
        // Wake at the READY-timeout at the latest — the cond broadcast on
        // flashReady==2 or on a state change usually arrives earlier.
        if (m_flashSeqState == FLASH_SEQ_READY) {
            nsecs_t toReadyTimeout = m_flashSeqStateEnter +
                (nsecs_t)FLASH_SEQ_READY_TIMEOUT_MS * 1000000LL -
                systemTime(SYSTEM_TIME_MONOTONIC);
            if (toReadyTimeout > 0 && toReadyTimeout < remaining)
                remaining = toReadyTimeout;
        }
        m_flashSeqCond.waitRelative(m_flashSeqLock, remaining);
    }
}

int G800FPipeEngine::getFlashSeqState() const
{
    Mutex::Autolock l(m_flashSeqLock);
    return m_flashSeqState;
}

uint32_t G800FPipeEngine::getFlashCaptureFrames() const
{
    return m_flashSeqCaptureFrames;
}

void G800FPipeEngine::armFlashCapture()
{
    Mutex::Autolock l(m_flashSeqLock);
    if (m_flashSeqState != FLASH_SEQ_READY) {
        ALOGW("FLASHSEQ: armFlashCapture in unexpected state %s — arming anyway",
              flashSeqStateName(m_flashSeqState));
    }
    nsecs_t now = systemTime(SYSTEM_TIME_MONOTONIC);
    int totalMs = (int)((now - m_flashSeqBegin) / 1000000);
    // CAPTURE: the next frame write carries the real main-strobe tuple:
    // aeflashMode=CAPTURE(6) + ctl.flash.flashMode=SINGLE(2) +
    // firingTime=500000 (500µs) + firingPower=0x3f + template ae/awb.
    // The kernel does rt5033_flash_force_enable(true) +
    // rt5033_gpio_flash_lock(true) at the aeflashMode AUTO→CAPTURE change
    // (the critical section is already held by our early armFlashLock,
    // so the lock there is a no-op — "strobe already locked!").
    writeFlashSeqMetadata(FLASH_SEQ_CAPTURE);
    setAeFlashMode(AA_FLASHMODE_CAPTURE);
    if (m_ispPipe) m_ispPipe->resetLatestFiringStable();
    m_flashSeqState         = FLASH_SEQ_CAPTURE;
    m_flashSeqStateFrames   = 0;
    m_flashSeqCaptureFrames = 0;
    m_flashSeqStateEnter    = now;
    // FLITE thread: from now on accept frames for reproc (main flash armed)
    if (m_frameSelector) m_frameSelector->setMainFlashArmed(true);
    ALOGI("FLASHSEQ: %s -> CAPTURE (main flash armed) aeState=%d totalMs=%d",
          flashSeqStateName(FLASH_SEQ_READY), getLatestAeState(), totalMs);
}

void G800FPipeEngine::endFlashSequence()
{
    int totalMs = 0;
    {
        Mutex::Autolock l(m_flashSeqLock);
        if (m_flashSeqState == FLASH_SEQ_IDLE)
            return;
        nsecs_t now = systemTime(SYSTEM_TIME_MONOTONIC);
        totalMs = (int)((now - m_flashSeqBegin) / 1000000);
        // OFF → kernel: rt5033_flash_force_enable(false) → LED off.
        // Also resets firingTime/firingPower (cleanup writes firing=0).
        writeFlashSeqMetadata(FLASH_SEQ_IDLE);
        setAeFlashMode(AA_FLASHMODE_OFF);  // reset the per-frame stamp too
        ALOGI("FLASHSEQ: %s -> OFF (LED off after %dms, captureFrames=%u)",
              flashSeqStateName(m_flashSeqState), totalMs,
              m_flashSeqCaptureFrames);
        m_flashSeqState         = FLASH_SEQ_IDLE;
        m_flashSeqStateFrames   = 0;
        m_flashSeqCaptureFrames = 0;
        m_flashSeqStateEnter    = now;
        if (m_frameSelector) m_frameSelector->setMainFlashArmed(false);
        m_flashSeqCond.broadcast();
    }
    // Release the RT5033 strobe critical section ('U' on rear_flash_ext),
    // as after every flash capture.
    releaseFlash();

    // Wait up to 1s for flash shutdown (500ms + 10×50ms polling).
    // We wait 100ms as hardware stabilization time after LED off.
    // The RT5033 needs a few ms to return from strobe mode to idle mode.
    // Without this wait, the next flash capture may fail because the
    // LED driver is not yet ready.
    usleep(100000);  // 100ms
}

void G800FPipeEngine::finishFlashCapture()
{
    // API compatibility — endFlashSequence() does OFF + strobe unlock.
    endFlashSequence();
}

status_t G800FPipeEngine::releaseFlash()
{
    // Unlock the RT5033 strobe after capture.
    int fd = open("/sys/class/camera/flash/rear_flash_ext", O_WRONLY);
    if (fd < 0) {
        ALOGE("%s: open rear_flash_ext failed: %s", __FUNCTION__, strerror(errno));
        return UNKNOWN_ERROR;
    }
    int ret = write(fd, "U", 1);
    close(fd);
    if (ret != 1) {
        ALOGE("%s: write 'U' failed: %s", __FUNCTION__, strerror(errno));
        return UNKNOWN_ERROR;
    }
    ALOGI("%s: flash strobe unlocked after capture", __FUNCTION__);
    return NO_ERROR;
}

status_t G800FPipeEngine::armFlashLock()
{
    // Lock the RT5033 strobe critical section EARLY (sequence start).
    // rt5033_fled_strobe_critial_section_lock() also clears the FLED TA
    // (charger) status — without this, a charging device stays in the
    // TA-limited mode after the previous capture's 'U' (which restores
    // the REAL charger state), and the preflash torch (force_enable
    // BEFORE the CAPTURE-time gpio_flash_lock) stays dark: later captures
    // ran completely without flash (dm.flash.flashReady never 2,
    // photos torch-less, 2026-09-23).  The kernel's CAPTURE-time lock
    // becomes a harmless "strobe already locked!" no-op.
    int fd = open("/sys/class/camera/flash/rear_flash_ext", O_WRONLY);
    if (fd < 0) {
        ALOGE("%s: open rear_flash_ext failed: %s", __FUNCTION__, strerror(errno));
        return UNKNOWN_ERROR;
    }
    int ret = write(fd, "L", 1);
    close(fd);
    if (ret != 1) {
        ALOGE("%s: write 'L' failed: %s", __FUNCTION__, strerror(errno));
        return UNKNOWN_ERROR;
    }
    ALOGI("%s: flash strobe locked early (TA override for whole sequence)",
          __FUNCTION__);
    return NO_ERROR;
}

// ============================================================================
// DEPRECATED: reconfigureFlite — no longer used.
// FLITE runs continuously at 3280x2458, no setSize on capture.
// ============================================================================

status_t G800FPipeEngine::reconfigureFlite(int newW, int newH)
{
    // DEPRECATED: FLITE runs continuously at 3280x2458.
    // No setSize on capture — FrameSelector selects asynchronously.
    // TODO: Remove once all references are updated.
    ALOGW("%s: DEPRECATED — FLITE runs continuously, no setSize (%dx%d)",
          __FUNCTION__, newW, newH);
    (void)newW;
    (void)newH;
    return INVALID_OPERATION;
}

// ============================================================================
//  requestStillCapture — asynchronous still-capture request

//
//  FLITE runs continuously at 3280x2458. No setSize, no stream off.
//  FrameSelector selects the FLITE Bayer frame (pipeId=0) with a 200ms timeout.
//  On flash: ISP redirects the follow-up frame (fcount+1) to reprocessing
//  at firingStable==1 (capture target = firingStable frame + 1).
//  The reprocessing pipeline (ISP_REPROC → SCC_REPROC) processes asynchronously.
// ============================================================================

status_t G800FPipeEngine::requestStillCapture(int targetW, int targetH,
                                              bool needFlash)
{
    if (!m_initialized || !m_previewRunning) {
        ALOGE("%s: not initialized or preview not running", __FUNCTION__);
        return NO_INIT;
    }
    if (m_reprocessing) {
        ALOGE("%s: reprocessing already in progress", __FUNCTION__);
        return ALREADY_EXISTS;
    }

    ALOGI("%s: target=%dx%d flash=%d (asynchron, kein setSize)",
          __FUNCTION__, targetW, targetH, needFlash);

    m_reprocessing = true;

    // Reset FrameSelector for new capture sequence
    if (m_frameSelector) {
        m_frameSelector->reset();
        m_frameSelector->setFlashCapture(needFlash);
        // Flash frames are selected ISP-side via firingStable.
        m_frameSelector->setFrameSkipCount(0);
        m_frameSelector->setMainFlashArmed(false);  // set in armFlashCapture()
    }

    // Start reprocessing pipeline (ISP_REPROC + SCC_REPROC, instance 1)
    // FLITE stays on instance 0, continues running continuously.
    int ispW, ispH, sccW, sccH;
    if (m_cameraId == 0) {
        ispW = FLITE_W; ispH = FLITE_H;  // 3280x2458
        sccW = SCC_W;   sccH = SCC_H;    // 3264x2448
    } else {
        ispW = FLITE_FRONT_W; ispH = FLITE_FRONT_H;  // 1936x1090
        sccW = SCC_FRONT_W;   sccH = SCC_FRONT_H;   // 1920x1080
    }

    // Reprocessing nodes AND buffers were already created in
    // startPreview() in a separate thread (async). The Exynos 3470
    // kernel does not allow s_fmt on ISP instance 1 while instance 0 is streaming.
    //
    // If the async setup thread is still running, we wait for it — bounded.
    // The thread does V4L2 s_fmt/setInput on ISP[1]/SCC[1]; the kernel can
    // serialize instance-1 config against the streaming ISP[0], so an
    // unbounded join() would freeze the capture worker during the flash
    // sequence (Snap: torch on, no photo, preview alive).  We poll the
    // completion flag with a timeout; only once it clears is join() instant.
    if (m_reprocSetupThread.joinable() && m_reprocSetupInProgress) {
        ALOGI("%s: waiting for async reproc setup thread to finish", __FUNCTION__);
        nsecs_t deadline = systemTime(SYSTEM_TIME_MONOTONIC)
                         + (nsecs_t)REPROC_SETUP_TIMEOUT_MS * 1000000LL;
        while (m_reprocSetupInProgress &&
               systemTime(SYSTEM_TIME_MONOTONIC) < deadline) {
            usleep(2000);
        }
        if (m_reprocSetupInProgress) {
            ALOGE("%s: reproc setup thread wedged (>=%dms) — aborting still",
                  __FUNCTION__, REPROC_SETUP_TIMEOUT_MS);
            if (needFlash) {
                ALOGW("%s: flash active during wedged reproc setup, forcing OFF",
                      __FUNCTION__);
                endFlashSequence();
            }
            m_reprocessing = false;
            return TIMED_OUT;
        }
    }
    if (m_reprocSetupThread.joinable()) {
        // Thread signalled completion — this join() is guaranteed fast.
        m_reprocSetupThread.join();
    }
    if (!m_reprocNodesReady) {
        ALOGW("%s: reprocessing nodes not pre-configured, trying late setup (may fail)",
              __FUNCTION__);
        status_t setupErr = setupReprocessingNodes(ispW, ispH, sccW, sccH);
        if (setupErr != NO_ERROR) {
            ALOGE("%s: late setupReprocessingNodes failed: %d", __FUNCTION__, setupErr);
            if (needFlash) {
                ALOGW("%s: Flash active during setupReprocessingNodes failure, forcing OFF",
                      __FUNCTION__);
                endFlashSequence();
            }
            m_reprocessing = false;
            return setupErr;
        }
        m_reprocNodesReady = true;
    }

    // Buffers were already allocated async in startPreview().
    // If they are not ready yet (async thread not finished or failed),
    // we allocate them here synchronously.
    if (!m_reprocBuffersReady) {
        ALOGW("%s: reprocessing buffers not pre-allocated, allocating now", __FUNCTION__);
        status_t allocErr = allocReprocessingBuffers(ispW, ispH, sccW, sccH);
        if (allocErr != NO_ERROR) {
            ALOGE("%s: allocReprocessingBuffers failed: %d", __FUNCTION__, allocErr);
            releaseReprocessingBuffers();
            if (needFlash) {
                ALOGW("%s: Flash active during allocReprocessingBuffers failure, forcing OFF",
                      __FUNCTION__);
                endFlashSequence();
            }
            m_reprocessing = false;
            return allocErr;
        }
        m_reprocBuffersReady = true;
    }

    // Only start the pipeline if it is not yet streaming.
    // For the first photo after startPreview(), m_reprocPipelineStreaming=false.
    // For subsequent photos, the pipeline stays streaming (no STREAMOFF/ON).
    if (!m_reprocPipelineStreaming) {
        status_t err = startReprocessingPipeline();
        if (err != NO_ERROR) {
            ALOGE("%s: startReprocessingPipeline failed: %d", __FUNCTION__, err);
            if (needFlash) {
                ALOGW("%s: Flash active during startReprocessingPipeline failure, forcing OFF",
                      __FUNCTION__);
                endFlashSequence();
            }
            m_reprocessing = false;
            return err;
        }
        m_reprocPipelineStreaming = true;
    } else {
        ALOGI("%s: reproc pipeline already streaming — skipping STREAMON", __FUNCTION__);
    }

    if (needFlash)
        m_ispPipe->setFlashCaptureRedirect(&m_bayerForReprocQ);
    else
        m_flitePipe->setCaptureRedirect(&m_bayerForReprocQ);

    // Start the asynchronous reprocessing thread
    m_reprocThreadRunning = true;
    m_reprocThread = new G800FReprocThread(this);
    m_reprocThread->run("G800FReproc", PRIORITY_URGENT_DISPLAY);

    ALOGI("%s: still capture requested (async, frameSelector + reprocThread active)",
          __FUNCTION__);
    return NO_ERROR;
}

status_t G800FPipeEngine::getReprocessedFrame(G800FFrame** frame, int timeoutMs)
{
    if (!m_reprocessing) {
        ALOGE("%s: no reprocessing in progress", __FUNCTION__);
        return NO_INIT;
    }

    // Wait for a frame from the reprocessing output queue
    status_t err = m_reprocOutputQ.pop_front(frame, timeoutMs);
    if (err != NO_ERROR) {
        ALOGE("%s: timeout waiting for reprocessed frame (%dms): %d",
              __FUNCTION__, timeoutMs, err);
        return err;
    }

    ALOGI("%s: reprocessed frame ready (pipeId=%d)",
          __FUNCTION__, (*frame)->producerPipeId);
    return NO_ERROR;
}

void G800FPipeEngine::releaseReprocessingFrame(G800FFrame* frame)
{
    if (!frame) return;
    // Release FLITE buffer (was borrowed for reprocessing)
    if (frame->producerPipeId == PIPE_FLITE && m_flitePipe) {
        m_flitePipe->requeueBuffer(frame->bufferIndex);
    }
    // Release frame object
    delete frame;
}

void G800FPipeEngine::finishStillCapture()
{
    ALOGI("%s: finishing still capture (preview continues undisturbed)",
          __FUNCTION__);

    // Cancel capture redirect → preview pipeline continues normally
    if (m_flitePipe) m_flitePipe->clearCaptureRedirect();
    if (m_ispPipe) m_ispPipe->clearFlashCaptureRedirect();

    // The reprocessing pipeline (ISP[1]+SCC[1]) stays streaming
    // between photos — the reproc worker keeps running without
    // stopPipes(); stopPipes() only runs on stopPreview.
    //
    // The reproc thread is stopped (it waits for frames from the queue).
    // On the next requestStillCapture() it is restarted.
    // The pipeline stays STREAMON — no STREAMOFF/STREAMON between photos.
    //
    // Deadlock avoidance: the thread waits in pop_front(1000ms timeout).
    // When m_reprocThreadRunning is set to false, it wakes up after at most
    // 1s and exits (return false). It does not reach dqbuf because it only
    // continues on a successful pop_front and we drain the queue.
    m_reprocThreadRunning = false;
    m_bayerForReprocQ.wakeAll();  // wake thread from pop_front

    // Return remaining frames in the queue to FLITE
    G800FFrame* leftover = NULL;
    while (m_bayerForReprocQ.try_pop_front(&leftover) == NO_ERROR && leftover) {
        if (m_flitePipe) {
            m_flitePipe->clearFliteInUse(leftover->bufferIndex);
            m_flitePipe->requeueBuffer(leftover->bufferIndex);
        }
        delete leftover;
    }

    // Join thread — it exits after timeout/wakeAll (no blocking dqbuf)
    if (m_reprocThread != NULL) {
        m_reprocThread->requestExitAndWait();
        m_reprocThread.clear();
    }

    // Reset FrameSelector
    if (m_frameSelector) {
        m_frameSelector->cancel(NULL);  // discard candidates
        m_frameSelector->reset();
    }

    // Clean output queue — pop + delete leftover output frames.  The SCC[1]
    // buffer is already requeued by the reproc thread and the FLITE input
    // buffer already returned, so a plain clear() would only leak the small
    // G800FFrame object — but with the warm path the reproc output is never
    // consumed, so this now runs on every flash photo.
    {
        G800FFrame* outLeft = NULL;
        while (m_reprocOutputQ.try_pop_front(&outLeft) == NO_ERROR && outLeft) {
            delete outLeft;
            outLeft = NULL;
        }
    }

    // Drain any leftover warm-path SCC[0] frames (lit frames that weren't the
    // still target) so the next capture starts with a clean capture queue.
    if (m_sccPipe) {
        G800FFrame* sccLeft = NULL;
        while (m_sccPipe->getCaptureFrame(&sccLeft, 0) == NO_ERROR && sccLeft) {
            m_sccPipe->releaseCaptureFrame(sccLeft);
            sccLeft = NULL;
        }
    }

    m_reprocessing = false;
    ALOGI("%s: done (preview still running: %d, pipeline streaming: %d)",
          __FUNCTION__, m_previewRunning ? 1 : 0, m_reprocPipelineStreaming ? 1 : 0);
}

status_t G800FPipeEngine::setupReprocessingNodes(int ispW, int ispH,
                                                  int sccW, int sccH)
{
    ALOGI("%s: ISP[1]=%dx%d SCC[1]=%dx%d", __FUNCTION__, ispW, ispH, sccW, sccH);

    // --- ISP[1] (Instance 1) ---
    status_t err = m_reprocIspNode.create(NODE_ISP);
    if (err != NO_ERROR) return err;
    err = m_reprocIspNode.open();
    if (err != NO_ERROR) return err;
    err = m_reprocIspNode.setInput(packSensorIdISPReproc(m_cameraId));
    if (err != NO_ERROR) {
        ALOGE("%s: ISP[1] s_input(0x%x) failed: %d",
              __FUNCTION__, packSensorIdISPReproc(m_cameraId), err);
        return err;
    }
    // Load setfile 5 (ISS_SUB_SCENARIO_STILL_CAPTURE) on ISP[1].
    // This configures BDS for full-res capture output.
    // Must be called BEFORE stream-on.
    err = m_reprocIspNode.setControl(V4L2_CID_IS_SET_SETFILE, 5);
    if (err != NO_ERROR) {
        ALOGW("%s: ISP[1] V4L2_CID_IS_SET_SETFILE(5) failed: %d (non-fatal)",
              __FUNCTION__, err);
    } else {
        ALOGI("%s: ISP[1] setfile 5 (STILL_CAPTURE) loaded", __FUNCTION__);
    }
    err = m_reprocIspNode.setSize(ispW, ispH);
    if (err != NO_ERROR) return err;
    err = m_reprocIspNode.setColorFormat(V4L2_PIX_FMT_SBGGR12, 2);
    if (err != NO_ERROR) return err;
    // ISP[1] (leader/INPUT) uses V4L2_MEMORY_DMABUF.
    // IMPORTANT: stable DMA address! We use our own ION buffers
    // (m_reprocIspBuffers) for BOTH planes — NOT the FLITE buffer directly.
    // The kernel stores the DMA address per buffer index. If we use a
    // different FLITE buffer (with a different DMA address) for idx=0
    // each frame, fimc_is_queue_buffer_queue rejects the qbuf with EINVAL
    // ("buffer %d plane %d is changed" in fimc-is-video.c:549).
    // Solution: copy Bayer data from the FLITE DMABUF into our ISP ION buffer
    // DMABUF gives us automatic
    // cache coherency (vb2_ion_sync_for_device in vb2_ion_map_dmabuf).
    err = m_reprocIspNode.setBufferType(2, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                                        V4L2_MEMORY_DMABUF);
    if (err != NO_ERROR) return err;
    err = m_reprocIspNode.setFormat();
    if (err != NO_ERROR) return err;
    err = m_reprocIspNode.reqBuffers(NUM_REPROC_BUFFERS);
    if (err != NO_ERROR) return err;

    // --- SCC[1] (Instance 1) ---
    err = m_reprocSccNode.create(NODE_SCC);
    if (err != NO_ERROR) return err;
    err = m_reprocSccNode.open();
    if (err != NO_ERROR) return err;
    err = m_reprocSccNode.setInput(packSensorIdSCCReproc(m_cameraId));
    if (err != NO_ERROR) {
        ALOGE("%s: SCC[1] s_input(0x%x) failed: %d",
              __FUNCTION__, packSensorIdSCCReproc(m_cameraId), err);
        return err;
    }
    err = m_reprocSccNode.setSize(sccW, sccH);
    if (err != NO_ERROR) return err;
    // SCC_REPROCESSING uses YUYV (2 planes), not NV21M.
    err = m_reprocSccNode.setColorFormat(V4L2_PIX_FMT_YUYV, SCC_REPROC_PLANES);
    if (err != NO_ERROR) return err;
    err = m_reprocSccNode.setBufferType(SCC_REPROC_PLANES, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                                        V4L2_MEMORY_DMABUF);
    if (err != NO_ERROR) return err;
    err = m_reprocSccNode.setFormat();
    if (err != NO_ERROR) return err;
    err = m_reprocSccNode.reqBuffers(NUM_SCC_REPROC_BUFFERS);
    if (err != NO_ERROR) return err;

    ALOGI("%s: ISP[1] + SCC[1] configured", __FUNCTION__);
    return NO_ERROR;
}

void G800FPipeEngine::closeReprocessingNodes()
{
    // Close reprocessing nodes (after stopReprocessingPipeline).
    // Called by stopPreview() to release the nodes opened in startPreview().
    // The nodes were opened and configured in a separate thread,
    // so we must first wait for the thread before closing the nodes
    // (avoid race condition).
    ALOGI("%s: closing reprocessing nodes", __FUNCTION__);

    // Wait for the async setup thread if it is still running.
    // The thread could still be opening/configuring nodes — we must not
    // call close() in parallel.
    if (m_reprocSetupThread.joinable()) {
        ALOGI("%s: joining reproc setup thread before closing nodes", __FUNCTION__);
        m_reprocSetupThread.join();
    }

    if (m_reprocIspNode.isOpen()) {
        m_reprocIspNode.close();
    }
    if (m_reprocSccNode.isOpen()) {
        m_reprocSccNode.close();
    }
    m_reprocNodesReady = false;
}

status_t G800FPipeEngine::allocReprocessingBuffers(int ispW, int ispH,
                                                    int sccW, int sccH)
{
    // ISP[1] buffers: plane 0 = Bayer, plane 1 = SPARE (shot_ext).
    // Buffer index == fliteIdx (see reprocThreadLoop): the qbuf passes the
    // FLITE buffer's own dmaBufFd for plane 0, so our own plane-0 memory is
    // never used — one page is enough.  Plane 1 carries the shot_ext we
    // build per capture (its dmaBufFd is what the driver reads).
    (void)ispW; (void)ispH;
    size_t ispPlaneSizes[2] = { 4096, G800FPipeEngine::SPARE_SIZE };

    for (int i = 0; i < NUM_REPROC_BUFFERS; i++) {
        m_reprocIspBuffers[i] = new G800FExynosCameraBuffer();
        status_t err = m_reprocIspBuffers[i]->allocDmabuf(2, ispPlaneSizes,
                                                     (1 << 5), 0);
        if (err != NO_ERROR) {
            ALOGE("%s: ISP[1] buffer %d allocDmabuf failed: %d", __FUNCTION__, i, err);
            return err;
        }
        // Setup shot_ext metadata in plane 1
        m_reprocIspShotExt[i] = reinterpret_cast<camera2_shot_ext*>(
            m_reprocIspBuffers[i]->planeVaddr(1));
        memset(m_reprocIspShotExt[i], 0, sizeof(camera2_shot_ext));
        m_reprocIspShotExt[i]->shot.magicNumber = SHOT_MAGIC_NUMBER;
        m_reprocIspShotExt[i]->setfile = 5; // ISS_SUB_SCENARIO_STILL_CAPTURE
        m_reprocIspShotExt[i]->drc_bypass = 1;
        m_reprocIspShotExt[i]->dis_bypass = 1;
        m_reprocIspShotExt[i]->dnr_bypass = 1;
        m_reprocIspShotExt[i]->fd_bypass = 1;
    }

    // SCC[1] buffers: YUYV packed (2 bytes/pixel) + SPARE (stream metadata)
    // SCC uses YUYV, not NV21M.  1 data plane + 1 SPARE = 2 planes.
    size_t sccPlaneSizes[2] = { (size_t)sccW * sccH * 2, G800FPipeEngine::SPARE_SIZE };
    for (int i = 0; i < NUM_SCC_REPROC_BUFFERS; i++) {
        m_reprocSccBuffers[i] = new G800FExynosCameraBuffer();
        status_t err = m_reprocSccBuffers[i]->allocDmabuf(SCC_REPROC_PLANES, sccPlaneSizes,
                                                     (1 << 5), 0);
        if (err != NO_ERROR) {
            ALOGE("%s: SCC[1] buffer %d allocDmabuf failed: %d", __FUNCTION__, i, err);
            return err;
        }
        m_reprocSccStream[i] = reinterpret_cast<camera2_stream*>(
            m_reprocSccBuffers[i]->planeVaddr(1));
        memset(m_reprocSccStream[i], 0, sizeof(camera2_stream));
    }

    ALOGI("%s: ISP[1] %dx%d + SCC[1] %dx%d buffers allocated",
          __FUNCTION__, ispW, ispH, sccW, sccH);
    return NO_ERROR;
}

void G800FPipeEngine::releaseReprocessingBuffers()
{
    for (int i = 0; i < NUM_REPROC_BUFFERS; i++) {
        if (m_reprocIspBuffers[i]) {
            m_reprocIspBuffers[i]->release();
            delete m_reprocIspBuffers[i];
            m_reprocIspBuffers[i] = NULL;
            m_reprocIspShotExt[i] = NULL;
        }
    }
    for (int i = 0; i < NUM_SCC_REPROC_BUFFERS; i++) {
        if (m_reprocSccBuffers[i]) {
            m_reprocSccBuffers[i]->release();
            delete m_reprocSccBuffers[i];
            m_reprocSccBuffers[i] = NULL;
            m_reprocSccStream[i] = NULL;
        }
    }
}

status_t G800FPipeEngine::startReprocessingPipeline()
{
    // startPipes order: SCC first, then ISP (downstream first)
    ALOGI("%s: starting [SCC>ISP]", __FUNCTION__);

    // Queue SCC[1] buffers first
    // SCC[1] uses YUYV (2 planes: YUYV data + SPARE).
    // DMABUF mode: pass dmabuf fds per plane.
    for (int i = 0; i < NUM_SCC_REPROC_BUFFERS; i++) {
        G800FExynosCameraBuffer* buf = m_reprocSccBuffers[i];
        v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(planes, 0, sizeof(planes));
        for (int p = 0; p < SCC_REPROC_PLANES; p++) {
            planes[p].m.fd = buf->dmaBufFd(p);
            planes[p].bytesused = buf->planeSize(p);
            planes[p].length    = buf->planeSize(p);
        }
        v4l2_buffer vbuf;
        memset(&vbuf, 0, sizeof(vbuf));
        vbuf.index  = i;
        vbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        vbuf.memory = V4L2_MEMORY_DMABUF;
        vbuf.length = SCC_REPROC_PLANES;
        vbuf.m.planes = planes;
        status_t err = m_reprocSccNode.qBuf(&vbuf);
        if (err != NO_ERROR) {
            ALOGE("%s: SCC[1] qBuf %d failed: %d", __FUNCTION__, i, err);
            return err;
        }
    }

    // Start SCC[1] (downstream first)
    status_t err = m_reprocSccNode.start(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
    if (err != NO_ERROR) {
        ALOGE("%s: SCC[1] start failed: %d", __FUNCTION__, err);
        return err;
    }

    // Start ISP[1]
    err = m_reprocIspNode.start(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
    if (err != NO_ERROR) {
        ALOGE("%s: ISP[1] start failed: %d", __FUNCTION__, err);
        m_reprocSccNode.stop();
        return err;
    }

    ALOGI("%s: [SCC>ISP] success", __FUNCTION__);
    return NO_ERROR;
}

status_t G800FPipeEngine::stopReprocessingPipeline()
{
    // stopPipes order: ISP stopThread → SCC stopThread → ISP stop → SCC stop
    ALOGI("%s: stopping [ISP>SCC]", __FUNCTION__);

    // ISP[1] stop (upstream first)
    if (m_reprocIspNode.isOpen()) {
        m_reprocIspNode.stop(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
    }
    // SCC[1] stop
    if (m_reprocSccNode.isOpen()) {
        m_reprocSccNode.stop(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
    }

    ALOGI("%s: [ISP>SCC] stopped", __FUNCTION__);
    return NO_ERROR;
}

// ============================================================================
// Settings application — stub for now, implemented in Phase R5
// ============================================================================
int G800FPipeEngine::applyRequestSettings(const camera_metadata_t* settings)
{
    // Resolved values written to the shot buffers — either from this
    // request's settings or (HAL3 spec: NULL settings = reuse the most
    // recent request) from the m_lastReq* cache.  Re-applying on NULL
    // requests is what restores afMode/aeMode/awbMode after
    // cancelAutoFocus() or a flash sequence left stale values in the
    // shot buffers — previously NULL requests were a no-op and the
    // stale values persisted forever.
    int32_t afMode;
    int32_t afTrigger = ANDROID_CONTROL_AF_TRIGGER_IDLE;
    // Pulse afTrigger=1 whenever the AF mode is (re)applied — treat a
    // settings-driven mode change like an implicit START so the
    // firmware kicks its scan.
    bool afKick = false;
    enum aa_aemode ispAeMode;
    enum aa_afmode ispAfMode;
    enum aa_awbmode ispAwbMode;
    enum aa_ae_flashmode ispAeFlashMode;
    enum aa_scene_mode ispSceneMode;
    bool flashSeqActive;
    int32_t aeTargetFpsRange[2];
    int32_t aeExpCompensation;
    int64_t sensorExposureTime;
    int64_t sensorFrameDuration;
    int32_t sensorSensitivity;
    int32_t lensFocusDistance;
    int32_t afRegions[5];

    if (settings == NULL) {
        afMode              = m_lastReqAfMode;
        ispAfMode           = (enum aa_afmode)m_lastReqIspAfMode;
        ispAeMode           = (enum aa_aemode)m_lastReqAeMode;
        ispAwbMode          = (enum aa_awbmode)m_lastReqAwbMode;
        ispAeFlashMode      = (enum aa_ae_flashmode)m_lastReqAeFlashMode;
        ispSceneMode        = (enum aa_scene_mode)m_lastReqSceneMode;
        aeTargetFpsRange[0] = m_lastReqAeFps[0];
        aeTargetFpsRange[1] = m_lastReqAeFps[1];
        aeExpCompensation   = m_lastReqAeExpComp;
        sensorExposureTime  = m_lastReqSensorExposure;
        sensorFrameDuration = m_lastReqSensorFrameDur;
        sensorSensitivity   = m_lastReqSensorSensitivity;
        lensFocusDistance   = m_lastReqFocusDistance;
        memcpy(afRegions, m_lastReqAfRegions, sizeof(afRegions));
        goto writeCtl;
    }

    {
    // Read Camera2 settings
    camera_metadata_ro_entry_t entry;
    int32_t aeMode = ANDROID_CONTROL_AE_MODE_ON;
    int32_t awbMode = ANDROID_CONTROL_AWB_MODE_AUTO;
    int32_t captureIntent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
    int32_t controlMode = ANDROID_CONTROL_MODE_AUTO;
    int32_t sceneMode = ANDROID_CONTROL_SCENE_MODE_FACE_PRIORITY; // default
    int32_t flashMode = ANDROID_FLASH_MODE_OFF;
    int32_t aePrecaptureTrigger = ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER_IDLE;
    int32_t aeLock = ANDROID_CONTROL_AE_LOCK_OFF;
    int32_t awbLock = ANDROID_CONTROL_AWB_LOCK_OFF;

    afMode = ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE;
    aeTargetFpsRange[0] = 15; aeTargetFpsRange[1] = 30;
    aeExpCompensation = 0;
    sensorExposureTime = 0;  // 0 = auto
    sensorFrameDuration = 0; // 0 = auto
    sensorSensitivity = 0;   // 0 = auto
    lensFocusDistance = 0;
    afRegions[0] = 0; afRegions[1] = 0; afRegions[2] = 0;
    afRegions[3] = 0; afRegions[4] = 1000;

    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AE_MODE, &entry) == OK && entry.count > 0)
        aeMode = entry.data.i32[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AF_MODE, &entry) == OK && entry.count > 0)
        afMode = entry.data.i32[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AWB_MODE, &entry) == OK && entry.count > 0)
        awbMode = entry.data.i32[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_CAPTURE_INTENT, &entry) == OK && entry.count > 0)
        captureIntent = entry.data.i32[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_MODE, &entry) == OK && entry.count > 0)
        controlMode = entry.data.i32[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_SCENE_MODE, &entry) == OK && entry.count > 0)
        sceneMode = entry.data.i32[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_FLASH_MODE, &entry) == OK && entry.count > 0)
        flashMode = entry.data.i32[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AE_TARGET_FPS_RANGE, &entry) == OK && entry.count > 0) {
        aeTargetFpsRange[0] = entry.data.i32[0];
        aeTargetFpsRange[1] = entry.data.i32[1];
    }
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION, &entry) == OK && entry.count > 0)
        aeExpCompensation = entry.data.i32[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AF_TRIGGER, &entry) == OK && entry.count > 0)
        afTrigger = entry.data.i32[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER, &entry) == OK && entry.count > 0)
        aePrecaptureTrigger = entry.data.i32[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_SENSOR_EXPOSURE_TIME, &entry) == OK && entry.count > 0)
        sensorExposureTime = entry.data.i64[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_SENSOR_FRAME_DURATION, &entry) == OK && entry.count > 0)
        sensorFrameDuration = entry.data.i64[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_SENSOR_SENSITIVITY, &entry) == OK && entry.count > 0)
        sensorSensitivity = entry.data.i32[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_LENS_FOCUS_DISTANCE, &entry) == OK && entry.count > 0)
        lensFocusDistance = (int32_t)(entry.data.f[0] * 1000); // float→int approx
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AE_LOCK, &entry) == OK && entry.count > 0)
        aeLock = entry.data.u8[0];
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AWB_LOCK, &entry) == OK && entry.count > 0)
        awbLock = entry.data.u8[0];
    // AF regions: {x1,y1,x2,y2,weight} in active-array coordinates —
    // same layout as ctl.aa.afRegions (copied 1:1 for touch-focus).
    if (find_camera_metadata_ro_entry(settings, ANDROID_CONTROL_AF_REGIONS, &entry) == OK && entry.count >= 5) {
        for (int i = 0; i < 5; i++)
            afRegions[i] = entry.data.i32[i];
    }

    // Mapping Camera2 → FIMC-IS enums
    // NOTE: The Samsung aa_aemode enum is AE-METERING (OFF/LOCKED/CENTER/
    // AVERAGE/MATRIX/SPOT), NOT Camera2 AE mode (ON/AUTO_FLASH/etc.)!
    // Camera2 AE mode is mapped to aeflashMode + sceneMode.
    // Session default is metering-mode=center → AA_AEMODE_CENTER.
    // The metering mode also steers the firmware's flash-preflash
    // metering weighting — MATRIX on the unlit periphery produced the
    // blue-cast flash shots.
    ispAeMode = AA_AEMODE_CENTER;
    if (aeMode == ANDROID_CONTROL_AE_MODE_OFF)
        ispAeMode = AA_AEMODE_OFF;
    // AE_LOCK: the flash state machine writes aeMode=LOCKED when a lock
    // is requested.  Folding it into the request-derived mode makes
    // m_lastReqAeMode carry the effective value for both preview and
    // the flash templates.
    if (aeLock == ANDROID_CONTROL_AE_LOCK_ON)
        ispAeMode = AA_AEMODE_LOCKED;

    // aa.afMode: ANDROID_CONTROL_AF_MODE_* → enum aa_afmode
    // Samsung-Enum: OFF=1, SLEEP=2, INFINITY=3, MACRO=4, ..., AUTO=11, ...,
    //   CONTINUOUS_PICTURE=21, CONTINUOUS_VIDEO=22, ...
    ispAfMode = AA_AFMODE_CONTINUOUS_PICTURE;
    switch (afMode) {
        case ANDROID_CONTROL_AF_MODE_OFF:                    ispAfMode = AA_AFMODE_OFF; break;
        case ANDROID_CONTROL_AF_MODE_AUTO:                   ispAfMode = AA_AFMODE_AUTO; break;
        case ANDROID_CONTROL_AF_MODE_MACRO:                  ispAfMode = AA_AFMODE_MACRO; break;
        case ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE:     ispAfMode = AA_AFMODE_CONTINUOUS_PICTURE; break;
        case ANDROID_CONTROL_AF_MODE_CONTINUOUS_VIDEO:       ispAfMode = AA_AFMODE_CONTINUOUS_VIDEO; break;
        default: ispAfMode = AA_AFMODE_CONTINUOUS_PICTURE; break;
    }

    // aa.awbMode: ANDROID_CONTROL_AWB_MODE_* → enum aa_awbmode
    // Samsung-Enum: OFF=1, LOCKED=2, WB_AUTO=3, WB_INCANDESCENT=4, ...
    ispAwbMode = AA_AWBMODE_WB_AUTO;
    switch (awbMode) {
        case ANDROID_CONTROL_AWB_MODE_OFF:              ispAwbMode = AA_AWBMODE_OFF; break;
        case ANDROID_CONTROL_AWB_MODE_AUTO:             ispAwbMode = AA_AWBMODE_WB_AUTO; break;
        case ANDROID_CONTROL_AWB_MODE_INCANDESCENT:     ispAwbMode = AA_AWBMODE_WB_INCANDESCENT; break;
        case ANDROID_CONTROL_AWB_MODE_FLUORESCENT:      ispAwbMode = AA_AWBMODE_WB_FLUORESCENT; break;
        case ANDROID_CONTROL_AWB_MODE_WARM_FLUORESCENT: ispAwbMode = AA_AWBMODE_WB_WARM_FLUORESCENT; break;
        case ANDROID_CONTROL_AWB_MODE_DAYLIGHT:         ispAwbMode = AA_AWBMODE_WB_DAYLIGHT; break;
        case ANDROID_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT:  ispAwbMode = AA_AWBMODE_WB_CLOUDY_DAYLIGHT; break;
        case ANDROID_CONTROL_AWB_MODE_TWILIGHT:         ispAwbMode = AA_AWBMODE_WB_TWILIGHT; break;
        case ANDROID_CONTROL_AWB_MODE_SHADE:            ispAwbMode = AA_AWBMODE_WB_SHADE; break;
        default: ispAwbMode = AA_AWBMODE_WB_AUTO; break;
    }
    // AWB_LOCK — same fold as AE_LOCK (lock → awbMode=LOCKED).
    if (awbLock == ANDROID_CONTROL_AWB_LOCK_ON)
        ispAwbMode = AA_AWBMODE_LOCKED;

    // aa.aeflashMode: Samsung enum AA_FLASHMODE_*
    // OFF=1, START=2, CANCEL=3, ON=4, AUTO=5, CAPTURE=6, ON_ALWAYS=7
    //
    // aeflashMode is written PER FRAME by the flash state machine,
    //   NOT directly from the Camera2 request. During normal preview,
    //   aeflashMode=1 (OFF), even if flash mode is AUTO.
    //   The firmware interprets aeflashMode=5 (AUTO) as "fire flash if
    //   needed" — if we write that into every frame, the LED
    //   fires continuously!
    //
    // Camera2 AE mode is only stored as a flag (for the flash state machine).
    // In shot_ext we ALWAYS write aeflashMode=1 (OFF) for preview.
    // The flash state machine (beginFlashSequence/advanceFlashSequence) sets
    // aeflashMode=START/ON/CAPTURE/OFF via setAeFlashMode() separately.
    //
    // If the flash sequence is active, we must NOT overwrite aeflashMode
    // (otherwise we would kill the state machine values).
    ispAeFlashMode = AA_FLASHMODE_OFF;  // OFF for preview by default

    // Video torch (continuous light): keep the LED lit while the
    // request asks for it — flashMode=TORCH or AE_MODE_ON_ALWAYS_FLASH.
    // ON_ALWAYS_FLASH means "continuous light" per spec — also on still
    // requests (LED stays lit during the shot, no strobe; the still must
    // not write OFF here or the light dies mid-capture).
    // Torch tuple:
    //   aeflashMode=ON_ALWAYS(7), flashMode=TORCH(3), firingTime=50000, 0x3f.
    // The matching ctl.flash.* tuple is written in the shot loop below, keyed
    // on ispAeFlashMode==ON_ALWAYS so NULL-settings requests keep torch on.
    bool torchWanted = (m_cameraId == 0) &&
            (flashMode == ANDROID_FLASH_MODE_TORCH ||
             aeMode == ANDROID_CONTROL_AE_MODE_ON_ALWAYS_FLASH);
    if (torchWanted)
        ispAeFlashMode = AA_FLASHMODE_ON_ALWAYS;

    // aeMode/flashMode additionally gate the still flash sequence in the
    // caller (autoFlash/singleFlash) and the torch request above.

    // aa.sceneMode: Samsung enum AA_SCENE_MODE_*
    // PREVIEW=27 for normal preview, FACE_PRIORITY=2 as Camera2 default
    ispSceneMode = AA_SCENE_MODE_PREVIEW;
    if (captureIntent == ANDROID_CONTROL_CAPTURE_INTENT_STILL_CAPTURE)
        ispSceneMode = AA_SCENE_MODE_PREVIEW; // TODO: check scene mode for still capture
    // TODO: complete scene-mode mapping

    // afTrigger semantics: afTrigger=1 is a ONE-FRAME pulse — written
    // only in the single "mode applied / scan start" frame, then 0.
    // We therefore write START→1, everything else→0.  The pulse ends
    // automatically on the next applyRequestSettings() call (settings
    // or NULL).
    //
    // (The previous "leave afTrigger untouched on IDLE" kept the
    // init-time trigger=1 forever — a permanently-held trigger is not
    // what the firmware expects and can restart the AF scan every
    // frame.)
    //
    // Cache the resolved values so NULL-settings requests (repeating
    // requests per the HAL3 spec) re-apply the identical block —
    // including restoring afMode after cancelAutoFocus() wrote OFF.
    afKick = (afMode != m_lastReqAfMode);
    m_lastReqAfMode       = afMode;
    m_lastReqIspAfMode    = ispAfMode;
    m_lastReqAeMode       = ispAeMode;
    m_lastReqAwbMode      = ispAwbMode;
    m_lastReqAeFlashMode  = ispAeFlashMode;
    m_lastReqSceneMode    = ispSceneMode;
    m_lastReqAeFps[0]     = aeTargetFpsRange[0];
    m_lastReqAeFps[1]     = aeTargetFpsRange[1];
    m_lastReqAeExpComp    = aeExpCompensation;
    m_lastReqSensorExposure  = sensorExposureTime;
    m_lastReqSensorFrameDur  = sensorFrameDuration;
    m_lastReqSensorSensitivity = sensorSensitivity;
    m_lastReqFocusDistance   = lensFocusDistance;
    memcpy(m_lastReqAfRegions, afRegions, sizeof(afRegions));

    // Sparse: only fires on settings-bearing requests (repeating
    // requests arrive with settings==NULL and take the cached path).
    /* per-request log disabled.
    ALOGI("HAL3-REQ afMode=%d→%d afTrigger=%d kick=%d "
          "afRegions=%d,%d,%d,%d,%d aeMode=%d aeLock=%d awbMode=%d "
          "intent=%d ctlMode=%d flash=%d",
          afMode, (int)ispAfMode, afTrigger, (int)afKick,
          afRegions[0], afRegions[1], afRegions[2], afRegions[3],
          afRegions[4], aeMode, aeLock, awbMode,
          captureIntent, controlMode, flashMode);
    */
    }  // settings != NULL

writeCtl:
    flashSeqActive = isFlashSequenceActive();

    // Session-start kick (set by startPreview): afTrigger=1 pulses on
    // every AF-mode apply — including the session-start apply.  Consumed
    // here so it fires on the first apply regardless of the request path
    // (settings or NULL) and regardless of mode equality with the cache.
    if (m_afKickPending) {
        afKick = true;
        m_afKickPending = false;
    }

    // Write into all FLITE shot_ext buffers
    // (FLITE is the leader — the firmware reads the ctl fields from the FLITE buffer)
    for (int i = 0; i < NUM_FLITE_BUFFERS; i++) {
        if (m_fliteShotExt[i] == NULL) continue;
        camera2_shot_ext* shot = m_fliteShotExt[i];
        shot->shot.ctl.aa.afMode = ispAfMode;
        // aeMode/awbMode/aeflashMode: only write if the flash state machine
        // is INACTIVE.  While it runs (START/ON/METERING/AF_WAIT/CAPTURE/...),
        // writeFlashSeqMetadata() owns the full tuple — overwriting aeMode
        // with the request value while aeflashMode=ON+firing is set is what
        // crashed the ISP firmware (noiseIndDenum assert → kernel panic).
        // afMode stays writable: AF must keep scanning during AF_WAIT.
        if (!flashSeqActive) {
            shot->shot.ctl.aa.aeMode = ispAeMode;
            shot->shot.ctl.aa.awbMode = ispAwbMode;
            shot->shot.ctl.aa.aeflashMode = ispAeFlashMode;
            // ctl.flash tuple — the firmware drives the FLED from these per
            // frame (isp_peri_ctl.flashUd).  For aeflashMode=ON_ALWAYS (video
            // torch) write the torch tuple so the LED stays lit;
            // otherwise OFF so a torch→off transition actually turns it off.
            if (ispAeFlashMode == AA_FLASHMODE_ON_ALWAYS) {
                shot->shot.ctl.flash.flashMode   = CAM2_FLASH_MODE_TORCH;
                shot->shot.ctl.flash.firingTime  = 50000;
                shot->shot.ctl.flash.firingPower = 0x3f;
            } else {
                shot->shot.ctl.flash.flashMode   = CAM2_FLASH_MODE_OFF;
                shot->shot.ctl.flash.firingTime  = 0;
                shot->shot.ctl.flash.firingPower = 0;
            }
        }
        shot->shot.ctl.aa.sceneMode = ispSceneMode;
        shot->shot.ctl.aa.aeTargetFpsRange[0] = aeTargetFpsRange[0];
        shot->shot.ctl.aa.aeTargetFpsRange[1] = aeTargetFpsRange[1];
        // Exposure compensation: the firmware reads aeExpCompensation
        // from the shot_ext metadata buffer (plane 1), NOT via
        // V4L2_CID_IS_CAMERA_EXPOSURE s_ctrl.
        // Internal range: 1..9 (for Camera2 -4..+4), default 5.
        shot->shot.ctl.aa.aeExpCompensation = aeExpCompensation + 5;
        // afTrigger is a one-frame pulse (see comment above): START or a
        // mode change→1, otherwise 0.  The next apply (settings or
        // NULL-settings replay) returns it to 0.
        shot->shot.ctl.aa.afTrigger =
            (afTrigger == ANDROID_CONTROL_AF_TRIGGER_START || afKick) ? 1 : 0;
        for (int r = 0; r < 5; r++)
            shot->shot.ctl.aa.afRegions[r] = afRegions[r];
        shot->shot.ctl.sensor.exposureTime = sensorExposureTime;
        shot->shot.ctl.sensor.frameDuration = sensorFrameDuration;
        shot->shot.ctl.sensor.sensitivity = sensorSensitivity;
        shot->shot.ctl.lens.focusDistance = lensFocusDistance;
    }

    // Mirror the AF control tuple into the ISP shot buffers too —
    // cancelAutoFocus() clears them to OFF; without re-applying here,
    // they would stay OFF forever (init values are only written once).
    for (int i = 0; i < NUM_ISP_BUFFERS; i++) {
        camera2_shot_ext* shot = m_ispShotExt[i];
        if (shot == NULL) continue;
        shot->shot.ctl.aa.afMode = ispAfMode;
        shot->shot.ctl.aa.afTrigger =
            (afTrigger == ANDROID_CONTROL_AF_TRIGGER_START || afKick) ? 1 : 0;
        for (int r = 0; r < 5; r++)
            shot->shot.ctl.aa.afRegions[r] = afRegions[r];
    }

    // Mirror into the parameter control block (m_paramsCtl).
    // Unlike the per-frame shots this is ALWAYS request-driven — the
    // flash state machine writes only per-frame shot ctl, never the
    // parameters block.  The reproc path copies exactly this block into
    // the reproc shot.
    m_paramsCtl.aa.afMode = ispAfMode;
    m_paramsCtl.aa.aeMode = ispAeMode;
    m_paramsCtl.aa.awbMode = ispAwbMode;
    m_paramsCtl.aa.aeflashMode = ispAeFlashMode;
    if (ispAeFlashMode == AA_FLASHMODE_ON_ALWAYS) {
        m_paramsCtl.flash.flashMode   = CAM2_FLASH_MODE_TORCH;
        m_paramsCtl.flash.firingTime  = 50000;
        m_paramsCtl.flash.firingPower = 0x3f;
    } else {
        m_paramsCtl.flash.flashMode   = CAM2_FLASH_MODE_OFF;
        m_paramsCtl.flash.firingTime  = 0;
        m_paramsCtl.flash.firingPower = 0;
    }
    m_paramsCtl.aa.sceneMode = ispSceneMode;
    m_paramsCtl.aa.aeTargetFpsRange[0] = aeTargetFpsRange[0];
    m_paramsCtl.aa.aeTargetFpsRange[1] = aeTargetFpsRange[1];
    m_paramsCtl.aa.aeExpCompensation = aeExpCompensation + 5;
    m_paramsCtl.aa.afTrigger =
        (afTrigger == ANDROID_CONTROL_AF_TRIGGER_START) ? 1 : 0;
    for (int r = 0; r < 5; r++)
        m_paramsCtl.aa.afRegions[r] = afRegions[r];
    m_paramsCtl.sensor.exposureTime = sensorExposureTime;
    m_paramsCtl.sensor.frameDuration = sensorFrameDuration;
    m_paramsCtl.sensor.sensitivity = sensorSensitivity;
    m_paramsCtl.lens.focusDistance = lensFocusDistance;

    // Exposure compensation is controlled exclusively via shot_ext metadata
    // (see above, aeExpCompensation = ev + 5).  The old V4L2_CID_IS_CAMERA_EXPOSURE
    // s_ctrl path was removed because:
    //   1. The active fimc-is-mc2 driver does not implement V4L2_CID_IS_CAMERA_EXPOSURE
    //      on the ISP/SCP/SCC nodes (s_ctrl returns -EINVAL).
    //   2. Exposure compensation is carried exclusively in shot_ext
    //      metadata (ev + 5), NO s_ctrl.
    //   3. The firmware reads aeExpCompensation from the shot_ext buffer (plane 1).

    // Debug: what HAL3 requested
    //ALOGI("%s: HAL3-REQ aeMode=%d afMode=%d awbMode=%d flashMode=%d "
    //      "intent=%d controlMode=%d sceneMode=%d aeFps=[%d,%d] "
    //      "aeExpComp=%d(internal=%d) afTrigger=%d(apply=%d) aePrecap=%d | "
    //      "sensor: expTime=%lldns frameDur=%lldns sens=%d focusDist=%d | "
    //      "→ isp: aeMode=%d afMode=%d awbMode=%d aeFlash=%d(flashSeq=%d) scene=%d",
    //      __FUNCTION__,
    //      aeMode, afMode, awbMode, flashMode,
    //      captureIntent, controlMode, sceneMode,
    //      aeTargetFpsRange[0], aeTargetFpsRange[1],
    //      aeExpCompensation, aeExpCompensation + 5,
    //      afTrigger, (int)applyAfTrigger, aePrecaptureTrigger,
    //      (long long)sensorExposureTime, (long long)sensorFrameDuration,
    //      sensorSensitivity, lensFocusDistance,
    //      ispAeMode, ispAfMode, ispAwbMode, ispAeFlashMode, (int)flashSeqActive,
    //      ispSceneMode);

    return 0;
}

// ============================================================================
// cancelAutoFocus — resets AF in all shot_ext buffers.
// Cancel AF: park afMode at OFF and clear the trigger.
// Called on flush() and close().
// ============================================================================
void G800FPipeEngine::cancelAutoFocus()
{
    ALOGI("%s: cancelling AF (afMode=OFF, afTrigger=0)", __FUNCTION__);

    // Re-arm the session kick: the next applyRequestSettings re-writes the
    // request afMode — an afTrigger=1 pulse on every such apply
    // (START step), which is what restarts the firmware's scan after a
    // cancel parked it at afMode=OFF.
    m_afKickPending = (m_cameraId == 0);

    // Safety: only access if pipes still exist (not after deinit)
    if (m_flitePipe == NULL && m_ispPipe == NULL) {
        ALOGW("%s: pipes already destroyed, skipping", __FUNCTION__);
        return;
    }
    for (int i = 0; i < NUM_FLITE_BUFFERS; i++) {
        if (m_fliteShotExt[i] != NULL) {
            m_fliteShotExt[i]->shot.ctl.aa.afMode = AA_AFMODE_OFF;
            m_fliteShotExt[i]->shot.ctl.aa.afTrigger = 0;
        }
    }
    for (int i = 0; i < NUM_ISP_BUFFERS; i++) {
        if (m_ispShotExt[i] != NULL) {
            m_ispShotExt[i]->shot.ctl.aa.afMode = AA_AFMODE_OFF;
            m_ispShotExt[i]->shot.ctl.aa.afTrigger = 0;
        }
    }
}

// ============================================================================
//  reprocThreadLoop — asynchronous reprocessing worker
//
//  Gets Bayer frames from m_bayerForReprocQ (from FLITE pipe via FrameSelector),
//  processes them through ISP_REPROC → SCC_REPROC, and pushes the result
//  into m_reprocOutputQ.
//
//  The thread runs as long as m_reprocThreadRunning == true.
// ============================================================================

bool G800FPipeEngine::reprocThreadLoop()
{
    if (!m_reprocThreadRunning) return false;

    // Wait for Bayer frame from FLITE pipe (via FrameSelector)
    G800FFrame* bayerFrame = NULL;
    status_t err = m_bayerForReprocQ.pop_front(&bayerFrame, 1000);
    if (err != NO_ERROR) {
        if (err == -ETIMEDOUT) return m_reprocThreadRunning;  // keep waiting
        ALOGE("%s: pop_front failed: %d", __FUNCTION__, err);
        return m_reprocThreadRunning;
    }
    if (!bayerFrame) return m_reprocThreadRunning;

    int fliteIdx = bayerFrame->bufferIndex;
    ALOGI("%s: processing Bayer frame (fliteIdx=%d, fcount=%u)",
          __FUNCTION__, fliteIdx, bayerFrame->frameCount);

    // Bugfix for OS freeze: as soon as the reproc thread has a frame,
    // cancel capture redirect and return remaining frames in m_bayerForReprocQ
    // to FLITE.  Without this, all 8 FLITE buffers are consumed by the
    // FrameSelector → FLITE has no free buffers →
    // DQBUF EINVAL → busy loop → OS freeze.
    if (m_flitePipe) m_flitePipe->clearCaptureRedirect();
    if (m_ispPipe) m_ispPipe->clearFlashCaptureRedirect();
    // Return remaining frames in the queue to FLITE
    G800FFrame* leftover = NULL;
    while (m_bayerForReprocQ.try_pop_front(&leftover) == NO_ERROR && leftover) {
        int li = leftover->bufferIndex;
        m_flitePipe->clearFliteInUse(li);
        m_flitePipe->requeueBuffer(li);
        delete leftover;
    }
    // Release FrameSelector candidates
    if (m_frameSelector) m_frameSelector->cancel(NULL);

    // Feed ISP[1] with FLITE Bayer
    camera2_shot_ext* fliteShot = m_flitePipe->getShotExt(fliteIdx);
    // Reproc buffer index == fliteIdx: the ISP[1] qbuf passes the FLITE
    // buffer's own dmaBufFd for plane 0 — with a fixed index↔address pair
    // the kernel's stable-DMA-address check (fimc-is-video.c "buffer %d
    // plane %d is changed") is satisfied and NO memcpy is needed.
    // (The 283 ms CPU copy also made the reprocess arrive ~300 ms late;
    // the firmware may have evicted the per-frame flash context by then.)
    if (fliteIdx < 0 || fliteIdx >= NUM_REPROC_BUFFERS) {
        ALOGE("%s: fliteIdx %d out of reproc range", __FUNCTION__, fliteIdx);
        delete bayerFrame;
        return m_reprocThreadRunning;
    }
    camera2_shot_ext* ispShot = m_reprocIspShotExt[fliteIdx];

    if (fliteShot && ispShot) {
        // Fetch THIS frame's firmware results (dm + udm — computed WB
        // gains, exposure, flash metering).  The 3AA output metadata
        // travels in plane 1 of the bayer buffer itself — the buffer is
        // held by us (not requeued) so its meta is still valid.
        // The ISP[0] ring is only a fallback (its udm is the ISP output
        // view, which may differ from the 3AA/bayer meta — notably the
        // flash-metered AWB gains live in the bayer frame's udm).
        camera2_dm frameDm;
        camera2_udm frameUdm;
        camera2_uctl frameUctl;
        bool metaFresh = false;
        if (m_ispPipe) {
            for (int i = 0; i < 40 && !metaFresh; i++) {
                metaFresh = m_ispPipe->copyFrameMeta(
                    (int32_t)bayerFrame->frameCount,
                    &frameDm, &frameUctl, &frameUdm);
                if (!metaFresh) usleep(5000);
            }
            if (!metaFresh)
                ALOGW("%s: no fresh firmware meta for frame %u — reproc "
                      "metadata may be wrong", __FUNCTION__,
                      bayerFrame->frameCount);
        }
        /* diagnostic meta-compare log disabled.
        {
            const uint32_t* gf = bayerFrame->sensorUdm.awb.vendorSpecific;
            const uint32_t* gr = frameUdm.awb.vendorSpecific;
            ALOGI("%s: AWB-CMP fcount=%u sens(vsLen=%u) g[0..7]=%u %u %u %u "
                  "%u %u %u %u | ring(vsLen=%u) g[0..7]=%u %u %u %u %u %u %u %u",
                  __FUNCTION__, bayerFrame->frameCount,
                  bayerFrame->sensorUdm.awb.vsLength,
                  gf[0], gf[1], gf[2], gf[3], gf[4], gf[5], gf[6], gf[7],
                  metaFresh ? frameUdm.awb.vsLength : 0,
                  gr[0], gr[1], gr[2], gr[3], gr[4], gr[5], gr[6], gr[7]);
            // same for dm.flash — does the sensor dm mark the flash-lit
            // frame (aeflashMode=CAPTURE echo)?
            ALOGI("%s: DM-CMP fcount=%u sensFcount=%u idx=%d sens[aeFlash=%d firingStable=%d "
                  "flashMode=%d decision=%d aeMode=%d awbMode=%d exp=%lld "
                  "iso=%u] ring[fresh=%d aeFlash=%d firingStable=%d]",
                  __FUNCTION__, bayerFrame->frameCount,
                  bayerFrame->sensorDm.request.frameCount,
                  bayerFrame->bufferIndex,
                  bayerFrame->sensorDm.aa.aeflashMode,
                  bayerFrame->sensorDm.flash.firingStable,
                  bayerFrame->sensorDm.flash.flashMode,
                  bayerFrame->sensorDm.flash.decision,
                  bayerFrame->sensorDm.aa.aeMode,
                  bayerFrame->sensorDm.aa.awbMode,
                  (long long)bayerFrame->sensorDm.sensor.exposureTime,
                  bayerFrame->sensorDm.sensor.sensitivity,
                  (int)metaFresh,
                  frameDm.aa.aeflashMode, frameDm.flash.firingStable);
        }
        */
        // ctl comes from the parameters block (request-derived values,
        // never touched by the flash state machine) — NOT from the
        // captured frame's shot, whose ctl still carries flash-sequence
        // residue (LOCKED ae/awb, CAPTURE mode, firing tuple).
        ispShot->shot.ctl = m_paramsCtl;
        // The params-ctl block carries the FIXED still-capture context,
        // not the app's per-request preview scene.  ISP[1] reads only
        // ctl/uctl/node_group on input — dm/udm are output and ignored.
        // Defaults (the 3A-mode fields aeMode/awbMode/afMode stay
        // request-derived):
        //   captureIntent CUSTOM(0), mode AUTO(2), sceneMode FACE_PRIORITY(2)
        //   isoMode AUTO(1)   — 0 selects the preview-tuned ISP path
        //   rather than the still path.
        //   stats.* : the enums are 1-based — OFF=1, 0 is out-of-range.
        ispShot->shot.ctl.aa.captureIntent        = AA_CAPTURE_INTENT_CUSTOM;
        ispShot->shot.ctl.aa.mode                 = AA_CONTROL_AUTO;
        ispShot->shot.ctl.aa.sceneMode            = AA_SCENE_MODE_FACE_PRIORITY;
        ispShot->shot.ctl.aa.isoMode              = AA_ISOMODE_AUTO;
        ispShot->shot.ctl.stats.faceDetectMode    = FACEDETECT_MODE_OFF;
        ispShot->shot.ctl.stats.histogramMode     = STATS_MODE_OFF;
        ispShot->shot.ctl.stats.sharpnessMapMode  = STATS_MODE_OFF;
        // The reproc shot deliberately carries NO flash-strobe tuple — the
        // flash metering results travel in dm/udm.  Declaring the frame a
        // flash capture to ISP[1] (aeflash=CAPTURE + SINGLE + 500µs) made the
        // ISP apply flash defaults → green (the main_fire tuple belongs to
        // the preview/capture shots only).
        if (metaFresh) {
            // Primary source: the ISP[0] output meta ring for THIS frame.
            // Its dm carries the firmware's real applied-echo for this exact
            // frame (aeFlash=6/firingStable on the lit frame), and its udm
            // holds the per-frame AWB measurement (vsLen=1600) — on the lit
            // frame these are the flash-metered values the ISP[1] needs to
            // reproduce the flash WB.  The FLITE sensor meta (sensorDm) lags
            // the exposure by several frames and carries udm.vsLength=0.
            ispShot->shot.dm   = frameDm;
            ispShot->shot.udm  = frameUdm;
        } else if (bayerFrame->hasSensorMeta) {
            // Fallback: the bayer frame's own sensor-node dm/udm
            // (snapshotted at FLITE dequeue before the ISP dequeue overwrote
            // the meta plane).
            ispShot->shot.dm  = bayerFrame->sensorDm;
            ispShot->shot.udm = bayerFrame->sensorUdm;
        } else {
            memcpy(&ispShot->shot.dm, &fliteShot->shot.dm,
                   sizeof(fliteShot->shot.dm));
        }
        // uctl: the new frame's own uctl goes here (zeroed template).
        // The capture's uctl holds the firmware's post-flash "next frame"
        // plan (sensorUd/flashUd) and must not leak into this request.
        memset(&ispShot->shot.uctl, 0, sizeof(ispShot->shot.uctl));
        memcpy(&ispShot->node_group, &fliteShot->node_group, sizeof(fliteShot->node_group));
        /* per-frame diagnostic logs disabled.
        // Diagnostic: was the selected frame really the main-flash frame?
        // (firingStable=1 / aeFlash=6 in dm → flash-lit; exp/iso show the
        // sensor settings the firmware applied for that frame)
        ALOGI("%s: REPROC-DM fcount=%u fresh=%d: ctl[aeMode=%d awbMode=%d aeFlash=%d "
              "flashMode=%d firing=%llu/%u] dm[aeState=%d awbState=%d afState=%d "
              "aeMode=%d awbMode=%d aeFlash=%d flashMode=%d decision=%d "
              "firingStable=%d exp=%lldns iso=%u]",
              __FUNCTION__, bayerFrame->frameCount, (int)metaFresh,
              ispShot->shot.ctl.aa.aeMode, ispShot->shot.ctl.aa.awbMode,
              ispShot->shot.ctl.aa.aeflashMode,
              ispShot->shot.ctl.flash.flashMode,
              (unsigned long long)ispShot->shot.ctl.flash.firingTime,
              ispShot->shot.ctl.flash.firingPower,
              ispShot->shot.dm.aa.aeState, ispShot->shot.dm.aa.awbState,
              ispShot->shot.dm.aa.afState, ispShot->shot.dm.aa.aeMode,
              ispShot->shot.dm.aa.awbMode, ispShot->shot.dm.aa.aeflashMode,
              ispShot->shot.dm.flash.flashMode, ispShot->shot.dm.flash.decision,
              ispShot->shot.dm.flash.firingStable,
              (long long)ispShot->shot.dm.sensor.exposureTime,
              ispShot->shot.dm.sensor.sensitivity);
        if (metaFresh) {
            // Firmware-applied parameters for THIS frame (uctl = the
            // "what the ISP actually used" channel): flash strobe and
            // sensor exposure/ISO as applied at capture time.
            ALOGI("%s: REPROC-UCTL fcount=%u: flash[fMode=%d fT=%llu fP=%u] "
                  "sensor[exp=%lld iso=%u frameDur=%lld] "
                  "uUpdateBitMap=0x%08x",
                  __FUNCTION__, bayerFrame->frameCount,
                  frameUctl.flashUd.ctl.flashMode,
                  (unsigned long long)frameUctl.flashUd.ctl.firingTime,
                  frameUctl.flashUd.ctl.firingPower,
                  (unsigned long long)frameUctl.sensorUd.ctl.exposureTime,
                  frameUctl.sensorUd.ctl.sensitivity,
                  (unsigned long long)frameUctl.sensorUd.ctl.frameDuration,
                  frameUctl.uUpdateBitMap);
        }
        // Firmware-computed AWB gains (locked at preflash metering) from
        // udm.awb.vendorSpecific — logs the per-capture gain set so the
        // convergence between captures is directly observable.
        {
            const uint32_t* g = ispShot->shot.udm.awb.vendorSpecific;
            ALOGI("%s: REPROC-AWBG fcount=%u: vsLen=%u g[0..7]=%u %u %u %u %u %u %u %u",
                  __FUNCTION__, bayerFrame->frameCount,
                  ispShot->shot.udm.awb.vsLength,
                  g[0], g[1], g[2], g[3], g[4], g[5], g[6], g[7]);
            // Non-zero scan: which words of the 400-word awb blob did the
            // firmware actually fill?  If only a few words are set, the
            // real WB data may live elsewhere (or not be written at all).
            int nz = 0, first = -1, last = -1;
            for (int i = 0; i < 400; i++) {
                if (g[i]) { nz++; if (first < 0) first = i; last = i; }
            }
            ALOGI("%s: REPROC-AWBNZ fcount=%u: nz=%d first=%d last=%d "
                  "g[8..19]=%u %u %u %u %u %u %u %u %u %u %u %u",
                  __FUNCTION__, bayerFrame->frameCount, nz, first, last,
                  g[8], g[9], g[10], g[11], g[12], g[13],
                  g[14], g[15], g[16], g[17], g[18], g[19]);
        }
        {
            // EXIF reads the real metering results from
            // internal.vendorSpecific2[100..103] (cml exposure/iso/Bv).
            const uint32_t* v2 = ispShot->shot.udm.internal.vendorSpecific2;
            ALOGI("%s: REPROC-UDM2 fcount=%u: v2[0]=%u v2[100..103]=%u %u %u %u "
                  "v2[1..4]=%u %u %u %u",
                  __FUNCTION__, bayerFrame->frameCount,
                  v2[0], v2[100], v2[101], v2[102], v2[103],
                  v2[1], v2[2], v2[3], v2[4]);
        }
        */
        // Reprocessing setfile (5 = STILL_CAPTURE)
        ispShot->setfile = 5;

        // The reproc shot gets ctl = params ctl (already applied above
        // via m_paramsCtl), dm+udm from the captured frame (the meta
        // ring), fresh uctl — and NO flash override.  The flash metering
        // results travel in udm; the main_fire tuple belongs to the
        // preview/capture shots only.  (Writing ON+firing here made the
        // ISP apply defaults → green.)
        int flashState = getFlashSeqState();
        if (flashState == FLASH_SEQ_READY ||
            flashState == FLASH_SEQ_CAPTURE ||
            flashState == FLASH_SEQ_MAIN_WAIT ||
            flashState == FLASH_SEQ_DONE) {
            ALOGI("%s: Flash reprocessing (flashSeqState=%d) — params ctl",
                  __FUNCTION__, flashState);
        }
        // Node group for reprocessing:
        // leader=ISP, capture[0]=SCC, capture[1]=SCP must be DISABLED!
        // IMPORTANT: crop must be SMALLER than sensor, otherwise kernel error.
        // Reprocessing crop uses bayerCropSize.
        int reprocSccW = (m_cameraId == 0) ? SCC_W : SCC_FRONT_W;
        int reprocSccH = (m_cameraId == 0) ? SCC_H : SCC_FRONT_H;
        ispShot->node_group.leader.request = 1;
        ispShot->node_group.leader.vid = 30; // ISP
        ispShot->node_group.leader.input.cropRegion[0] = 0;
        ispShot->node_group.leader.input.cropRegion[1] = 0;
        ispShot->node_group.leader.input.cropRegion[2] = reprocSccW;
        ispShot->node_group.leader.input.cropRegion[3] = reprocSccH;
        ispShot->node_group.leader.output.cropRegion[0] = 0;
        ispShot->node_group.leader.output.cropRegion[1] = 0;
        ispShot->node_group.leader.output.cropRegion[2] = reprocSccW;
        ispShot->node_group.leader.output.cropRegion[3] = reprocSccH;
        // capture[0] = SCC (full-res YUV)
        ispShot->node_group.capture[0].request = 1;
        ispShot->node_group.capture[0].vid = 34; // SCC
        ispShot->node_group.capture[0].input.cropRegion[0] = 0;
        ispShot->node_group.capture[0].input.cropRegion[1] = 0;
        ispShot->node_group.capture[0].input.cropRegion[2] = reprocSccW;
        ispShot->node_group.capture[0].input.cropRegion[3] = reprocSccH;
        ispShot->node_group.capture[0].output.cropRegion[0] = 0;
        ispShot->node_group.capture[0].output.cropRegion[1] = 0;
        ispShot->node_group.capture[0].output.cropRegion[2] = reprocSccW;
        ispShot->node_group.capture[0].output.cropRegion[3] = reprocSccH;
        // capture[1] = SCP must be DISABLED — reprocessing has no SCP!
        // If request stays 1, the firmware tries SCP output →
        // "framemgr is NULL" → "isp shot is skipped" → no SCC output.
        ispShot->node_group.capture[1].request = 0;
        ispShot->node_group.capture[1].vid = 0;
    }

    // ISP[1] qBuf: DMABUF, buffer index = fliteIdx (stable address pair).
    // plane 0 = the FLITE buffer's own dmaBufFd (the Bayer data, NO copy —
    //   the preview path qBufIsp does the same for ISP[0]).  The flite
    //   buffer stays pinned via fliteInUse until after ISP[1] dqbuf.
    // plane 1 = shot_ext metadata from our own ION buffer (same index).
    G800FExynosCameraBuffer* fliteBuf = m_flitePipe->getBuffer(fliteIdx);
    G800FExynosCameraBuffer* ispBuf = m_reprocIspBuffers[fliteIdx];
    if (!fliteBuf || !ispBuf || fliteBuf->dmaBufFd(0) < 0) {
        ALOGE("%s: bad flite/isp buffer (fliteIdx=%d)", __FUNCTION__, fliteIdx);
        m_flitePipe->clearFliteInUse(fliteIdx);
        m_flitePipe->requeueBuffer(fliteIdx);
        delete bayerFrame;
        return m_reprocThreadRunning;
    }
    size_t bayerSize = fliteBuf->planeSize(0);
    // Sanity: a stale/mismatched frame (e.g. left in m_bayerForReprocQ across
    // a stream reconfigure) must not be pushed through ISP[1].  The Bayer
    // plane is exactly roundup(W,10)*8/5*H — anything else means the frame's
    // bufferIndex does not refer to a live full-res FLITE buffer.
    {
        int fw = getFliteWidth(), fh = getFliteHeight();
        size_t expectedBayer = (size_t)((fw + 9) / 10 * 10) * 8 / 5 * fh;
        if (bayerSize != expectedBayer) {
            ALOGE("%s: fliteIdx=%d bayerSize=%zu != expected %zu — skipping "
                  "stale frame", __FUNCTION__, fliteIdx, bayerSize,
                  expectedBayer);
            m_flitePipe->clearFliteInUse(fliteIdx);
            m_flitePipe->requeueBuffer(fliteIdx);
            delete bayerFrame;
            return m_reprocThreadRunning;
        }
    }

    // NOTE: the FLITE pixel plane (fliteBuf->planeVaddr(0)) is NOT safe to
    // dereference in this path — for a DMABUF buffer the CPU-side mapping is
    // not guaranteed valid here (crash seen: G800FReproc SIGSEGV).  The real
    // reprocess path only uses dmaBufFd (kernel-side DMA), never the pixel
    // vaddr.  Metadata access below uses ispShot/fliteShot (meta plane).
    if (!ispShot) {
        ALOGE("%s: ispShot is NULL (fliteIdx=%d)", __FUNCTION__, fliteIdx);
        m_flitePipe->clearFliteInUse(fliteIdx);
        m_flitePipe->requeueBuffer(fliteIdx);
        delete bayerFrame;
        return m_reprocThreadRunning;
    }

    // Single pass: feed the lit Bayer once through ISP[1].  (An earlier
    // experiment replayed the same frame N times hoping the cold-ISP 3A
    // would converge on the flash illumination — it cannot reproduce the
    // preview ISP's flash-weighted AWB, so a single pass is sufficient.)
    const int replayN = 1;

    // Snapshot the input portion of the shot: the firmware overwrites
    // dm/udm (output direction) on each pass, so the ctl/uctl/node_group/
    // setfile inputs must be restored before every re-queue.
    camera2_shot_ext shotInput;
    memcpy(&shotInput, ispShot, sizeof(shotInput));

    int sccIdx = -1;
    G800FExynosCameraBuffer* sccBuf = NULL;
    bool ispOk = false;
    for (int iter = 0; iter < replayN; ++iter) {
        bool lastIter = (iter == replayN - 1);
        if (iter > 0) {
            // restore the input fields the firmware consumed/overwrote
            ispShot->setfile        = shotInput.setfile;
            ispShot->shot.ctl       = shotInput.shot.ctl;
            ispShot->shot.uctl      = shotInput.shot.uctl;
            ispShot->shot.dm        = shotInput.shot.dm;
            memcpy(&ispShot->node_group, &shotInput.node_group,
                   sizeof(ispShot->node_group));
            // distinct frameCount per pass so the firmware treats each
            // replay as a new frame and re-runs its 3A statistics
            ispShot->shot.dm.request.frameCount  = shotInput.shot.dm.request.frameCount + iter;
            ispShot->shot.ctl.request.frameCount = shotInput.shot.ctl.request.frameCount + iter;
        }

        v4l2_plane ispPlanes[VIDEO_MAX_PLANES];
        memset(ispPlanes, 0, sizeof(ispPlanes));
        ispPlanes[0].m.fd = fliteBuf->dmaBufFd(0);
        ispPlanes[0].bytesused = bayerSize;
        ispPlanes[0].length    = bayerSize;
        ispPlanes[1].m.fd = ispBuf->dmaBufFd(1);
        ispPlanes[1].bytesused = ispBuf->planeSize(1);
        ispPlanes[1].length    = ispBuf->planeSize(1);

        v4l2_buffer ispVbuf;
        memset(&ispVbuf, 0, sizeof(ispVbuf));
        ispVbuf.index  = fliteIdx;
        ispVbuf.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        ispVbuf.memory = V4L2_MEMORY_DMABUF;
        ispVbuf.length = 2;
        ispVbuf.m.planes = ispPlanes;

        err = m_reprocIspNode.qBuf(&ispVbuf);
        if (err != NO_ERROR) {
            ALOGE("%s: ISP[1] qBuf failed (iter=%d): %d", __FUNCTION__, iter, err);
            break;
        }

        // ISP[1] dequeue (DMABUF) — poll with timeout: a wedged ISP[1] frame
        // must never block forever, or finishStillCapture()'s join would hang
        // the capture worker (Snap freeze: preview runs, no photo).
        v4l2_buffer ispDq;
        v4l2_plane ispDqPlanes[VIDEO_MAX_PLANES];
        memset(ispDqPlanes, 0, sizeof(ispDqPlanes));
        memset(&ispDq, 0, sizeof(ispDq));
        ispDq.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        ispDq.memory = V4L2_MEMORY_DMABUF;
        ispDq.length = 2;
        ispDq.m.planes = ispDqPlanes;

        err = m_reprocIspNode.dqBufTimeout(&ispDq, REPROC_DQ_TIMEOUT_MS);
        if (err != NO_ERROR) {
            if (err == TIMED_OUT)
                ALOGE("%s: ISP[1] dqBuf TIMEOUT (iter=%d, %dms) — wedged frame, aborting",
                      __FUNCTION__, iter, REPROC_DQ_TIMEOUT_MS);
            else if (err != -EAGAIN)
                ALOGE("%s: ISP[1] dqBuf failed (iter=%d): %d", __FUNCTION__, iter, err);
            break;
        }

        // SCC[1] dequeue (full-res YUV) — poll with timeout (same reason as
        // ISP[1]: a stalled capture must not wedge the reproc thread).
        v4l2_buffer sccDq;
        v4l2_plane sccDqPlanes[VIDEO_MAX_PLANES];
        memset(sccDqPlanes, 0, sizeof(sccDqPlanes));
        memset(&sccDq, 0, sizeof(sccDq));
        sccDq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        sccDq.memory = V4L2_MEMORY_DMABUF;
        sccDq.length = SCC_REPROC_PLANES;
        sccDq.m.planes = sccDqPlanes;

        err = m_reprocSccNode.dqBufTimeout(&sccDq, REPROC_DQ_TIMEOUT_MS);
        if (err != NO_ERROR) {
            if (err == TIMED_OUT)
                ALOGE("%s: SCC[1] dqBuf TIMEOUT (iter=%d, %dms) — wedged frame, aborting",
                      __FUNCTION__, iter, REPROC_DQ_TIMEOUT_MS);
            else if (err != -EAGAIN)
                ALOGE("%s: SCC[1] dqBuf failed (iter=%d): %d", __FUNCTION__, iter, err);
            break;
        }
        sccIdx = sccDq.index;
        if (sccIdx < 0 || sccIdx >= NUM_SCC_REPROC_BUFFERS) {
            ALOGE("%s: SCC[1] dqBuf index out of range: %d (iter=%d)",
                  __FUNCTION__, sccIdx, iter);
            break;
        }
        sccBuf = m_reprocSccBuffers[sccIdx];
        ispOk = true;

        /* per-iteration diagnostic log disabled.
        {
            const uint32_t* v2 = ispShot->shot.udm.internal.vendorSpecific2;
            uint32_t cbIt = 0, crIt = 0;
            if (sccBuf)
                sampledYuyvChromaMean(sccBuf->planeVaddr(0),
                                      sccBuf->planeSize(0), &cbIt, &crIt);
            ALOGI("%s: REPROC-OUT iter=%d/%d fcount=%u: dm[aeState=%d "
                  "awbState=%d aeFlash=%d firingStable=%d exp=%lld iso=%u] "
                  "udm2[100..103]=%u %u %u %u Cb=%u Cr=%u",
                  __FUNCTION__, iter, replayN, bayerFrame->frameCount,
                  ispShot->shot.dm.aa.aeState, ispShot->shot.dm.aa.awbState,
                  ispShot->shot.dm.aa.aeflashMode,
                  ispShot->shot.dm.flash.firingStable,
                  (long long)ispShot->shot.dm.sensor.exposureTime,
                  ispShot->shot.dm.sensor.sensitivity,
                  v2[100], v2[101], v2[102], v2[103], cbIt, crIt);
        }
        */

        if (lastIter)
            break;  // keep sccIdx/sccBuf for output below

        // non-final iteration: recycle the SCC buffer for the next pass
        v4l2_buffer sccRq;
        v4l2_plane sccRqPlanes[VIDEO_MAX_PLANES];
        memset(sccRqPlanes, 0, sizeof(sccRqPlanes));
        memset(&sccRq, 0, sizeof(sccRq));
        sccRq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        sccRq.memory = V4L2_MEMORY_DMABUF;
        sccRq.index = sccIdx;
        sccRq.length = SCC_REPROC_PLANES;
        sccRq.m.planes = sccRqPlanes;
        sccRqPlanes[0].m.fd = m_reprocSccBuffers[sccIdx]->dmaBufFd(0);
        sccRqPlanes[0].length = m_reprocSccBuffers[sccIdx]->planeSize(0);
        sccRqPlanes[1].m.fd = m_reprocSccBuffers[sccIdx]->dmaBufFd(1);
        sccRqPlanes[1].length = m_reprocSccBuffers[sccIdx]->planeSize(1);
        m_reprocSccNode.qBuf(&sccRq);
    }

    // Release FLITE buffer (ISP[1] is done with Bayer)
    // IMPORTANT: clearFliteInUse BEFORE requeueBuffer — requeueBuffer checks
    // m_fliteInUse and refuses the requeue if still marked as in-use!
    m_flitePipe->clearFliteInUse(fliteIdx);
    m_flitePipe->requeueBuffer(fliteIdx);

    if (!ispOk || sccIdx < 0 || !sccBuf) {
        ALOGE("%s: reprocessing produced no output", __FUNCTION__);
        delete bayerFrame;
        return m_reprocThreadRunning;
    }

    {
        uint32_t yMean = sampledByteMean(sccBuf->planeVaddr(0),
                                         sccBuf->planeSize(0), 2);
        uint32_t cbMean = 0, crMean = 0;
        sampledYuyvChromaMean(sccBuf->planeVaddr(0), sccBuf->planeSize(0),
                              &cbMean, &crMean);
        ALOGI("%s: SCC[1] dqBuf idx=%d (full-res YUV, sampledYMean=%u "
              "Cb=%u Cr=%u)", __FUNCTION__, sccIdx, yMean, cbMean, crMean);
    }

    // Create output frame
    G800FFrame* outFrame = new G800FFrame();
    outFrame->bufferIndex = sccIdx;
    outFrame->buffer = m_reprocSccBuffers[sccIdx];
    outFrame->meta = m_reprocSccStream[sccIdx];
    outFrame->metaSize = sizeof(camera2_stream);
    outFrame->producerPipeId = PIPE_REPROC_SCC;
    outFrame->frameCount = bayerFrame->frameCount;
    outFrame->timestamp = bayerFrame->timestamp;

    // Push result into output queue
    m_reprocOutputQ.push_back(outFrame);

    // Release Bayer frame
    delete bayerFrame;

    // Requeue SCC[1] buffer for next reprocessing
    // TODO: collect multiple frames instead of immediate requeue
    v4l2_buffer sccQbuf;
    v4l2_plane sccQplanes[VIDEO_MAX_PLANES];
    memset(sccQplanes, 0, sizeof(sccQplanes));
    memset(&sccQbuf, 0, sizeof(sccQbuf));
    sccQbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    sccQbuf.memory = V4L2_MEMORY_DMABUF;
    sccQbuf.index = sccIdx;
    sccQbuf.length = SCC_REPROC_PLANES;
    sccQbuf.m.planes = sccQplanes;
    sccQplanes[0].m.fd = m_reprocSccBuffers[sccIdx]->dmaBufFd(0);
    sccQplanes[0].length = m_reprocSccBuffers[sccIdx]->planeSize(0);
    sccQplanes[1].m.fd = m_reprocSccBuffers[sccIdx]->dmaBufFd(1);
    sccQplanes[1].length = m_reprocSccBuffers[sccIdx]->planeSize(1);
    m_reprocSccNode.qBuf(&sccQbuf);

    return m_reprocThreadRunning;  // keep running as long as running
}

} // namespace android
