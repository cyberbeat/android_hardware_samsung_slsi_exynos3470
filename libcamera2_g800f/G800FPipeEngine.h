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

#ifndef G800F_PIPE_ENGINE_H
#define G800F_PIPE_ENGINE_H

#include <utils/Errors.h>
#include <utils/Mutex.h>
#include <utils/Condition.h>
#include <utils/Vector.h>
#include <utils/Thread.h>
#include <thread>
#include <atomic>
#include <system/camera_metadata.h>
#include "G800FPipe.h"
#include "G800FFrameQueue.h"
#include "G800FExynosCameraNode.h"
#include "G800FExynosCameraBuffer.h"
#include <fimc-is-metadata.h>

namespace android {

class G800FFrameSelector;

/*
 * G800FFlitePipe — FLITE sensor frontend (Bayer capture).
 *
 *   - m_getBuffer(): dequeue Bayer frame from FLITE V4L2 node
 *   - m_putBuffer(): requeue FLITE buffer for next capture
 *
 * On Exynos 3470, FLITE is the sensor capture node (/dev/video100).
 * It produces raw Bayer (SBGGR12) with shot_ext metadata in plane 1.
 *
 * The FLITE pipe feeds the ISP pipe via m_outputFrameQ.
 * The FLITE buffer is NOT requeued until the ISP pipe is done with it
 * (buffer ownership transfers to ISP pipe, which requeues it after
 * ISP dequeue — mirroring m_ispToFliteMap).
 *
 * For capture: the FrameSelector can intercept a FLITE frame and
 * redirect it to the reprocessing pipeline instead of the preview ISP.
 */
class G800FFlitePipe : public G800FPipe {
public:
    G800FFlitePipe();
    virtual ~G800FFlitePipe();

    virtual status_t create(int videoNodeNum, int sensorId);
    virtual status_t setupPipe(int w, int h, int pixFmt,
                               int numPlanes, int numBuffers,
                               v4l2_buf_type bufType,
                               v4l2_memory memory = V4L2_MEMORY_MMAP);

    // Queue initial FLITE buffers to the V4L2 node (call after setupPipe)
    status_t queueInitialBuffers();

    // Sensor stream control (V4L2_CID_IS_S_STREAM)
    status_t sensorStreamOn();
    status_t sensorStreamOff();

    // Set the shot_ext metadata pointer for each buffer.
    // Must be called after setupPipe, before queueInitialBuffers.
    void setShotExt(camera2_shot_ext** shotExt, int count) {
        m_shotExt = shotExt;
        m_shotExtCount = count;
    }
    camera2_shot_ext* getShotExt(int index) const {
        return (m_shotExt && index >= 0 && index < m_shotExtCount)
            ? m_shotExt[index] : NULL;
    }

    // --- Capture interception (FrameSelector integration) ---
    // When a capture is pending, FLITE frames are offered to the FrameSelector.
    // If accepted, the frame goes to the reprocessing queue.
    // If rejected (frame skip during flash), the frame is requeued to FLITE.
    void setCaptureRedirect(G800FFrameQueue* reprocQ) { m_reprocRedirectQ = reprocQ; }
    void clearCaptureRedirect() { m_reprocRedirectQ = NULL; }

    // FrameSelector — called by m_getBuffer() to offer FLITE frames
    // for selection. If the selector accepts the frame, it goes to
    // the reprocessing queue. Otherwise the normal preview path is used.
    void setFrameSelector(G800FFrameSelector* sel) { m_frameSelector = sel; }

    // Mark a FLITE buffer as in-use by ISP (don't requeue until ISP done)
    void markFliteInUse(int index) { m_fliteInUse[index] = true; }
    void clearFliteInUse(int index) { m_fliteInUse[index] = false; }
    bool isFliteInUse(int index) const { return m_fliteInUse[index]; }

    // Requeue a FLITE buffer after ISP/reprocessing is done with it
    status_t requeueBuffer(int index);

    // Flash state — written to all FLITE shot_ext before queueing
    void setAeFlashMode(int mode);
    int getAeFlashMode() const { return m_aeflashMode; }

protected:
    virtual status_t m_getBuffer();
    virtual status_t m_putBuffer();

private:
    camera2_shot_ext** m_shotExt;
    int m_shotExtCount;
    bool m_fliteInUse[8]; // max 8 FLITE buffers
    int m_aeflashMode;
    G800FFrameQueue* m_reprocRedirectQ; // if set, redirect capture frames here
    G800FFrameSelector* m_frameSelector; // FrameSelector (not owned)

    // Helper: build v4l2_buffer + planes for FLITE qbuf
    status_t qBufFlite(int index);
};

/*
 * G800FIspPipe — ISP processing (Bayer → YUV, 3A).
 *
 *   - m_putBuffer(): pop Bayer frame from input queue, queue to ISP
 *   - m_getBuffer(): dequeue processed frame from ISP, push to output
 *
 * On Exynos 3470, ISP is /dev/video130 (OUTPUT_MPLANE — receives Bayer
 * via DMA from FLITE buffer, produces YUV to SCC/SCP via OTF).
 *
 * The ISP pipe receives Bayer frames from FLITE pipe (m_inputFrameQ)
 * and pushes processed frames to SCC/SCP pipes (m_outputFrameQ).
 *
 * Buffer model: ISP is in DMA-input mode.  Plane 0 of the ISP qbuf
 * is the FLITE buffer's Bayer plane; plane 1 is the ISP buffer's
 * metadata (shot_ext).  The FLITE buffer stays "in use" until ISP
 * dequeue, at which point the FLITE pipe requeues it.
 */
class G800FIspPipe : public G800FPipe {
public:
    G800FIspPipe();
    virtual ~G800FIspPipe();

    virtual status_t create(int videoNodeNum, int sensorId);
    virtual status_t setupPipe(int w, int h, int pixFmt,
                               int numPlanes, int numBuffers,
                               v4l2_buf_type bufType,
                               v4l2_memory memory = V4L2_MEMORY_MMAP);

    // Queue initial ISP buffers (metadata only — Bayer comes from FLITE)
    status_t queueInitialBuffers();

    void setShotExt(camera2_shot_ext** shotExt, int count) {
        m_shotExt = shotExt;
        m_shotExtCount = count;
    }
    camera2_shot_ext* getShotExt(int index) const {
        return (m_shotExt && index >= 0 && index < m_shotExtCount)
            ? m_shotExt[index] : NULL;
    }

    // FLITE pipe reference for buffer requeue after ISP dequeue
    void setFlitePipe(G800FFlitePipe* flite) { m_flitePipe = flite; }

    void setFlashCaptureRedirect(G800FFrameQueue* reprocQ);
    void clearFlashCaptureRedirect();
    // Frame count of the frame that was actually redirected to reproc
    // (the firingStable+1 lit frame).  Set at redirect time — readable by the
    // still worker long before the slow ISP[1] reproc output arrives, so the
    // warm SCC[0] path can match on it without blocking on the cold result.
    uint32_t getFlashReprocTarget() const { return m_flashReprocTarget; }

    // Latest 3A state (updated by m_getBuffer from ISP dm fields)
    int getLatestAfState() const { return m_latestAfState; }
    int getLatestAfMode() const { return m_latestAfMode; }
    int getLatestAeState() const { return m_latestAeState; }
    int getLatestAwbState() const { return m_latestAwbState; }
    int getLatestFlashReady() const { return m_latestFlashReady; }
    int getLatestFiringStable() const { return m_latestFiringStable; }
    int getLatestFlashOffReady() const { return m_latestFlashOffReady; }
    void resetLatestFiringStable() { m_latestFiringStable = 0; m_latestFlashOffReady = 0; }
    int getLatestSensitivity() const { return m_latestSensitivity; }
    int64_t getLatestExposureTime() const { return m_latestExposureTime; }
    // dm-echoed control values (what the firmware actually applied)
    int getLatestDmAeMode() const { return m_latestDmAeMode; }
    int getLatestDmAwbMode() const { return m_latestDmAwbMode; }
    int getLatestFlashDecision() const { return m_latestFlashDecision; }
    int getLatestDmFlashMode() const { return m_latestDmFlashMode; }

    // Per-frame firmware results ring: the ISP[0] output's dm/udm/uctl
    // are stored here indexed by frameCount, decoupled from the flite
    // buffer lifetime — the flite buffer gets recycled while the reproc
    // thread still needs this frame's metadata.
    struct IspFrameMeta {
        volatile int32_t frameCount;  // -1 = empty, written last (release)
        camera2_dm   dm;
        camera2_uctl uctl;
        camera2_udm  udm;
    };
    // Copy out the firmware results for a frame; false if not arrived yet
    // or already evicted.
    bool copyFrameMeta(int32_t frameCount, camera2_dm* dm,
                       camera2_uctl* uctl, camera2_udm* udm) const {
        const IspFrameMeta& s = m_frameMetaRing[frameCount & 7];
        __sync_synchronize();
        if (s.frameCount != frameCount) return false;
        if (dm)   *dm   = s.dm;
        if (uctl) *uctl = s.uctl;
        if (udm)  *udm  = s.udm;
        __sync_synchronize();
        return s.frameCount == frameCount;  // re-check after copy
    }
    int getLatestDmAeflashMode() const { return m_latestDmAeflashMode; }

    // Recycle all frames still stuck in m_ispFrameMap
    // (e.g. after stopPreview). Returns them to the shared pool.
    void recycleStuckFrames();

protected:
    virtual status_t m_getBuffer();
    virtual status_t m_putBuffer();

private:
    camera2_shot_ext** m_shotExt;
    int m_shotExtCount;
    G800FFlitePipe* m_flitePipe;

    // ISP in-flight tracking
    Mutex m_inFlightLock;
    bool m_ispInFlight[8];
    int m_ispInFlightCount;
    int m_ispToFliteMap[8]; // ISP idx → FLITE idx
    // Frame pointer per ISP buffer index.
    // m_putBuffer stores the frame here, m_getBuffer retrieves it
    // and enriches it with metadata. The frame flows as a single
    // object through the pipeline (FLITE → ISP → SCP → HAL3).
    G800FFrame* m_ispFrameMap[8];

    // Latest 3A state (from ISP dm)
    int m_latestAfState;
    int m_latestAfMode;   // dm-echoed aa_afmode (kernel enum)
    int m_latestAeState;
    int m_latestAwbState;
    int m_latestFlashReady;
    int m_latestFiringStable;
    int m_latestFlashOffReady;
    int m_latestSensitivity;
    int64_t m_latestExposureTime;
    int m_latestDmAeMode;
    int m_latestDmAwbMode;
    int m_latestFlashDecision;
    int m_latestDmFlashMode;
    int m_latestDmAeflashMode;

    IspFrameMeta m_frameMetaRing[8];

    Mutex m_flashRedirectLock;
    G800FFrameQueue* m_flashReprocRedirectQ;
    /* Flash frame selection:
     *   Phase 1 (m_flashTargetFcount == 0): wait for firingStable==1.
     *     On firingStable: m_flashTargetFcount = frameCount + 1.
     *     The firingStable frame is released normally (preview).
     *   Phase 2 (m_flashTargetFcount != 0): wait for frame with
     *     frameCount >= m_flashTargetFcount. This frame is forwarded
     *     to reprocessing (flashRedirectQ).
     * waitFcount = startMainFlash() + 1
     * m_ShotFcount is set on firingStable */
    uint32_t m_flashTargetFcount;
    // Confirmed redirected-target fcount (== the frame actually pushed to the
    // reproc queue).  Unlike m_flashTargetFcount this is NOT cleared after the
    // redirect — it survives until the next setFlashCaptureRedirect, so the
    // still worker can read which lit frame was chosen without waiting for the
    // reproc output.  Written under m_flashRedirectLock; read as a plain u32.
    volatile uint32_t m_flashReprocTarget;

    // Memory mode for ISP V4L2 queue (MMAP or USERPTR).
    // USERPTR is required because the ISP reads Bayer data via VDMA1
    // from plane 0 — plane 0 must be the FLITE buffer's Bayer plane,
    // not the ISP's own MMAP buffer.
    v4l2_memory m_memory;

    // Helper: queue ISP buffer using FLITE Bayer + ISP metadata
    status_t qBufIsp(int ispIndex, int fliteIndex, G800FFrame* frame);
};

/*
 * G800FScpPipe — SCP scaler preview (preview YUV output).
 *
 *   - m_getBuffer(): dequeue preview YUV from SCP
 *   - m_putBuffer(): requeue SCP buffer
 *
 * On Exynos 3470, SCP is /dev/video137 (CAPTURE_MPLANE).
 * It receives YUV from ISP via OTF and scales to 1280x720 NV21M.
 */
class G800FScpPipe : public G800FPipe {
public:
    G800FScpPipe();
    virtual ~G800FScpPipe();

    virtual status_t create(int videoNodeNum, int sensorId);
    virtual status_t setupPipe(int w, int h, int pixFmt,
                               int numPlanes, int numBuffers,
                               v4l2_buf_type bufType,
                               v4l2_memory memory = V4L2_MEMORY_MMAP);

    status_t queueInitialBuffers();

    void setStreamMeta(camera2_stream** stream, int count) {
        m_stream = stream;
        m_streamCount = count;
    }
    camera2_stream* getStreamMeta(int index) const {
        return (m_stream && index >= 0 && index < m_streamCount)
            ? m_stream[index] : NULL;
    }

    // Get the latest preview frame (non-blocking).
    // Used by the HAL3 translation layer for preview delivery.
    // Returns NO_ERROR and sets *frame, or -EAGAIN if no frame available.
    status_t getPreviewFrame(G800FFrame** frame);

    // Recycle a preview frame after the consumer is done.
    void releasePreviewFrame(G800FFrame* frame);

    // Requeue a SCP V4L2 buffer by index (used internally by m_getBuffer).
    void requeueScpBuffer(int idx);

    // Set preview dimensions (called from startPreview)
    void setPreviewSize(int w, int h) { m_previewW = w; m_previewH = h; }

    // Pipeline model: SCP receives the frame from ISP via the base class
    // m_inputFrameQ (set by PipeEngine to &m_ispToScpQ).
    // SCP m_getBuffer pops the frame, dequeues from the V4L2 node,
    // adds preview data, and pushes it to the preview queue.
    // The frame flows as a single object through the entire pipeline
    // (FLITE → ISP → SCP → HAL3).

protected:
    virtual status_t m_getBuffer();
    virtual status_t m_putBuffer();

private:
    camera2_stream** m_stream;
    int m_streamCount;

    // SCP buffer in-use tracking
    Mutex m_inUseLock;
    bool m_scpInUse[16]; // max NUM_SCP_BUFFERS (9) + margin

    // Output queue for preview delivery (separate from pipe output queue)
    G800FFrameQueue m_previewQ;

    // Preview dimensions (for data copy sizing)
    int m_previewW;
    int m_previewH;
};

/*
 * G800FSccPipe — SCC scaler capture (full-res YUV output).
 *
 *   - m_getBuffer(): dequeue full-res YUV from SCC
 *   - m_putBuffer(): requeue SCC buffer
 *
 * On Exynos 3470, SCC is /dev/video134 (CAPTURE_MPLANE).
 * It receives YUV from ISP via OTF and scales to capture resolution.
 *
 * SCC is only active when a capture is requested (per-frame via
 * node_group capture request flag).  During preview-only, SCC buffers
 * are queued but the ISP doesn't route to SCC.
 */
class G800FSccPipe : public G800FPipe {
public:
    G800FSccPipe();
    virtual ~G800FSccPipe();

    virtual status_t create(int videoNodeNum, int sensorId);
    virtual status_t setupPipe(int w, int h, int pixFmt,
                               int numPlanes, int numBuffers,
                               v4l2_buf_type bufType,
                               v4l2_memory memory = V4L2_MEMORY_MMAP);

    status_t queueInitialBuffers();

    void setStreamMeta(camera2_stream** stream, int count) {
        m_stream = stream;
        m_streamCount = count;
    }
    camera2_stream* getStreamMeta(int index) const {
        return (m_stream && index >= 0 && index < m_streamCount)
            ? m_stream[index] : NULL;
    }

    // Get a capture frame (blocking with timeout).
    // Used by the HAL3 translation layer for still capture.
    status_t getCaptureFrame(G800FFrame** frame, int timeoutMs);
    void releaseCaptureFrame(G800FFrame* frame);

protected:
    virtual status_t m_getBuffer();
    virtual status_t m_putBuffer();

private:
    camera2_stream** m_stream;
    int m_streamCount;

    // SCC buffer in-use tracking
    Mutex m_inUseLock;
    bool m_sccInUse[8];

    // Output queue for capture delivery
    G800FFrameQueue m_captureQ;
};

/*
 * G800FPipeEngine — central manager for all pipes.
 *
 *   - Creates and configures all pipes
 *   - Connects pipes via frame queues
 *   - Starts/stops the pipeline
 *   - Manages buffer allocation
 *   - Provides preview/capture frame access to the HAL3 layer
 *
 * The engine is the single entry point for the HAL3 translation layer.
 * It hides all V4L2 details and pipe threading from the caller.
 */
class G800FPipeEngine {
    friend class G800FReprocThread;
    friend class G800FFlitePipe;
    friend class G800FIspPipe;
public:
    G800FPipeEngine(int cameraId);
    ~G800FPipeEngine();

    // --- Lifecycle ---
    status_t init();       // Open all nodes, create pipes
    void deinit();         // Close all nodes, destroy pipes

    // --- Preview pipeline ---
    // Configure and start the continuous preview pipeline:
    //   FLITE(3280x2458) → ISP(3280x2458) → SCC(3264x2448)
    //                                     → SCP(previewW x previewH)
    // FLITE runs CONTINUOUSLY at full-res. No setSize on capture.
    // SCC delivers full-res YUV for still capture (via FrameSelector).
    // SCP delivers preview YUV for the preview.
    status_t startPreview(int previewW, int previewH);
    status_t stopPreview();
    bool isPreviewRunning() const { return m_previewRunning; }
    bool isReprocessingActive() const { return m_reprocessing; }

    // Reconfigure only the SCP (preview) node to a new resolution.
    // FLITE/ISP/SCC stay streaming on full-res — only SCP is stopped,
    // reconfigured (s_fmt + new buffers), and restarted.
    // Used by configureStreams() when the app changes preview resolution
    // without closing/reopening the camera device.
    // Returns NO_ERROR on success, or falls back to full stopPreview+startPreview.
    status_t reconfigureScp(int newW, int newH);

    // --- Preview frame access ---
    // Get the latest preview frame from SCP (non-blocking).
    // Caller must releasePreviewFrame() when done.
    status_t getPreviewFrame(G800FFrame** frame);
    void releasePreviewFrame(G800FFrame* frame);

    // --- Capture frame access ---
    // Get a capture frame from SCC (blocking with timeout).
    // Only valid if SCC was configured (sccW/H > 0 in startPreview).
    status_t getCaptureFrame(G800FFrame** frame, int timeoutMs);
    void releaseCaptureFrame(G800FFrame* frame);
    // Warm-path still: pop SCC[0] frames until one matches the flash-redirect
    // target fcount (firingStable+1).  Falls back to the most recent lit frame.
    status_t getWarmCaptureFrame(G800FFrame** frame, int32_t targetFcount,
                                 int timeoutMs);
    // Confirmed flash-redirect target fcount (the lit firingStable+1 frame that
    // was forwarded to reproc).  Available ~when the frame enters ISP[1] —
    // ~1s before the cold reproc output — so the warm SCC[0] path can match on
    // it without blocking on getReprocessedFrame.
    uint32_t getFlashReprocTarget() const;
    // Bounded wait until the redirect target is known (0 on timeout / not armed).
    uint32_t waitFlashReprocTarget(int timeoutMs);

    // --- Still-Capture via FrameSelector + asynchronous reprocessing ---
    // Conformant behavior:
    //   1. FrameSelector selects a FLITE Bayer frame (pipeId=0) from
    //      the running preview stream (200ms timeout)
    //   2. Asynchronous reprocessing pipeline:
    //      Bayer → ISP_REPROC(0xCA) → SCC_REPROC(0xCB)
    //            → GSC_PICTURE(0xCD) → JPEG_REPROC(0xCE)
    //   3. No setSize, no stream off/on, no preview interruption
    //   4. With flash: use the previous ISP frame at dm.flash.firingStable==1
    status_t requestStillCapture(int targetW, int targetH,
                                 bool needFlash);
    status_t getReprocessedFrame(G800FFrame** frame, int timeoutMs);
    void releaseReprocessingFrame(G800FFrame* frame);

    // Complete still capture: stop reproc thread, teardown pipeline,
    // clear FLITE capture redirect. Preview continues uninterrupted.
    void finishStillCapture();

    bool needsReprocessing(int targetW, int targetH) const;

    // --- 3A state ---
    int getLatestAfState() const;
    int getLatestAfMode() const;
    // Last request's ANDROID_CONTROL_AF_MODE_* (for result echo +
    // afState mapping context).  Defaults to CONTINUOUS_PICTURE on cam0.
    int getLastReqAfMode() const { return m_lastReqAfMode; }
    int getLatestAeState() const;
    int getLatestAwbState() const;
    int getLatestFlashReady() const;
    int getLatestFiringStable() const;
    int getLatestFlashOffReady() const;
    int getLatestSensitivity() const;
    int64_t getLatestExposureTime() const;

    // --- Flash ---
    // Complete flash state machine.
    // Sequence: IDLE → START → ON → METERING → READY → CAPTURE → DONE → IDLE
    // Kernel reacts to aeflashMode changes via fimc_is_group_set_torch():
    //   START(2)   → rt5033_flash_force_enable(true)  [pre-flash on]
    //   CAPTURE(6) → rt5033_flash_force_enable(true) + rt5033_gpio_flash_lock(true) [main flash]
    //   OFF(1)     → rt5033_flash_force_enable(false) [LED off]
    //   ON(4), AUTO(5) → firmware-internal (no kernel action)
    enum FlashSeqState {
        FLASH_SEQ_IDLE = 0,
        FLASH_SEQ_START,
        FLASH_SEQ_ON,
        FLASH_SEQ_METERING,
        FLASH_SEQ_AF_WAIT,    // AF scan under preflash
        FLASH_SEQ_READY,      // AE converged, CAPTURE aeflashMode already set
        FLASH_SEQ_CAPTURE,    // Main flash fires (caller armed via armFlashCapture)
        FLASH_SEQ_MAIN_WAIT,  // P2: after main flash, wait for stabilization
        FLASH_SEQ_DONE,
    };

    void beginFlashSequence();
    void advanceFlashSequence(uint32_t frameCount);
    void writeFlashSeqMetadata(int state);
    void armFlashCapture();
    void endFlashSequence();
    void finishFlashCapture();  // Wrapper for endFlashSequence()
    status_t releaseFlash();    // sysfs 'U' on rear_flash_ext
    status_t armFlashLock();    // sysfs 'L' on rear_flash_ext (early strobe lock,
                                // clears the FLED TA/charger limit for the
                                // whole sequence incl. preflash torch)

    bool isFlashSequenceActive() const;
    bool isFlashSequenceReady() const;
    // Blocks on m_flashSeqCond until the sequence is ready to arm:
    // state READY *and* (dm.flashReady==2 seen or READY-timeout), or a
    // later state; dies (IDLE via watchdog/endFlashSequence), or
    // timeoutMs elapses.
    // Returns true when metering completed.
    bool waitFlashSequenceReady(int timeoutMs);
    int  getFlashSeqState() const;
    uint32_t getFlashCaptureFrames() const;
    static const char* flashSeqStateName(int state);

    void setAeFlashMode(int mode);
    int getAeFlashMode() const;

    // --- Geometry ---
    int getPreviewWidth() const { return m_previewW; }
    int getPreviewHeight() const { return m_previewH; }
    int getFliteWidth() const { return (m_cameraId == 0) ? FLITE_W : FLITE_FRONT_W; }
    int getFliteHeight() const { return (m_cameraId == 0) ? FLITE_H : FLITE_FRONT_H; }
    int getSccWidth() const { return (m_cameraId == 0) ? SCC_W : SCC_FRONT_W; }
    int getSccHeight() const { return (m_cameraId == 0) ? SCC_H : SCC_FRONT_H; }

    // --- Settings application ---
    // Apply Camera2 request settings to the next FLITE shot_ext.
    // Returns the AF trigger value that was applied.
    int applyRequestSettings(const camera_metadata_t* settings);

    // --- AF-Cancel ---
    // Sets aa.afMode=OFF and aa.afTrigger=0 in all FLITE/ISP shot_ext
    // buffers. Called on flush() and close() to reset the AF state machine.
    void cancelAutoFocus();

    // --- Node group helpers ---
    // Simple version: input == output crop (for leader or symmetric nodes).
    static void setNodeGroup(camera2_shot_ext* shot, int index, int vid,
                             int request, int x, int y, int w, int h);
    // Full version: separate input/output crops (conformant,
    //   corresponds to updateNodeGroupInfoMainPreview).
    //   ix/iy/iw/ih = input crop, ox/oy/ow/oh = output crop.
    static void setNodeGroup(camera2_shot_ext* shot, int index, int vid,
                             int request,
                             int ix, int iy, int iw, int ih,
                             int ox, int oy, int ow, int oh);

    // --- Sensor ID helpers (packing shared with the buffer layer) ---
    static int packSensorIdFLITE(int cameraId);
    static int packSensorIdISP(int cameraId);
    static int packSensorIdSCC(int cameraId);
    static int packSensorIdSCP(int cameraId);
    static int packSensorIdISPReproc(int cameraId);
    static int packSensorIdSCCReproc(int cameraId);

    // --- Constants ---
    static const int SENSOR_ID_S5K4H5 = 13;
    static const int SENSOR_ID_S5K6B2 = 7;
    static const int NODE_FLITE_REAR  = 100;
    static const int NODE_FLITE_FRONT = 101;
    static const int NODE_ISP    = 130;
    static const int NODE_SCC    = 134;
    static const int NODE_SCP    = 137;

    // Full-res sensor dimensions (hwSensorSize + margin)
    // S5K4H5 (Rear): hwSensorSize = 3264x2448, margin = 16x10
    //   FLITE and ISP run CONTINUOUSLY at this size.
    static const int FLITE_W = 3280;
    static const int FLITE_H = 2458;
    static const int ISP_W   = 3280;
    static const int ISP_H   = 2458;
    static const int SCC_W   = 3264;  // = hwSensorSize (FLITE - margin)
    static const int SCC_H   = 2448;

    // Front camera (S5K6B2/S5K8B1, 2MP): maxSensor = 1920x1080, margin = 16x10
    //   maxSensorW = 1920, maxSensorH = 1080
    //   sensorMarginW = 16, sensorMarginH = 10
    //   FLITE = 1920+16 x 1080+10 = 1936x1090
    //   bdsPreviewEnabled = 0 (no BDS scaling on front camera!)
    static const int FLITE_FRONT_W = 1936;
    static const int FLITE_FRONT_H = 1090;
    static const int ISP_FRONT_W   = 1936;
    static const int ISP_FRONT_H   = 1090;
    static const int SCC_FRONT_W   = 1920;
    static const int SCC_FRONT_H   = 1080;

    // Default preview dimensions (SCP output, configurable)
    static const int DEFAULT_PREVIEW_W = 960;
    static const int DEFAULT_PREVIEW_H = 720;

    // BDS (Bad pixel Detection & Scaling) table — S5K4H5 rear,
    // 7 entries of 11 int32_t each.
    // getPreviewBdsSize() reads entry[7] and entry[8].
    // BDS index from g_previewResolutionsRear: third column = BDS index.
    //   960x720  → index 1 → BDS 1440x1080 (4:3)
    //   1280x720 → index 0 → BDS 1920x1080 (16:9)
    // leader.output = min(bayerCrop, bds) — ISP scales to BDS size,
    // SCC/SCP scale further from there.
    struct BdsEntry {
        int ratioNum;   // Aspect ratio numerator
        int ratioDen;   // Aspect ratio denominator
        int bdsW;       // BDS output width
        int bdsH;       // BDS output height
    };
    static const BdsEntry BDS_TABLE[];
    static int getPreviewBdsSize(int previewW, int previewH, int *bdsW, int *bdsH);

    // Buffer counts:
    //   FLITE: m_buffers[8], ISP: m_buffers[8], SCC: m_buffers[5], SCP: m_buffers[9]
    static const int NUM_FLITE_BUFFERS = 8;
    static const int NUM_ISP_BUFFERS   = 8;
    static const int NUM_SCP_BUFFERS   = 9;
    static const int NUM_SCC_BUFFERS   = 5;
    // Reproc buffers are indexed by fliteIdx (see reprocThreadLoop):
    // the ISP[1] qbuf passes the FLITE buffer's own dmaBufFd for plane 0,
    // so the kernel's stable-DMA-address-per-index check requires one
    // reproc buffer index per FLITE buffer index.
    static const int NUM_REPROC_BUFFERS     = 8;  // ISP[1] input, per-fliteIdx
    static const int NUM_SCC_REPROC_BUFFERS = 5;  // SCC[1] capture output

    // Plane counts — per kernel fimc-is-video.c:
    //   V4L2_MEMORY_DMABUF (m_memory=4) uses 4 planes for SCP, where the
    //   4th plane (SPARE/metadata) is allocated by the user.
    //   We use V4L2_MEMORY_MMAP (memory=1), where the kernel allocates
    //   the buffers. The kernel supports only 3 planes for SCP in MMAP
    //   mode (Y + VU + SPARE). With 4 planes, QUERYBUF fails
    //   (plane[3].length=0). Therefore we use 3 planes in MMAP mode.
    //   SBGGR12: 1+SPARE=2, YUYV: 1+SPARE=2
    static const int FLITE_PLANES = 2; // Bayer + SPARE
    static const int ISP_PLANES   = 2; // Bayer + SPARE
    static const int SCP_PLANES   = 3; // Y + VU + SPARE (MMAP mode; DMABUF: 4)
    static const int SCC_PLANES   = 2; // YUYV packed + SPARE (YUYV, not NV21M!)
    static const int SCC_REPROC_PLANES = 2; // YUYV packed + SPARE (Reprocessing)
    static const int SPARE_SIZE   = 32 * 1024; // metadata plane (kernel: fimc-is-video.c SPARE_SIZE)

private:
    int m_cameraId;
    std::atomic<bool> m_initialized;
    std::atomic<bool> m_previewRunning;
    std::atomic<bool> m_reprocessing;
    std::atomic<bool> m_reprocNodesReady;  // Reprocessing nodes created+configured in startPreview
    std::atomic<bool> m_reprocBuffersReady; // Reprocessing buffers allocated in startPreview
    std::atomic<bool> m_reprocPipelineStreaming; // Reproc pipeline (ISP[1]+SCC[1]) is STREAMON — stays active between photos (conformant)
    std::atomic<bool> m_reprocSetupInProgress;  // async setup thread running
    std::thread m_reprocSetupThread;            // async setup thread (joinable)

    // Pipeline geometry (FLITE/ISP/SCC always full-res, SCP = preview)
    int m_previewW, m_previewH;  // SCP V4L2 buffer size (= BDS size)

    // Pipes — held as sp<> because G800FPipe extends Thread (RefBase).
    // Android's Thread::_threadLoop() releases its strong reference after each
    // threadLoop() iteration and re-acquires it via weak.promote(). If no one
    // else holds a strong reference, the thread exits and the object is deleted
    // from within the thread. Using sp<> here keeps the pipes alive.
    sp<G800FFlitePipe> m_flitePipe;
    sp<G800FIspPipe>   m_ispPipe;
    sp<G800FScpPipe>   m_scpPipe;
    sp<G800FSccPipe>   m_sccPipe;

    // Frame queues connecting pipes
    G800FFrameQueue m_fliteToIspQ;   // FLITE → ISP
    G800FFrameQueue m_ispToSccQ;     // ISP → SCC (capture frames)
    G800FFrameQueue m_ispToScpQ;     // ISP → SCP (preview frames)

    // Shared frame pool — a frame flows as a single object
    // through FLITE → ISP → SCP → HAL3 and is returned at the end.
    Vector<G800FFrame*> m_sharedFramePool;
    Mutex m_sharedFramePoolLock;

    // FLITE/ISP metadata buffers
    camera2_shot_ext* m_fliteShotExt[NUM_FLITE_BUFFERS];
    camera2_shot_ext* m_ispShotExt[NUM_ISP_BUFFERS];
    camera2_stream*   m_scpStream[NUM_SCP_BUFFERS];
    camera2_stream*   m_sccStream[NUM_SCC_BUFFERS];

    // Reprocessing nodes (asynchronous pipeline, separate V4L2 instances)
    // ISP_REPROC (0xCA) and SCC_REPROC (0xCB) use instance 1.
    // FLITE stays on instance 0 (preview), Bayer is passed across.
    G800FExynosCameraNode m_reprocIspNode;
    G800FExynosCameraNode m_reprocSccNode;
    G800FExynosCameraBuffer* m_reprocIspBuffers[NUM_REPROC_BUFFERS];
    G800FExynosCameraBuffer* m_reprocSccBuffers[NUM_SCC_REPROC_BUFFERS];
    camera2_shot_ext* m_reprocIspShotExt[NUM_REPROC_BUFFERS];
    camera2_stream*   m_reprocSccStream[NUM_SCC_REPROC_BUFFERS];

    // FrameSelector for Bayer frame selection from FLITE stream
    G800FFrameSelector* m_frameSelector;

    // Asynchronous reprocessing queue: FrameSelector → reprocessing pipeline
    G800FFrameQueue m_bayerForReprocQ;   // FLITE Bayer → reprocessing
    G800FFrameQueue m_reprocOutputQ;     // Reprocessing → JPEG output

    // Flash frame skip (m_flashFrameSkipCount = 8)
    int m_flashFrameSkipCount;
    void setFlashFrameSkipCount(int count) { m_flashFrameSkipCount = count; }

    // --- Flash State Machine ---
    // Frame-based transitions: START→ON (3 frames), ON→METERING (3 frames),
    // METERING→READY (AE converged or timeout 300-1500ms).
    // READY→CAPTURE via armFlashCapture() (caller-driven).
    // CAPTURE→OFF via endFlashSequence() (caller-driven).
    mutable Mutex      m_flashSeqLock;
    Condition          m_flashSeqCond;
    int                m_flashSeqState;          // FlashSeqState
    uint32_t           m_flashSeqStateFrames;    // Frames in current state
    volatile uint32_t  m_flashSeqCaptureFrames;  // Frames with CAPTURE queued
    nsecs_t            m_flashSeqStateEnter;     // systemTime() on state entry
    nsecs_t            m_flashSeqBegin;          // systemTime() on beginFlashSequence()
    // MAIN_READY gate: dm.flash.flashReady==2 = firmware finished
    // preflash metering and computed the main-flash parameters.
    // Firing CAPTURE before this is a blind strobe (blown highlights on
    // bright scenes — observed 2026-09-24).  Latched from dm per frame.
    bool               m_flashReadyMain;
    int                m_flashState;             // Current aeflashMode (for applyRequestSettings)
    // Last request-derived ae/awb/aeFlash modes — cached in
    // applyRequestSettings(), restored to ctl when the flash sequence
    // ends (repeating preview requests carry NULL settings and cannot
    // restore them, which left AE/AWB OFF → green preview).
    int                m_lastReqAeMode;
    int                m_lastReqAwbMode;
    int                m_lastReqAeFlashMode;
    // Last request-derived AF state (ANDROID_CONTROL_AF_MODE_* space and
    // the mapped aa_afmode).  NULL-settings requests re-apply these —
    // HAL3 spec: NULL settings = reuse the most recent request's values.
    int                m_lastReqAfMode;
    int                m_lastReqIspAfMode;
    // Rest of the last-request cache replayed on NULL-settings requests:
    // scene/fps/ev/sensor/lens values + AF regions.
    int                m_lastReqSceneMode;
    int                m_lastReqAeFps[2];
    int                m_lastReqAeExpComp;
    int64_t            m_lastReqSensorExposure;
    int64_t            m_lastReqSensorFrameDur;
    int                m_lastReqSensorSensitivity;
    int                m_lastReqFocusDistance;
    int                m_lastReqAfRegions[5];
    // AF kick pending: set at every startPreview — afTrigger=1 must pulse
    // whenever the AF mode is applied, including at session start.
    // Without a session-start pulse the firmware's CAF engine never scans
    // (observed: first session worked only because the buffer-init trigger
    // leaked into early frames by timing luck).
    bool               m_afKickPending;
    // Parameter control block: holds the request-derived ctl
    // (applyRequestSettings + init defaults).  The flash state machine
    // NEVER writes here — it only touches the per-frame shot_ext buffers.
    // The reprocessing shot's ctl is copied from this block, NOT from the
    // captured frame's shot, which carries flash-sequence residue.
    camera2_ctl        m_paramsCtl;

    static const uint32_t FLASH_SEQ_START_FRAMES    = 3;
    static const uint32_t FLASH_SEQ_ON_FRAMES       = 3;
    // AE_WAIT semantics: exits on aeState==INACTIVE (never reported by
    // this firmware with aeMode=OFF) → must run a generous metering
    // window of preflash flash-AE/WB metering.  Our old 300ms exited on
    // the ambient-converged aeState=3 almost immediately — the firmware's
    // flash metering raced (blue tint / blown captures, 2026-09-23).
    static const int      FLASH_SEQ_METERING_MIN_MS = 1200;
    static const int      FLASH_SEQ_METERING_MAX_MS = 2500;
    static const int      FLASH_SEQ_AF_MIN_MS       = 150;
    static const int      FLASH_SEQ_AF_MAX_MS       = 1200;
    // MAIN_READY timeout: the AUTO tuple (LED off) is held until
    // dm.flash.flashReady==2, bounded by ~31 frames (≈ 1 s at 30 fps).
    static const int      FLASH_SEQ_READY_TIMEOUT_MS = 1100;
    // Sensor protection: maximum duration of the entire flash sequence. If the
    // sequence runs longer (e.g. because the caller never calls
    // armFlashCapture/endFlashSequence, or the capture thread has crashed),
    // the watchdog in advanceFlashSequence() forcibly turns off the flash.
    static const int      FLASH_SEQ_TOTAL_TIMEOUT_MS = 8000;
    // ISP[1]/SCC[1] dequeue timeout in the async reproc thread.  The kernel
    // dqbuf blocks when a buffer is queued but the firmware has not finished
    // it — on a wedged/stale frame this would hang the thread forever and
    // finishStillCapture()'s join would block the capture worker (Snap freeze).
    // Normal reprocess latency is ~35ms; 2500ms is generous yet well inside
    // the 10s getReprocessedFrame budget so a stall falls back cleanly.
    static const int      REPROC_DQ_TIMEOUT_MS = 2500;
    // Bounded wait for the async reproc-node/buffer setup thread in
    // requestStillCapture().  The thread does V4L2 s_fmt on ISP[1]/SCC[1] which
    // the kernel can serialize against streaming ISP[0]; an unbounded join()
    // would freeze the capture worker.  Normally the setup finished at preview
    // start long before the still, so this wait is a no-op — 3000ms is only a
    // safety bound for a wedged setup.
    static const int      REPROC_SETUP_TIMEOUT_MS = 3000;

    // Reprocessing setup/teardown (asynchronous, no FLITE setSize)
    status_t setupReprocessingNodes(int ispW, int ispH, int sccW, int sccH);
    void closeReprocessingNodes();
    status_t allocReprocessingBuffers(int ispW, int ispH, int sccW, int sccH);
    void releaseReprocessingBuffers();
    status_t startReprocessingPipeline();
    status_t stopReprocessingPipeline();

    // FLITE reconfiguration helper — NO LONGER USED (corrected in R4).
    // FLITE runs continuously at 3280x2458. No setSize on capture.
    // TODO: Remove once all references have been updated.
    status_t reconfigureFlite(int newW, int newH);

    // Asynchronous reprocessing thread
    // Fetches Bayer frame from m_bayerForReprocQ, processes via
    // ISP_REPROC → SCC_REPROC, pushes result to m_reprocOutputQ.
    sp<Thread> m_reprocThread;
    std::atomic<bool> m_reprocThreadRunning;
    bool reprocThreadLoop();

    // Instance lock (only one camera at a time)
    static Mutex s_instanceLock;
    static G800FPipeEngine* s_activeInstance;

    G800FPipeEngine(const G800FPipeEngine&);
    G800FPipeEngine& operator=(const G800FPipeEngine&);
};

// G800FReprocThread — asynchronous reprocessing worker.
// Fetches Bayer frames from the queue, processes them via ISP_REPROC → SCC_REPROC,
// and pushes the result to the output queue.
class G800FReprocThread : public Thread {
public:
    explicit G800FReprocThread(G800FPipeEngine* engine)
        : Thread(false), m_engine(engine) {}
    virtual bool threadLoop() { return m_engine->reprocThreadLoop(); }
private:
    G800FPipeEngine* m_engine;
};

} // namespace android

#endif // G800F_PIPE_ENGINE_H
