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

#ifndef G800F_CAMERA2_DEVICE_H
#define G800F_CAMERA2_DEVICE_H

#include <hardware/camera3.h>
#include <system/camera_metadata.h>
#include <utils/Mutex.h>
#include <utils/Condition.h>
#include <utils/threads.h>
#include <ui/GraphicBufferMapper.h>
#include <ui/Rect.h>

#include <list>
#include <vector>
#include <atomic>

#include "G800FPipeEngine.h"

namespace android {

class G800FFimcScaler;

/*
 * Camera3 (HAL3) device for the G800F.
 *
 * Architecture (3-layer model, all in libcamera2_g800f/):
 *
 *   Layer 3: HAL3 Interface ( this class )
 *     - camera3_device_ops_t, hw_module_t
 *     - Receives camera3_capture_request_t
 *     - Request queueing, result callbacks, stream management
 *
 *   Layer 2: HAL3→Pipeline Translation ( this class + G800FPipeEngine )
 *     - Translates camera3_capture_request_t into pipeline operations
 *     - Preview requests → SCP (non-blocking)
 *     - Still capture requests → FrameSelector + asynchronous reprocessing
 *     - Back-translation into camera3_capture_result_t
 *     - Metadata and JPEG output
 *
 *   Layer 1: Pipe Implementation ( G800FPipeEngine + G800FPipe* )
 *     - Direct V4L2/FIMC-IS access
 *     - Independent pipe threads
 *     - Frame queues between pipes
 *     - Continuous preview pipeline (FLITE 3280x2458)
 *     - Asynchronous reprocessing pipeline
 *     - Flash frame skip (8 frames)
 *
 * Processing:
 * processCaptureRequest pushes requests into a queue and returns immediately.
 * A capture thread worker processes them asynchronously and calls
 * process_capture_result when done.
 */
class G800FCamera2Device {
    friend class G800FCaptureThread;
    friend class G800FCaptureWorkerThread;
    friend class G800FAnalysisThread;
public:
    G800FCamera2Device(int cameraId, const hw_module_t* module);
    ~G800FCamera2Device();

    hw_device_t* open();
    int close();

    // camera3_device_ops_t callbacks
    int initialize(const camera3_callback_ops_t *callback_ops);
    int configureStreams(camera3_stream_configuration_t *stream_list);
    const camera_metadata_t* constructDefaultRequestSettings(int type);
    int processCaptureRequest(camera3_capture_request_t *request);
    int flush();
    void dump(int fd);

private:
    // --- Async request processing ---
    struct QueuedRequest {
        uint32_t frame_number;
        camera3_stream_buffer_t *output_buffers;  // cloned array
        uint32_t num_output_buffers;
        camera_metadata_t *settings;              // cloned (mutable)
        int64_t shutter_timestamp;                // set when queued
        bool shutter_sent;                        // true if shutter already notified
        bool metadata_sent;                       // true if result metadata already sent
        bool flashFired;                          // true if this request ran the flash sequence
    };

    std::list<QueuedRequest> m_requestQueue;
    mutable Mutex m_queueLock;
    Condition m_queueCond;
    sp<Thread> m_captureThread;

    // Async still-capture worker (separate thread so preview doesn't block)
    std::list<QueuedRequest> m_captureQueue;
    mutable Mutex m_captureQueueLock;
    Condition m_captureQueueCond;
    sp<Thread> m_captureWorkerThread;
    std::atomic<bool> m_captureWorkerRunning;

    // Async analysis/callback-stream worker.  Camera1 clients that use
    // setPreviewCallbackWithBuffer (e.g. Snap via Camera2Client) configure
    // an extra YCbCr_420_888 stream that must be filled for EVERY preview
    // request.  The scalar CPU copy (~11ms on uncached ION memory) was on
    // the capture thread's critical path, directly adding to every frame
    // interval (~58ms vs ~47ms single-stream ≈ 16fps vs 21fps).
    // The worker owns the preview frame until the fill is done — the SCP
    // V4L2 buffer must not be requeued while the CPU reads from it.
    struct AnalysisBuf {
        camera3_stream_buffer_t sb;   // sb.buffer stays valid until returned
        int w, h;                     // dims copied at enqueue time — the
                                      // worker never dereferences sb.stream
    };
    struct AnalysisJob {
        uint32_t frame_number;
        std::vector<AnalysisBuf> bufs;              // deferred output buffers
        G800FFrame *frame;                          // owned; worker releases
    };
    std::list<AnalysisJob> m_analysisQueue;
    mutable Mutex m_analysisQueueLock;
    Condition m_analysisQueueCond;
    sp<Thread> m_analysisThread;
    std::atomic<bool> m_analysisWorkerRunning;

    // Worker thread main loop
    bool captureThreadLoop();
    bool captureWorkerLoop();
    bool analysisWorkerLoop();
    void processOneRequest(QueuedRequest &qr);

    // Fill one deferred analysis buffer set and send it as a buffers-only
    // result (meta=NULL) for the same frame — legal per camera3 spec,
    // identical to the async still-capture buffer path.
    // Does NOT release job.frame — the caller owns it.
    void fillAnalysisJob(AnalysisJob &job);

    // Fast fill path for the callback stream: FIMC M2M (node 2) scales
    // SCP NV21M → NV21 into the bounce dmabuf, then deinterleaves
    // NV21 → the destination format (I420/YV12/NV21) — ~15ms total vs
    // ~150-250ms for the scalar CPU scaler on uncached ION memory.
    status_t fillAnalysisViaFimc(G800FExynosCameraBuffer *camBuf,
                                 buffer_handle_t outBuf, int w, int h);
    // Copies the NV21 bounce buffer into a gralloc output buffer
    // (deinterleaves chroma for planar destinations).
    void copyBounceNv21ToStream(buffer_handle_t outBuf, int w, int h);

    // Return deferred analysis buffers with BUFFER_STATUS_ERROR
    // (flush/close drain path).  Sends CAMERA3_MSG_ERROR_BUFFER notify.
    void errorAnalysisJob(AnalysisJob &job);

    // Dump all important Camera2 request settings (AE/AWB/AF/Flash/Scene/etc.)
    void dumpRequestSettings(const camera_metadata_t *settings);

    // Helpers used by the worker
    void sendResult(uint32_t frameNumber,
                    const camera3_stream_buffer_t *buffers,
                    uint32_t numBuffers,
                    camera_metadata_t *meta,
                    int64_t timestamp);
    void sendErrorResult(uint32_t frameNumber,
                         camera3_stream_buffer_t *buffers,
                         uint32_t numBuffers);

    // --- Layer 2: Request Dispatcher ---
    // Classifies requests:
    //   Preview-only → servePreviewRequest (SCP non-blocking)
    //   Still capture → serveCaptureRequest (FrameSelector + reprocessing)
    void handleRequest(QueuedRequest &qr);
    bool servePreviewRequest(QueuedRequest &qr, G800FFrame* previewFrame);
    bool serveCaptureRequest(QueuedRequest &qr, G800FFrame* previewFrame);

    // Build the per-frame result metadata.
    // If previewFrame is non-NULL, its per-frame 3A metadata (afState,
    // aeState, awbState, sensitivity, exposureTime) is used — conformant.
    // Otherwise, falls back to the PipeEngine's global latest values.
    camera_metadata_t* buildResultMetadata(const QueuedRequest &qr,
                                           int64_t timestamp,
                                           bool flashFired,
                                           G800FFrame* previewFrame = NULL);

    // Copy SCP NV21M frame into gralloc output buffer (from V4L2 buffer)
    void copyPreviewToStream(G800FExynosCameraBuffer *camBuf,
                             buffer_handle_t outBuf,
                             int width, int height);
    // Copy SCP NV21M frame into gralloc output buffer (from raw data copy)
    // srcY/srcVU = Y/VU plane pointers, srcW = stride, srcH = height
    // This is the software scaler — GSC-replaceable by an alternative
    // implementation that uses the same signature.
    void copyPreviewToStreamFromData(void* srcY, void* srcVU,
                                     int srcW, int srcH,
                                     buffer_handle_t outBuf,
                                     int width, int height);

    // Encode YUV frame to JPEG and write to BLOB buffer.
    // blobBufSize = actual size of the BLOB gralloc buffer (stream->width
    // for HAL_PIXEL_FORMAT_BLOB, where width=maxJpegSize and height=1).
    // dstW/dstH = target image dimensions (NOT the BLOB buffer dimensions).
    int copyCaptureToJpeg(G800FExynosCameraBuffer *camBuf,
                          buffer_handle_t outBuf,
                          int srcW, int srcH,
                          int quality, int rotation,
                          int dstW, int dstH,
                          bool isYuyv = false,
                          int blobBufSize = 0);

    // HW JPEG encoding via ExynosJpegEncoder (/dev/video12).
    // Takes YUYV (SCC) or NV21M (preview) directly as dmabuf,
    // encodes via hardware JPEG encoder, writes JPEG to gralloc BLOB.
    // Return: JPEG size in bytes, or -1 on error (caller falls back to
    // copyCaptureToJpeg).
    int copyCaptureToHwJpeg(G800FExynosCameraBuffer *camBuf,
                             buffer_handle_t outBuf,
                             int srcW, int srcH,
                             int quality, int rotation,
                             int dstW, int dstH,
                             bool isYuyv,
                             int blobBufSize);

    // Scales a camera buffer to the requested JPEG dimensions via FIMC
    // M2M (node 3) so the HW encoder produces a JPEG at dstW×dstH that
    // fits the BLOB buffer — the encoder always encodes at input size.
    // Returns m_stillScaleBuf on success, NULL if scaling is unneeded
    // (srcW==dstW && srcH==dstH) or failed (caller uses unscaled src).
    // isYuyv: true = YUYV single-plane, false = NV21M two-plane src.
    G800FExynosCameraBuffer* scaleForJpeg(G800FExynosCameraBuffer *src,
                                          int srcW, int srcH,
                                          int dstW, int dstH,
                                          bool isYuyv);

    // --- Flash state machine ---
    // The complete flash state machine runs in G800FPipeEngine (frame-based
    // in the FLITE thread). The device only delegates; the per-request
    // flashFired flag lives in QueuedRequest.
    // pumpPreviewWhileFlashMetering: serves preview requests while
    // the flash state machine is in the METERING state (waiting for AE convergence).
    // Prevents preview freeze and SCP queue overflow.
    int pumpPreviewWhileFlashMetering(int timeoutMs);

    int                    m_cameraId;
    const hw_module_t*     m_module;
    G800FPipeEngine*       m_pipeEngine;  // Layer 1+2 pipeline engine
    G800FFimcScaler*        m_fimcScaler;  // FIMC M2M hardware scaler (preview path)
    G800FFimcScaler*        m_fimcVideoScaler;  // FIMC M2M hardware scaler (video path)
    G800FFimcScaler*        m_analysisScaler;  // FIMC M2M node 2 (callback path)
    G800FFimcScaler*        m_stillScaler;     // FIMC M2M node 3 (still-JPEG scale)
    G800FExynosCameraBuffer* m_stillScaleBuf;  // scaled YUV for HW-JPEG input
    int                    m_stillScaleBufW, m_stillScaleBufH;
    int                    m_stillScaleSrcW, m_stillScaleSrcH;
    unsigned int           m_stillScaleSrcFmt;
    bool                   m_stillFimcFailed;  // FIMC still scaler failed
    bool                   m_fimcFailed;  // FIMC start failed — don't retry (NEON fallback)
    bool                   m_fimcVideoFailed;  // FIMC video scaler failed
    bool                   m_analysisFimcFailed; // FIMC analysis scaler failed
    G800FExynosCameraBuffer* m_analysisBounce;  // NV21 bounce dmabuf (FIMC dst)
    int                    m_analysisBounceW, m_analysisBounceH;
    camera3_device_t       m_hw;
    static camera3_device_ops_t s_ops;

    const camera3_callback_ops_t *m_callbackOps;
    camera3_stream_t*      m_previewStream;   // primary preview (IMPLEMENTATION_DEFINED/RGBA)
    camera3_stream_t*      m_analysisStream;  // secondary preview (YCbCr_420_888)
    camera3_stream_t*      m_videoStream;     // video recording (VIDEO_ENCODER or YCbCr_420_888)
    camera3_stream_t*      m_captureStream;   // JPEG BLOB
    int                    m_lastJpegSize;
    std::atomic<bool>      m_previewStarted;
    std::atomic<bool>      m_captureThreadRunning;
    std::atomic<bool>      m_flushing;
    std::atomic<bool>      m_workerBusy;

    mutable Mutex          m_lock;

    // static C-style ops -> C++ methods
    static int s_initialize(const camera3_device_t *d,
                            const camera3_callback_ops_t *callback_ops);
    static int s_configure_streams(const camera3_device_t *d,
                                   camera3_stream_configuration_t *stream_list);
    static const camera_metadata_t* s_construct_default_request_settings(
            const camera3_device_t *d, int type);
    static int s_process_capture_request(const camera3_device_t *d,
                                         camera3_capture_request_t *request);
    static int s_flush(const camera3_device_t *d);
    static void s_dump(const camera3_device_t *d, int fd);
    static int s_close(hw_device_t *device);

    G800FCamera2Device(const G800FCamera2Device&);
    G800FCamera2Device& operator=(const G800FCamera2Device&);
};

// CaptureThread: runs captureThreadLoop() in a separate thread.
class G800FCaptureThread : public Thread {
public:
    explicit G800FCaptureThread(G800FCamera2Device *dev)
        : Thread(false), m_dev(dev) {}
    virtual bool threadLoop() { return m_dev->captureThreadLoop(); }
private:
    G800FCamera2Device *m_dev;
};

// CaptureWorkerThread: runs captureWorkerLoop() for async still captures.
class G800FCaptureWorkerThread : public Thread {
public:
    explicit G800FCaptureWorkerThread(G800FCamera2Device *dev)
        : Thread(false), m_dev(dev) {}
    virtual bool threadLoop() { return m_dev->captureWorkerLoop(); }
private:
    G800FCamera2Device *m_dev;
};

// AnalysisThread: runs analysisWorkerLoop() for deferred callback-stream fills.
class G800FAnalysisThread : public Thread {
public:
    explicit G800FAnalysisThread(G800FCamera2Device *dev)
        : Thread(false), m_dev(dev) {}
    virtual bool threadLoop() { return m_dev->analysisWorkerLoop(); }
private:
    G800FCamera2Device *m_dev;
};

} // namespace android

#endif // G800F_CAMERA2_DEVICE_H
