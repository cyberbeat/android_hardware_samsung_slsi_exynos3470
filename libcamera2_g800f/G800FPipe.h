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

#ifndef G800F_PIPE_H
#define G800F_PIPE_H

#include <utils/Errors.h>
#include <utils/Thread.h>
#include <utils/Vector.h>
#include <atomic>
#include "G800FExynosCameraNode.h"
#include "G800FExynosCameraBuffer.h"
#include "G800FFrameQueue.h"
#include <fimc-is-metadata.h>

namespace android {

/*
 * Pipe IDs.
 *
 * m_pipes[] array slot → logical pipe ID mapping:
 *   m_pipes[2]  → FLITE  = 0
 *   m_pipes[4]  → ISP    = 2   (3AA_ISP on Exynos 3470)
 *   m_pipes[7]  → SCC    = 5
 *   m_pipes[8]  → SCP    = 6
 *   m_pipes[9]  → GSC    = 7
 *   m_pipes[10] → GSC_VIDEO = 8
 *   m_pipes[11] → GSC_PICTURE = 9
 *   m_pipes[12] → JPEG   = 10
 *
 * Reprocessing logical pipe IDs:
 *   ISP_REPROC     = 0xCA
 *   SCC_REPROC     = 0xCB
 *   SCP_REPROC     = 0xCC
 *   GSC_REPROC     = 0xCD
 *   JPEG_REPROC    = 0xCE
 */
enum G800FPipeId {
    PIPE_FLITE       = 0,    // Sensor frontend (Bayer capture, continuous 3280x2458)
    PIPE_ISP         = 2,    // ISP (Bayer → YUV, 3A processing)
    PIPE_SCC         = 5,    // Scaler Capture (full-res YUV 3264x2448)
    PIPE_SCP         = 6,    // Scaler Preview (preview YUV, e.g. 960x720)
    PIPE_GSC         = 7,    // GSC scaler (future)
    PIPE_GSC_VIDEO   = 8,    // GSC for video scaling (future)
    PIPE_GSC_PICTURE = 9,    // GSC for still picture scaling (future)
    PIPE_JPEG        = 10,   // JPEG hardware encoder (future)
    MAX_NUM_PIPES    = 11,

    // Reprocessing pipe IDs (separate logical pipeline)
    PIPE_REPROC_ISP     = 0xCA,
    PIPE_REPROC_SCC     = 0xCB,
    PIPE_REPROC_SCP     = 0xCC,
    PIPE_REPROC_GSC     = 0xCD,
    PIPE_REPROC_JPEG    = 0xCE,
};

/*
 * G800FPipe — base class for pipe threads.
 *
 * Each pipe wraps one V4L2 node and runs a thread that:
 *   1. m_getBuffer() — dequeue a processed frame from the V4L2 node
 *   2. m_putBuffer() — queue a buffer to the V4L2 node for processing
 *
 * The thread loop runs the classic dequeue/queue cycle:
 *   - Check stop flag
 *   - Call m_getBuffer() (dequeue from hardware)
 *   - Call m_putBuffer() (queue next input to hardware)
 *   - Loop
 *
 * Pipes are connected via G800FFrameQueue:
 *   - m_inputFrameQ: frames TO be processed (from upstream pipe)
 *   - m_outputFrameQ: frames FROM processing (to downstream pipe)
 *
 * Buffer ownership:
 *   - The pipe owns its buffer pool (m_buffers[])
 *   - When a frame is pushed to m_outputFrameQ, ownership transfers
 *     to the downstream pipe
 *   - When a frame is popped from m_inputFrameQ, the pipe reclaims
 *     its buffer and queues it back to the V4L2 node
 */
class G800FPipe : public Thread {
public:
    G800FPipe(int pipeId, const char* name);
    virtual ~G800FPipe();

    // --- Lifecycle ---
    // Create: open V4L2 node, configure format
    virtual status_t create(int videoNodeNum, int sensorId) = 0;
    // setupPipe: setSize, setFormat, reqBuffers, queue initial buffers
    virtual status_t setupPipe(int w, int h, int pixFmt,
                               int numPlanes, int numBuffers,
                               v4l2_buf_type bufType,
                               v4l2_memory memory = V4L2_MEMORY_MMAP) = 0;
    // Start V4L2 streaming only (VIDIOC_STREAMON), no thread.
    virtual status_t start();
    // Start worker thread only (separate from STREAMON).
    virtual status_t startThread();
    // Stop thread + streaming
    virtual status_t stop();
    // Destroy: close node, release buffers
    virtual void destroy();

    // --- Queue management ---
    void setInputFrameQ(G800FFrameQueue* q)  { m_inputFrameQ = q; }
    void setOutputFrameQ(G800FFrameQueue* q) { m_outputFrameQ = q; }
    G800FFrameQueue* getInputFrameQ()  { return m_inputFrameQ; }
    G800FFrameQueue* getOutputFrameQ() { return m_outputFrameQ; }

    // --- Buffer pool ---
    // Allocate numBuffers ION buffers with the given plane sizes (USERPTR mode).
    status_t allocBuffers(int numBuffers, int numPlanes, const size_t* planeSizes);
    // Allocate numBuffers MMAP buffers via QUERYBUF+mmap (kernel-allocated).
    // Call after reqBuffers(MMAP).  The kernel owns kvaddr/dvaddr.
    status_t allocBuffersMmap(int numBuffers, int numPlanes);
    // Allocate numBuffers DMABUF buffers (userspace ION, non-cacheable).
    // Call after reqBuffers(DMABUF).  Pass dma_buf fds to V4L2 qbuf.
    status_t allocBuffersDmabuf(int numBuffers, int numPlanes,
                                 const size_t* planeSizes);
    void releaseBuffers();
    G800FExynosCameraBuffer* getBuffer(int index) {
        return (index >= 0 && index < m_numBuffers) ? m_buffers[index] : NULL;
    }
    int getNumBuffers() const { return m_numBuffers; }

    // --- Node access ---
    G800FExynosCameraNode* getNode() { return &m_node; }
    int getPipeId() const { return m_pipeId; }
    const char* getName() const { return m_name; }
    bool isRunning() const { return m_running; }

    // --- Thread control ---
    void signalStop() { m_running = false; m_stopping = true; }
    void signalStart() { m_running = true; m_stopping = false; }

    // --- Frame allocation ---
    // Allocate a G800FFrame from the pool.  Returns NULL if pool exhausted.
    // Pipeline model: if a shared pool is set (via setSharedFramePool),
    // it is used — the frame flows as a single object through all pipes.
    G800FFrame* allocFrame(int bufferIndex);
    // Recycle a frame back to the pool (does NOT delete it).
    void recycleFrame(G800FFrame* frame);
    // Set a shared frame pool (pipeline model: one pool for all pipes).
    void setSharedFramePool(Vector<G800FFrame*>* pool, Mutex* lock) {
        m_sharedPool = pool;
        m_sharedLock = lock;
    }

protected:
    // Thread loop — calls m_getBuffer() + m_putBuffer()
    virtual bool threadLoop();

    // Subclass-specific: dequeue from V4L2, build frame, push to output queue
    virtual status_t m_getBuffer() = 0;
    // Subclass-specific: pop frame from input queue, queue buffer to V4L2
    virtual status_t m_putBuffer() = 0;

    // V4L2 node
    G800FExynosCameraNode m_node;
    int m_videoNodeNum;
    int m_sensorId;

    // Buffer pool
    G800FExynosCameraBuffer** m_buffers;
    int m_numBuffers;
    int m_numPlanes;  // V4L2 plane count (data planes + SPARE), set by setupPipe()

    // Frame pool (recycled G800FFrame objects)
    Vector<G800FFrame*> m_framePool;
    Mutex m_framePoolLock;

    // Pipeline model: shared frame pool (owned by PipeEngine).
    // When set, allocFrame/recycleFrame use this pool
    // instead of the local m_framePool. The frame flows as a single object
    // through FLITE → ISP → SCP → HAL3 and is returned at the end.
    Vector<G800FFrame*>* m_sharedPool;
    Mutex* m_sharedLock;

    // Frame queues (connected to other pipes)
    G800FFrameQueue* m_inputFrameQ;
    G800FFrameQueue* m_outputFrameQ;

    // Identity
    int m_pipeId;
    char m_name[32];

    // Thread state
    std::atomic<bool> m_running;
    std::atomic<bool> m_stopping;

    // V4L2 stream state
    std::atomic<bool> m_streamOn;

    // Synchronizes threadLoop (m_putBuffer/qbuf) with stop() (STREAMOFF).
    // Prevents qbuf after STREAMOFF → kernel corruption → OS freeze.
    // stop() holds this lock during STREAMOFF, threadLoop holds it
    // during m_putBuffer. This ensures qbuf can never happen after STREAMOFF.
    Mutex m_v4l2Lock;

    // Retry loop parameters:
    // m_retryGetBufferCount: number of retry attempts for m_getBuffer()+m_putBuffer()
    //   after the first m_getBuffer(). FLITE sets this value externally
    //   (e.g. by flash/reprocessing trigger). 0 = no retry.
    // m_numOfRunningFrame: number of frames currently being processed.
    //   If < 2: drain input queue (m_putBuffer() for every input frame).
    //   Otherwise: single m_putBuffer().
    int m_retryGetBufferCount;
    int m_numOfRunningFrame;
    // Watchdog counter: how many consecutive EAGAIN retries with numOfRunning>0.
    // After >500 retries (≈5s), m_putBuffer is forced to prevent firmware error
    // deadlocks. Reset on successful m_getBuffer.
    int m_eagainRetryCount;

private:
    G800FPipe(const G800FPipe&);
    G800FPipe& operator=(const G800FPipe&);
};

} // namespace android

#endif // G800F_PIPE_H
