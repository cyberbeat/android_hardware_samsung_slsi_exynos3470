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

#define LOG_TAG "G800FPipe"
#include <cutils/log.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "G800FPipe.h"

namespace android {

G800FPipe::G800FPipe(int pipeId, const char* name)
    : m_node(), m_videoNodeNum(-1), m_sensorId(-1),
      m_buffers(NULL), m_numBuffers(0), m_numPlanes(0),
      m_sharedPool(NULL), m_sharedLock(NULL),
      m_inputFrameQ(NULL), m_outputFrameQ(NULL),
      m_pipeId(pipeId), m_running(false), m_stopping(false),
      m_streamOn(false),
      m_retryGetBufferCount(0), m_numOfRunningFrame(0),
      m_eagainRetryCount(0)
{
    strncpy(m_name, name ? name : "G800FPipe", sizeof(m_name) - 1);
    m_name[sizeof(m_name) - 1] = '\0';
}

G800FPipe::~G800FPipe()
{
    ALOGI("%s: %s destructor", __FUNCTION__, m_name);
    destroy();
}

status_t G800FPipe::start()
{
    // start() only calls m_node->start() (VIDIOC_STREAMON).
    //   Does NOT start a thread — that is done separately by startThread().
    if (m_streamOn) return NO_ERROR;
    m_stopping = false;
    m_eagainRetryCount = 0;
    status_t err = m_node.start();
    if (err != NO_ERROR) {
        ALOGE("%s: %s streamon failed: %d", __FUNCTION__, m_name, err);
        return err;
    }
    m_streamOn = true;
    ALOGI("%s: %s streamon", __FUNCTION__, m_name);
    return NO_ERROR;
}

status_t G800FPipe::startThread()
{
    // startThread() launches the pipe thread.
    if (m_running) return NO_ERROR;
    m_running = true;
    run(m_name, PRIORITY_URGENT_DISPLAY);
    ALOGI("%s: %s thread started", __FUNCTION__, m_name);
    return NO_ERROR;
}

status_t G800FPipe::stop()
{
    // Improved order (bugfix for OS freeze):
    //   1. m_running = false
    //   2. signalStop() — m_stopping = true (thread checks this after dqbuf)
    //   3. node->stop()  (VIDIOC_STREAMOFF) — unblocks pending dqbuf in the thread
    //   4. mainThread->requestExitAndWait()  (join)
    //
    // Why signalStop before STREAMOFF:
    //   The old order (STREAMOFF first, then signalStop) had a race window:
    //   After STREAMOFF, dqbuf returns EINVAL, which is converted by
    //   G800FExynosCameraNode::dqBuf() to -EAGAIN (5ms sleep),
    //   and the thread keeps looping until signalStop() is finally called.
    //   In this window the thread can call dqbuf hundreds of times
    //   on a stopped stream → FIMC-IS kernel instability → OS freeze.
    //
    //   With signalStop() first: when dqbuf returns EINVAL→EAGAIN, the
    //   thread checks m_stopping=true and exits immediately.  At most 1
    //   dqbuf call after STREAMOFF, not hundreds.
    ALOGI("%s: %s stop() called (running=%d streamOn=%d) from tid=%d",
          __FUNCTION__, m_name, (int)m_running.load(), (int)m_streamOn.load(), (int)gettid());
    bool wasRunning = m_running;
    m_running = false;
    if (wasRunning) {
        signalStop();
        // Wake input queue in case we're blocked on pop_front
        if (m_inputFrameQ) m_inputFrameQ->wakeAll();
    }
    if (m_streamOn) {
        // Synchronization: m_v4l2Lock prevents the threadLoop from
        // executing a qbuf (m_putBuffer) while we do STREAMOFF.
        // The thread holds m_v4l2Lock during m_putBuffer — we wait here
        // until it is done, then STREAMOFF.  After that the thread can
        // see m_stopping=true and exit without another qbuf.
        Mutex::Autolock l(m_v4l2Lock);
        m_node.stop();
        m_streamOn = false;
    }
    if (wasRunning) {
        // Wait for thread to exit
        ALOGI("%s: %s joining thread...", __FUNCTION__, m_name);
        join();
        ALOGI("%s: %s thread joined", __FUNCTION__, m_name);
    }
    ALOGI("%s: %s stopped", __FUNCTION__, m_name);
    return NO_ERROR;
}

void G800FPipe::destroy()
{
    stop();
    if (m_node.isOpen()) {
        m_node.close();
    }
    releaseBuffers();
    // Free frame pool
    Mutex::Autolock l(m_framePoolLock);
    for (size_t i = 0; i < m_framePool.size(); i++) {
        delete m_framePool[i];
    }
    m_framePool.clear();
}

status_t G800FPipe::allocBuffers(int numBuffers, int numPlanes, const size_t* planeSizes)
{
    if (m_buffers) {
        ALOGW("%s: %s buffers already allocated, releasing first", __FUNCTION__, m_name);
        releaseBuffers();
    }
    m_numBuffers = numBuffers;
    m_buffers = new G800FExynosCameraBuffer*[numBuffers];
    for (int i = 0; i < numBuffers; i++) {
        m_buffers[i] = new G800FExynosCameraBuffer();
        status_t err = m_buffers[i]->alloc(numPlanes, planeSizes,
                                            (1 << 5) /* ION_HEAP_EXYNOS_MASK */, 0);
        if (err != NO_ERROR) {
            ALOGE("%s: %s buffer %d alloc failed: %d", __FUNCTION__, m_name, i, err);
            // Clean up partial allocation
            for (int j = 0; j <= i; j++) {
                if (m_buffers[j]) {
                    m_buffers[j]->release();
                    delete m_buffers[j];
                    m_buffers[j] = NULL;
                }
            }
            delete[] m_buffers;
            m_buffers = NULL;
            m_numBuffers = 0;
            return err;
        }
    }
    ALOGI("%s: %s allocated %d buffers (%d planes each)",
          __FUNCTION__, m_name, numBuffers, numPlanes);
    return NO_ERROR;
}

status_t G800FPipe::allocBuffersMmap(int numBuffers, int numPlanes)
{
    /* MMAP mode: kernel allocates buffers via vb2-ION after reqbufs(MMAP).
     * We call QUERYBUF + mmap for each buffer to get user pointers.
     * The kernel retains kvaddr/dvaddr, which ISP/SCC/SCP nodes need
     * for shot_ext/stream metadata.  This is the MMAP buffer mode. */
    if (m_buffers) {
        ALOGW("%s: %s buffers already allocated, releasing first", __FUNCTION__, m_name);
        releaseBuffers();
    }
    m_numBuffers = numBuffers;
    m_buffers = new G800FExynosCameraBuffer*[numBuffers];
    int fd = m_node.getFd();
    int bufType = (int)m_node.getBufferType();
    for (int i = 0; i < numBuffers; i++) {
        m_buffers[i] = new G800FExynosCameraBuffer();
        status_t err = m_buffers[i]->allocMmap(fd, i, numPlanes, bufType);
        if (err != NO_ERROR) {
            ALOGE("%s: %s buffer %d allocMmap failed: %d", __FUNCTION__, m_name, i, err);
            for (int j = 0; j <= i; j++) {
                if (m_buffers[j]) {
                    m_buffers[j]->release();
                    delete m_buffers[j];
                    m_buffers[j] = NULL;
                }
            }
            delete[] m_buffers;
            m_buffers = NULL;
            m_numBuffers = 0;
            return err;
        }
    }
    ALOGI("%s: %s allocated %d MMAP buffers (%d planes each)",
          __FUNCTION__, m_name, numBuffers, numPlanes);
    return NO_ERROR;
}

status_t G800FPipe::allocBuffersDmabuf(int numBuffers, int numPlanes,
                                        const size_t* planeSizes)
{
    /* DMABUF mode: userspace allocates non-cacheable ION buffers and
     * exports them as dma_buf fds.  The fds are passed to V4L2 with
     * V4L2_MEMORY_DMABUF.  The kernel's vb2 DMABUF path performs
     * cache sync on each qbuf/dqbuf — fixing the FIMC-IS SCP cache
     * coherency issue. */
    if (m_buffers) {
        ALOGW("%s: %s buffers already allocated, releasing first", __FUNCTION__, m_name);
        releaseBuffers();
    }
    m_numBuffers = numBuffers;
    m_buffers = new G800FExynosCameraBuffer*[numBuffers];
    for (int i = 0; i < numBuffers; i++) {
        m_buffers[i] = new G800FExynosCameraBuffer();
        status_t err = m_buffers[i]->allocDmabuf(numPlanes, planeSizes,
                                                  (1 << 5), 0);
        if (err != NO_ERROR) {
            ALOGE("%s: %s buffer %d allocDmabuf failed: %d",
                  __FUNCTION__, m_name, i, err);
            for (int j = 0; j <= i; j++) {
                if (m_buffers[j]) {
                    m_buffers[j]->release();
                    delete m_buffers[j];
                    m_buffers[j] = NULL;
                }
            }
            delete[] m_buffers;
            m_buffers = NULL;
            m_numBuffers = 0;
            return err;
        }
    }
    ALOGI("%s: %s allocated %d DMABUF buffers (%d planes each)",
          __FUNCTION__, m_name, numBuffers, numPlanes);
    return NO_ERROR;
}

void G800FPipe::releaseBuffers()
{
    if (!m_buffers) return;
    for (int i = 0; i < m_numBuffers; i++) {
        if (m_buffers[i]) {
            m_buffers[i]->release();
            delete m_buffers[i];
            m_buffers[i] = NULL;
        }
    }
    delete[] m_buffers;
    m_buffers = NULL;
    m_numBuffers = 0;
    // Also release kernel-side V4L2 buffers (REQBUFS(0)).
    // Must be called after streamoff. Without this, the next setupPipe()
    // reqbufs(N) fails with EBUSY because the kernel still has N buffers
    // allocated from the previous session.
    m_node.releaseBuffers();
}

G800FFrame* G800FPipe::allocFrame(int bufferIndex)
{
    // Pipeline model: use shared pool if set
    Vector<G800FFrame*>* pool = m_sharedPool ? m_sharedPool : &m_framePool;
    Mutex* lock = m_sharedLock ? m_sharedLock : &m_framePoolLock;

    Mutex::Autolock l(*lock);
    G800FFrame* frame = NULL;
    if (pool->isEmpty()) {
        frame = new G800FFrame();
    } else {
        frame = pool->top();
        pool->pop();
        *frame = G800FFrame(); // reset to defaults
    }
    frame->bufferIndex = bufferIndex;
    frame->buffer = getBuffer(bufferIndex);
    frame->producerPipeId = m_pipeId;
    return frame;
}

void G800FPipe::recycleFrame(G800FFrame* frame)
{
    if (!frame) return;
    // Free owned preview data copy (conformant: SCP buffer was immediately
    // requeued, the data copy belongs to the frame and must be freed).
    if (frame->ownsPreviewData) {
        if (frame->previewData[0]) free(frame->previewData[0]);
        if (frame->previewData[1]) free(frame->previewData[1]);
        frame->previewData[0] = NULL;
        frame->previewData[1] = NULL;
        frame->ownsPreviewData = false;
    }

    // Pipeline model: use shared pool if set
    Vector<G800FFrame*>* pool = m_sharedPool ? m_sharedPool : &m_framePool;
    Mutex* lock = m_sharedLock ? m_sharedLock : &m_framePoolLock;

    Mutex::Autolock l(*lock);
    pool->push(frame);
}

bool G800FPipe::threadLoop()
{
    // Main thread loop:
    //
    //   if (m_flagTryStop) { usleep(5000); return true; }
    //   ret = m_getBuffer();
    //   if (m_flagTryStop) { log; return false; }
    //   if (ret < 0) log "m_getBuffer fail";
    //   // Retry loop
    //   for (i = 0; i < m_retryGetBufferCount; i++) {
    //       log "retryGetBufferCount(N)";
    //       m_getBuffer();
    //       m_putBuffer();
    //       if (m_flagTryStop) return false;
    //       if (putRet == -ETIMEDOUT) goto done;
    //   }
    //   m_retryGetBufferCount = 0;
    //   // Input queue drain or single m_putBuffer()
    //   if (m_numOfRunningFrame < 2) {
    //       count = inputFrameQ->size();
    //       do { m_putBuffer(); count--; } while (count > 0);
    //   } else {
    //       m_putBuffer();
    //   }
    // done:
    //   return true;
    //
    // The retry loop is important for flash/reprocessing: when a frame
    // is diverted for reprocessing, the caller sets m_retryGetBufferCount
    // to fetch additional frames and keep the pipeline flow going.

    if (m_stopping) {
        ALOGI("%s: %s threadLoop exiting (stopping)", __FUNCTION__, m_name);
        usleep(5000);
        return false; // exit thread
    }

    // 1. First m_getBuffer() — blocks in dqbuf until a buffer arrives.
    // On -EAGAIN (queue empty / buffer starvation): NO short-circuit!
    // m_putBuffer() must still be called so the pipeline flow continues.
    // This is critical for ISP: ISP has no initially queued buffers
    // (kernel requires STREAMON first).  ISP m_getBuffer() returns -EAGAIN
    // as long as m_ispInFlightCount==0.  Only m_putBuffer() (qBufIsp)
    // increments inFlightCount.  If we abort here, ISP would never get
    // buffers → pipeline deadlock.
    //
    // BUT: if m_numOfRunningFrame > 0 (ISP already has buffers in-flight),
    // then EAGAIN is not buffer starvation but "buffer not yet finished
    // processing".  In this case we must NOT fall through to putBuffer —
    // otherwise inFlight would exceed the limit (2).
    // This was the cause of the OS freeze: inFlight=3 → ISP firmware
    // hangs → DQBUF EINVAL → busy loop → freeze.
    status_t getErr = m_getBuffer();
    if (m_stopping) {
        ALOGI("%s: %s threadLoop exiting after getBuffer (stopping)", __FUNCTION__, m_name);
        return false;
    }
    // Bugfix for OS freeze: -EINVAL can have two causes:
    //   a) Stream is stopped (m_stopping=true) → exit thread
    //   b) ISP kernel has no finished frames yet (especially with tight
    //      qbuf→dqbuf timing) → NO thread exit, but retry!
    //
    // Previously, EINVAL was always interpreted as "stopped stream" and
    // the thread would exit.  This led to a pipeline deadlock: ISP thread
    // exits after 5 frames → no further ISP DQBUF → SCP runs empty →
    // buffer starvation → app freeze.
    //
    // Now: treat EINVAL as exit only when m_stopping=true.
    // Otherwise treat like EAGAIN (sleep briefly, retry).
    if (getErr == -EINVAL && m_stopping) {
        ALOGI("%s: %s threadLoop exiting (EINVAL from stopped stream)",
              __FUNCTION__, m_name);
        return false;
    }
    if (getErr == -EINVAL) {
        // EINVAL on a running stream: ISP kernel not ready yet.
        // Sleep briefly and retry — do not exit!
        ALOGW("%s: %s m_getBuffer EINVAL (stream running, retrying)",
              __FUNCTION__, m_name);
        usleep(5000); // 5ms
        return true; // continue loop
    }
    bool gotBuffer = (getErr == NO_ERROR);
    bool putOK = false;
    if (gotBuffer) m_eagainRetryCount = 0; // watchdog reset on ISP output
    if (getErr == -EAGAIN) {
        // Buffer starvation — normal when all buffers are downstream
        // or (ISP) no buffers queued yet.
        //
        // IMPORTANT (bugfix for ISP stall with 2 in-flight buffers):
        // The Exynos 3470 ISP firmware stalls when >1 buffer is
        // in-flight simultaneously.  In live logs: with inFlight=2 the
        // ISP stops responding to DQBUF after 5 frames → black preview.
        // With inFlight=1 (as in the first session) the pipeline runs
        // stably.
        //
        // Therefore: on EAGAIN + numOfRunning > 0, do NOT call m_putBuffer
        // (equivalent to the old goto-done behavior).  The ISP processes
        // the one in-flight buffer, m_getBuffer returns it on the next
        // iteration (numOfRunning→0), and only then is the next buffer
        // queued.
        //
        // EXCEPTION (watchdog): if numOfRunning > 0 and the ISP has not
        // produced output for >5 seconds (m_eagainRetryCount > 500 at
        // 10ms sleep), we force m_putBuffer to queue a new buffer.
        // This prevents the firmware-error deadlock: if the ISP consumes
        // a buffer but responds with EINVAL (firmware reset), numOfRunning
        // stays at 1 and without the watchdog the pipeline would stall.
        usleep(5000); // 5ms — reduces polling frequency
        if (m_numOfRunningFrame > 0) {
            m_eagainRetryCount++;
            if (m_eagainRetryCount > 500) {
                // Watchdog: 5s without ISP output → try new buffer
                ALOGW("%s: %s EAGAIN watchdog: %d retries, forcing putBuffer",
                      __FUNCTION__, m_name, m_eagainRetryCount);
                m_eagainRetryCount = 0;
                // Fall through to m_putBuffer (below)
            } else {
                goto done; // 1-in-flight limit: no putBuffer
            }
        }
        // Fall through to m_putBuffer (numOfRunning==0 or watchdog)
    } else if (getErr != NO_ERROR) {
        ALOGW("%s: %s m_getBuffer err=%d", __FUNCTION__, m_name, getErr);
    }

    // 2. Retry loop (m_retryGetBufferCount)
    // Only useful if we got a buffer.
    if (gotBuffer) {
        for (int i = 0; i < m_retryGetBufferCount; i++) {
            if (m_stopping) return false;
            //ALOGI("%s: %s retryGetBufferCount(%d) retry=%d",
            //      __FUNCTION__, m_name, m_retryGetBufferCount, i);
            m_getBuffer();
            // m_v4l2Lock protects m_putBuffer (qbuf) from STREAMOFF in stop().
            // stop() waits for this lock before doing STREAMOFF.
            Mutex::Autolock l(m_v4l2Lock);
            if (m_stopping) return false;
            status_t putErr = m_putBuffer();
            if (m_stopping) return false;
            if (putErr == -ETIMEDOUT) goto done;
        }
    }
    m_retryGetBufferCount = 0;

    // 3. Input queue drain — but limited to 1 in-flight buffer!
    // For ISP: this is the only way to queue buffers (qBufIsp).
    //
    // IMPORTANT: The Exynos 3470 ISP firmware stalls when >1 buffer
    // is in-flight simultaneously (confirmed in live logs: inFlight=2
    // → ISP stops after 5 frames).  Therefore we limit to
    // m_numOfRunningFrame < 1 (max. 1 buffer in-flight).
    //
    // This matches the expected behavior: m_putBuffer internally checks
    // m_numOfRunningFrame and returns immediately when the limit is
    // reached.  Since our m_putBuffer does not do this, we limit here.
    if (m_numOfRunningFrame < 1) {
        int count = (m_inputFrameQ) ? m_inputFrameQ->size() : 0;
        while (count > 0 && m_numOfRunningFrame < 1) {
            // m_v4l2Lock protects m_putBuffer (qbuf) from STREAMOFF in stop().
            Mutex::Autolock l(m_v4l2Lock);
            if (m_stopping) return false;
            status_t putErr = m_putBuffer();
            if (m_stopping) return false;
            if (putErr == -ETIMEDOUT) break;
            if (putErr == -EAGAIN) break;  // no free ISP buffer
            if (putErr == NO_ERROR) putOK = true;
            count--;
        }
    }
    // else: ISP already has 1 buffer in-flight.  Wait for m_getBuffer
    // (ISP output), which decrements m_numOfRunningFrame.  The watchdog
    // above (EAGAIN + numOfRunning > 0 + 500 retries) prevents deadlocks
    // on firmware errors.

    // 4. Anti-busy-loop: if neither getBuffer nor putBuffer produced
    //    anything, sleep briefly.  Prevents 100% CPU spin when the ISP
    //    input queue is empty and there are no in-flight buffers.
    //    10ms instead of 2ms — reduces ioctl storm during buffer starvation.
    if (!gotBuffer && !putOK) {
        usleep(10000); // 10ms
    }

done:
    return true; // continue loop
}

} // namespace android
