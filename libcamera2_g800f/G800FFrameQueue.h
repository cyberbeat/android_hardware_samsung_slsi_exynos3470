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

#ifndef G800F_FRAME_QUEUE_H
#define G800F_FRAME_QUEUE_H

#include <utils/Errors.h>
#include <utils/Mutex.h>
#include <utils/Condition.h>
#include <utils/Timers.h>
#include <utils/Vector.h>
#include "G800FExynosCameraBuffer.h"  // linux/videodev2.h → _LINUX_TYPES_H first
#include <fimc-is-metadata.h>         // (needs the kernel types guard)

namespace android {

/*
 * G800FFrame — lightweight frame descriptor passed between pipes.
 *
 * Each frame carries buffer references and metadata for one pipeline
 * stage — one frame = one buffer + metadata, no entity graph.
 *
 * Ownership: the pipe that creates the frame owns it.  When a frame is
 * pushed to a queue, ownership transfers to the receiver.  The receiver
 * must call recycle() or release() when done.
 */
struct G800FFrame {
    // Buffer index in the owning pipe's buffer pool
    int             bufferIndex;
    // Buffer pointer (may be NULL if previewData is used instead)
    G800FExynosCameraBuffer* buffer;
    // Metadata pointer (shot_ext for FLITE/ISP, camera2_stream for SCC/SCP)
    void*           meta;
    size_t          metaSize;
    // Sensor frame count (from shot_ext->shot.dm.request.frameCount)
    uint32_t        frameCount;
    // Sensor timestamp (ns)
    uint64_t        timestamp;
    // Which pipe produced this frame (for debugging/tracking)
    int             producerPipeId;
    // True if this frame is a capture candidate (selected by FrameSelector)
    bool            captureCandidate;
    // Flash state at time of capture (aeflashMode)
    int             aeflashMode;
    // AF/AE/AWB state snapshot (from ISP dm)
    int             afState;
    int             aeState;
    int             awbState;
    int             sensitivity;
    int64_t         exposureTime;

    // Preview data copy (SCP buffer is immediately requeued,
    // the data is copied into a separate malloc buffer).
    // previewData[0] = Y plane, previewData[1] = VU plane.
    // previewStride[0] = Y stride, previewStride[1] = VU stride.
    // If ownsPreviewData=true, the memory is freed in recycle/release.
    void*           previewData[2];
    size_t          previewDataSize[2];
    int             previewStride[2];
    int             previewWidth;
    int             previewHeight;
    bool            ownsPreviewData;

    // Direct V4L2 path (eliminates malloc+memcpy):
    // If ownsScpBuffer=true, frame->buffer points to the SCP V4L2 buffer
    // (G800FExynosCameraBuffer with DMABUF fds + mapped vaddr).
    // The V4L2 buffer is NOT requeued in m_getBuffer — only in
    // releasePreviewFrame().  servePreviewRequest reads directly from
    // buf->planeVaddr(0/1) via copyPreviewToStream().
    // scpBufferIndex = V4L2 buffer index for requeueScpBuffer().
    int             scpBufferIndex;
    bool            ownsScpBuffer;

    // Sensor-node metadata snapshot: filled at FLITE dequeue from the bayer
    // buffer's own shot_ext — BEFORE the ISP dequeue path overwrites
    // fliteShot->shot.dm with the ISP dm.  Reprocessing feeds exactly this
    // per-frame dm/udm to ISP[1]; for a flash-lit frame it may carry the
    // firmware's aeflashMode=CAPTURE echo, which the ISP needs to select
    // the flash-metered WB.
    camera2_dm      sensorDm;
    camera2_udm     sensorUdm;
    bool            hasSensorMeta;

    G800FFrame()
        : bufferIndex(-1), buffer(NULL), meta(NULL), metaSize(0),
          frameCount(0), timestamp(0), producerPipeId(-1),
          captureCandidate(false), aeflashMode(0),
          afState(0), aeState(0), awbState(0),
          sensitivity(0), exposureTime(0),
          previewData{NULL, NULL}, previewDataSize{0, 0},
          previewStride{0, 0}, previewWidth(0), previewHeight(0),
          ownsPreviewData(false),
          scpBufferIndex(-1), ownsScpBuffer(false),
          hasSensorMeta(false) {}
};

/*
 * G800FFrameQueue — thread-safe bounded queue of G800FFrame*.
 *
 * Bounded blocking queue:
 *   - push_back(): non-blocking, signals waiting consumer
 *   - pop_front(): blocking with optional timeout
 *   - push_front(): re-queue at head (for retry/reject)
 *
 * Used to connect pipes: producer pushes, consumer pops.
 * The queue does NOT own the frames — it just passes pointers.
 * The consumer is responsible for recycling or releasing frames.
 */
class G800FFrameQueue {
public:
    G800FFrameQueue();
    ~G800FFrameQueue();

    // Push frame to tail.  Non-blocking.  Wakes one waiting consumer.
    // Returns NO_ERROR on success, or -ENOSPC if queue is full.
    status_t push_back(G800FFrame* frame);

    // Push frame to head (high priority).  Non-blocking.
    status_t push_front(G800FFrame* frame);

    // Pop frame from head.  Blocks up to timeoutMs milliseconds.
    // Returns NO_ERROR and sets *frame, or -ETIMEDOUT if no frame arrives.
    status_t pop_front(G800FFrame** frame, int timeoutMs);

    // Pop frame from head.  Non-blocking.
    // Returns NO_ERROR and sets *frame, or -EAGAIN if empty.
    status_t try_pop_front(G800FFrame** frame);

    // Wake all waiting consumers (used for shutdown/flush).
    void wakeAll();

    // Clear the queue (does NOT release frames — caller must handle ownership).
    void clear();

    // Number of frames currently in the queue.
    int size() const;

    // Maximum queue capacity.
    int getCapacity() const { return m_capacity; }

    // Set maximum queue capacity.  Default = 8 (generous for burst).
    void setCapacity(int max);

private:
    mutable Mutex   m_lock;
    Condition       m_cond;
    Vector<G800FFrame*> m_queue;
    int             m_capacity;

    G800FFrameQueue(const G800FFrameQueue&);
    G800FFrameQueue& operator=(const G800FFrameQueue&);
};

} // namespace android

#endif // G800F_FRAME_QUEUE_H
