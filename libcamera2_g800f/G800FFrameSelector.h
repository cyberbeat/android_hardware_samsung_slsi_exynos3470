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

#ifndef G800F_FRAME_SELECTOR_H
#define G800F_FRAME_SELECTOR_H

#include <utils/Errors.h>
#include <utils/Mutex.h>
#include <utils/Condition.h>
#include <utils/Vector.h>
#include "G800FFrameQueue.h"
#include <fimc-is-metadata.h>

namespace android {

/*
 * G800FFrameSelector — Bayer frame selection.
 *
 * Frame selection API:
 *
 *   selectFrames(count, pipeId, maxFrame, waitTime)
 *
 * Call site in getBayerBuffer():
 *   selectFrames(bufferCount, 0, false, 0x1e)
 *   - pipeId=0 → FLITE (Bayer frame from running preview stream)
 *   - waitTime=0x1e (30) → ~200ms timeout
 *   - maxFrame=false → no frame count limit
 *
 * The FrameSelector is continuously fed with FLITE frames
 * (offerFrame()). On selectFrames() it waits up to 200ms for
 * sufficient candidates and picks the best one by 3A criteria.
 *
 * For flash capture, the ISP takes the frame with firingStable==1;
 * the FrameSelector is only used for capture without flash.
 */
class G800FFrameSelector {
public:
    G800FFrameSelector();
    ~G800FFrameSelector();

    // --- API ---

    // selectFrames — selects 'count' frames from pipe 'pipeId'.
    // Blocks up to 'waitTime' units (~6.67ms per unit → 200ms at 0x1e).
    // pipeId=0 → FLITE (Bayer). maxFrame=true → only the newest frame.
    // Returns the number of selected frames, or 0 on timeout.
    // The selected frames are placed in 'outFrames'.
    // The caller takes ownership of the returned frames.
    int selectFrames(int count, int pipeId, bool maxFrame,
                     int waitTime,
                     Vector<G800FFrame*>& outFrames);

    // Convenience for a single frame (most common case):
    // Equivalent to selectFrames(1, 0, false, 0x1e).
    // Returns the frame or NULL on timeout.
    G800FFrame* selectOneFrame(int pipeId = 0,
                               int waitTime = 0x1e);

    // --- Frame input (called from FLITE pipe thread) ---

    // Offers a FLITE frame for selection.
    // The selector stores the frame internally as a candidate.
    // For flash capture with setFrameSkipCount > 0, frames are
    // discarded until the skip count is reached.
    // Returns: true if frame accepted, false if discarded (skip).
    bool offerFrame(G800FFrame* frame);

    // Return non-selected candidates (for FLITE requeue).
    // Called when selection is cancelled.
    void cancel(G800FFrameQueue* recycleQ);

    // --- Control ---

    // Reset for a new capture sequence
    void reset();

    // Set flash mode: prefers frames with aeflashMode=CAPTURE
    void setFlashCapture(bool flash) { m_flashCapture = flash; }
    bool isFlashCapture() const { return m_flashCapture; }

    // Main flash armed: set by armFlashCapture().
    // The FLITE thread checks this before offerFrame(): for flash capture,
    // frames before armFlashCapture() are discarded (preflash frames).
    void setMainFlashArmed(bool armed) { m_mainFlashArmed = armed; }
    bool isMainFlashArmed() const { return m_mainFlashArmed; }

    // Number of sensor frames to discard before capture selection.
    // Independent from the preview skip during main flash.
    void setFrameSkipCount(int count) { m_frameSkipCount = count; }
    int getFrameSkipCount() const { return m_frameSkipCount; }

    // Has selected frames ready?
    bool hasSelectedFrame() const;

private:
    mutable Mutex m_lock;
    Condition m_cond;  // Signals new candidates

    Vector<G800FFrame*> m_candidates;  // Available FLITE frames
    Vector<G800FFrame*> m_selected;    // Selected frames (for caller)

    bool m_flashCapture;
    volatile bool m_mainFlashArmed;  // True after armFlashCapture()
    int m_frameSkipCount;   // For flash: skip N frames (value: 8)
    int m_framesSkipped;    // Frames already skipped

    // Score a frame by 3A criteria (higher = better)
    int scoreFrame(G800FFrame* frame) const;

    // Pick the best frame from candidates (under lock)
    G800FFrame* pickBest_locked();

    // Return non-selected candidates to recycleQ
    void releaseCandidates_locked(G800FFrameQueue* recycleQ);

    G800FFrameSelector(const G800FFrameSelector&);
    G800FFrameSelector& operator=(const G800FFrameSelector&);
};

} // namespace android

#endif // G800F_FRAME_SELECTOR_H
