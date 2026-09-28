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

#define LOG_TAG "G800FFrameSelector"
#include <log/log.h>
#include <utils/Timers.h>
#include "G800FFrameSelector.h"

namespace android {

G800FFrameSelector::G800FFrameSelector()
    : m_flashCapture(false),
      m_mainFlashArmed(false),
      m_frameSkipCount(0),
      m_framesSkipped(0)
{
}

G800FFrameSelector::~G800FFrameSelector()
{
    reset();
}

void G800FFrameSelector::reset()
{
    Mutex::Autolock l(m_lock);
    // Candidates are NOT freed — the caller must call cancel()
    // to return them to the FLITE queue.
    m_candidates.clear();
    m_selected.clear();
    m_flashCapture = false;
    m_mainFlashArmed = false;
    m_frameSkipCount = 0;
    m_framesSkipped = 0;
}

int G800FFrameSelector::scoreFrame(G800FFrame* frame) const
{
    if (!frame) return -1;
    int score = 0;

    // AF state: FOCUSED_LOCKED(2) or FOCUSED_LOCKED(3) → good for capture
    // AF_STATE_INACTIVE(0), AF_STATE_PASSIVE_SCAN(1) → not ready
    if (frame->afState == 2 || frame->afState == 3) {
        score += 100;
    } else if (frame->afState == 0 || frame->afState == 1) {
        score -= 50;
    }

    // AE state: CONVERGED(2) or LOCKED(3) → good
    // AE_STATE_FLASH_REQUIRED(4) → ok but needs flash
    // AE_STATE_INACTIVE(0) or SEARCHING(1) → not ready
    if (frame->aeState == 2 || frame->aeState == 3) {
        score += 80;
    } else if (frame->aeState == 4) {
        score += 40;
    } else if (frame->aeState == 0 || frame->aeState == 1) {
        score -= 40;
    }

    // Flash capture: prefer frames with aeflashMode=CAPTURE (6)
    if (m_flashCapture) {
        if (frame->aeflashMode == 6) { // AA_FLASHMODE_CAPTURE
            score += 200;
        } else if (frame->aeflashMode == 5) { // AA_FLASHMODE_AUTO
            score += 50;
        }
    }

    // Prefer later frames (higher frameCount = more recent)
    score += (int)(frame->frameCount & 0x7F);

    return score;
}

G800FFrame* G800FFrameSelector::pickBest_locked()
{
    if (m_candidates.isEmpty()) return NULL;

    int bestIdx = 0;
    int bestScore = scoreFrame(m_candidates[0]);
    for (size_t i = 1; i < m_candidates.size(); i++) {
        int s = scoreFrame(m_candidates[i]);
        if (s > bestScore) {
            bestScore = s;
            bestIdx = i;
        }
    }

    G800FFrame* best = m_candidates[bestIdx];
    m_candidates.removeAt(bestIdx);

    //ALOGI("%s: selected frame fcount=%u af=%d ae=%d flash=%d score=%d",
    //      __FUNCTION__, best->frameCount,
    //      best->afState, best->aeState,
    //      best->aeflashMode, bestScore);

    return best;
}

bool G800FFrameSelector::offerFrame(G800FFrame* frame)
{
    if (!frame) return false;
    Mutex::Autolock l(m_lock);

    // Flash frame skip: discard frames until skip count is reached
    if (m_frameSkipCount > 0 && m_framesSkipped < m_frameSkipCount) {
        m_framesSkipped++;
        //ALOGD("%s: skipping frame fcount=%u (%d/%d)",
        //      __FUNCTION__, frame->frameCount,
        //      m_framesSkipped, m_frameSkipCount);
        return false;  // Frame not accepted → FLITE requeue
    }

    // Bugfix for OS freeze: collect at most 3 candidates — reject further
    // frames so FLITE buffers remain free for preview.
    // Without this limit, all 8 FLITE buffers would be consumed →
    // FLITE buffer starvation → DQBUF EINVAL → busy loop → OS freeze.
    if ((int)m_candidates.size() >= 3) {
        //ALOGD("%s: candidate limit reached (3), rejecting fcount=%u",
        //      __FUNCTION__, frame->frameCount);
        return false;
    }

    // Store frame as candidate
    m_candidates.push(frame);
    m_cond.signal();  // wake selectFrames()

    //ALOGD("%s: offered frame fcount=%u af=%d ae=%d flash=%d (candidates=%zu)",
    //      __FUNCTION__, frame->frameCount, frame->afState, frame->aeState,
    //      frame->aeflashMode, m_candidates.size());

    return true;
}

int G800FFrameSelector::selectFrames(int count, int pipeId, bool maxFrame,
                                     int waitTime,
                                     Vector<G800FFrame*>& outFrames)
{
    Mutex::Autolock l(m_lock);

    // waitTime: 0x1e = 30 units ≈ 200ms
    // Each unit ≈ 6.67ms. Convert to ms for Condition::waitRelative.
    int timeoutMs = waitTime * 7;  // Round up to whole ms
    int64_t deadline = systemTime(SYSTEM_TIME_MONOTONIC) + ms2ns(timeoutMs);

    // Wait until enough candidates or timeout
    while ((int)m_candidates.size() < count) {
        int64_t now = systemTime(SYSTEM_TIME_MONOTONIC);
        if (now >= deadline) break;
        int64_t remaining = deadline - now;
        // Condition::waitRelative expects nanoseconds
        m_cond.waitRelative(m_lock, remaining);
    }

    if (m_candidates.isEmpty()) {
        ALOGW("%s: no candidates after %dms timeout (pipeId=%d)",
              __FUNCTION__, timeoutMs, pipeId);
        return 0;
    }

    int selected = 0;
    if (maxFrame) {
        // Only the newest frame
        G800FFrame* best = pickBest_locked();
        if (best) {
            outFrames.push(best);
            selected = 1;
        }
    } else {
        // Select the 'count' best frames
        while (selected < count && !m_candidates.isEmpty()) {
            G800FFrame* best = pickBest_locked();
            if (best) {
                outFrames.push(best);
                selected++;
            }
        }
    }

    //ALOGI("%s: selected %d/%d frames from pipeId=%d (candidates=%zu remaining)",
    //      __FUNCTION__, selected, count, pipeId, m_candidates.size());

    return selected;
}

G800FFrame* G800FFrameSelector::selectOneFrame(int pipeId, int waitTime)
{
    Vector<G800FFrame*> frames;
    int n = selectFrames(1, pipeId, false, waitTime, frames);
    if (n > 0 && frames.size() > 0) {
        return frames[0];
    }
    return NULL;
}

bool G800FFrameSelector::hasSelectedFrame() const
{
    Mutex::Autolock l(m_lock);
    return !m_selected.isEmpty();
}

void G800FFrameSelector::releaseCandidates_locked(G800FFrameQueue* recycleQ)
{
    if (!recycleQ) {
        m_candidates.clear();
        return;
    }
    for (size_t i = 0; i < m_candidates.size(); i++) {
        recycleQ->push_back(m_candidates[i]);
    }
    m_candidates.clear();
}

void G800FFrameSelector::cancel(G800FFrameQueue* recycleQ)
{
    Mutex::Autolock l(m_lock);
    releaseCandidates_locked(recycleQ);
    // Also return selected frames
    if (recycleQ) {
        for (size_t i = 0; i < m_selected.size(); i++) {
            recycleQ->push_back(m_selected[i]);
        }
    }
    m_selected.clear();
    m_framesSkipped = 0;
}

} // namespace android
