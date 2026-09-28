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

#define LOG_TAG "G800FFrameQueue"
#include <cutils/log.h>
#include <errno.h>
#include "G800FFrameQueue.h"

namespace android {

G800FFrameQueue::G800FFrameQueue()
    : m_capacity(8)
{
}

G800FFrameQueue::~G800FFrameQueue()
{
    clear();
}

status_t G800FFrameQueue::push_back(G800FFrame* frame)
{
    if (!frame) return BAD_VALUE;
    Mutex::Autolock l(m_lock);
    if ((int)m_queue.size() >= m_capacity) {
        ALOGW("%s: queue full (%d), dropping oldest", __FUNCTION__, m_capacity);
        // Drop oldest to make room — caller should have drained
        G800FFrame* old = m_queue.itemAt(0);
        m_queue.removeAt(0);
        // Note: we don't release the old frame here — the caller
        // is responsible for buffer recycling.  Dropping from the
        // queue means the consumer never sees it, so the producer
        // must recycle it.  In practice this should not happen.
        (void)old;
    }
    m_queue.push(frame);
    m_cond.signal();
    return NO_ERROR;
}

status_t G800FFrameQueue::push_front(G800FFrame* frame)
{
    if (!frame) return BAD_VALUE;
    Mutex::Autolock l(m_lock);
    m_queue.insertAt(frame, 0);
    m_cond.signal();
    return NO_ERROR;
}

status_t G800FFrameQueue::pop_front(G800FFrame** frame, int timeoutMs)
{
    Mutex::Autolock l(m_lock);
    while (m_queue.isEmpty()) {
        status_t err = m_cond.waitRelative(m_lock, ms2ns(timeoutMs));
        if (err != NO_ERROR) {
            // Timed out or interrupted
            *frame = NULL;
            return err == -EINTR ? -EINTR : -ETIMEDOUT;
        }
    }
    *frame = m_queue.itemAt(0);
    m_queue.removeAt(0);
    return NO_ERROR;
}

status_t G800FFrameQueue::try_pop_front(G800FFrame** frame)
{
    Mutex::Autolock l(m_lock);
    if (m_queue.isEmpty()) {
        *frame = NULL;
        return -EAGAIN;
    }
    *frame = m_queue.itemAt(0);
    m_queue.removeAt(0);
    return NO_ERROR;
}

void G800FFrameQueue::wakeAll()
{
    Mutex::Autolock l(m_lock);
    m_cond.broadcast();
}

void G800FFrameQueue::clear()
{
    Mutex::Autolock l(m_lock);
    m_queue.clear();
}

int G800FFrameQueue::size() const
{
    Mutex::Autolock l(m_lock);
    return m_queue.size();
}

void G800FFrameQueue::setCapacity(int max)
{
    Mutex::Autolock l(m_lock);
    m_capacity = max;
}

} // namespace android
