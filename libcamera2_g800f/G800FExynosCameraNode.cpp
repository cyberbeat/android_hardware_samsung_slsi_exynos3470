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

#define LOG_TAG "G800FExynosCameraNode"
#include <log/log.h>

#include "G800FExynosCameraNode.h"
#include <exynos_v4l2.h>
#include <videodev2_exynos_media.h>
#include <fimc-is-metadata.h>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <poll.h>
#include <unistd.h>
#include <sys/mman.h>

namespace android {

G800FExynosCameraNode::G800FExynosCameraNode()
    : m_fd(-1),
      m_nodeNum(-1),
      m_isCreated(false),
      m_isStarted(false),
      m_hasBuffers(false),
      m_width(0),
      m_height(0),
      m_pixFmt(0),
      m_numPlanes(0),
      m_bufType(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE),
      m_memory(V4L2_MEMORY_MMAP),
      m_hasSize(false),
      m_hasColor(false),
      m_hasBufferType(false)
{
    m_devPath[0] = '\0';
    memset(m_bytesPerLine, 0, sizeof(m_bytesPerLine));
    memset(m_planeSizeImage, 0, sizeof(m_planeSizeImage));
}

G800FExynosCameraNode::~G800FExynosCameraNode()
{
    if (m_fd >= 0)
        close();
}

status_t G800FExynosCameraNode::create(int videoNodeNum)
{
    // Idempotent: if already created with the same node, return OK.
    // This allows reprocessing nodes to be reused across captures.
    if (m_isCreated) {
        if (m_nodeNum == videoNodeNum)
            return NO_ERROR;
        ALOGE("%s: node already created with different num (%d vs %d)",
              __FUNCTION__, m_nodeNum, videoNodeNum);
        return INVALID_OPERATION;
    }
    m_nodeNum = videoNodeNum;
    snprintf(m_devPath, sizeof(m_devPath), "/dev/video%d", videoNodeNum);
    m_isCreated = true;
    return NO_ERROR;
}

status_t G800FExynosCameraNode::open()
{
    if (m_fd >= 0) {
        ALOGE("%s: node %d already open", __FUNCTION__, m_nodeNum);
        return INVALID_OPERATION;
    }
    if (!m_isCreated) {
        ALOGE("%s: create() not called", __FUNCTION__);
        return INVALID_OPERATION;
    }

    m_fd = exynos_v4l2_open(m_devPath, O_RDWR, 0666);
    if (m_fd < 0) {
        ALOGE("%s: exynos_v4l2_open(%s) failed: %d", __FUNCTION__, m_devPath, m_fd);
        return UNKNOWN_ERROR;
    }
    ALOGD("%s: opened %s fd=%d", __FUNCTION__, m_devPath, m_fd);
    return NO_ERROR;
}

status_t G800FExynosCameraNode::close()
{
    if (m_fd < 0)
        return NO_ERROR;
    if (m_isStarted) {
        stop();
    }
    if (exynos_v4l2_close(m_fd) < 0)
        ALOGE("%s: exynos_v4l2_close(%d) failed", __FUNCTION__, m_fd);
    m_fd = -1;
    return NO_ERROR;
}

int G800FExynosCameraNode::getFd() const
{
    return m_fd;
}

int G800FExynosCameraNode::getNodeNum() const
{
    return m_nodeNum;
}

v4l2_buf_type G800FExynosCameraNode::getBufferType() const
{
    return m_bufType;
}

status_t G800FExynosCameraNode::setSize(int w, int h)
{
    m_width = w;
    m_height = h;
    m_hasSize = true;
    return NO_ERROR;
}

status_t G800FExynosCameraNode::setColorFormat(int pixFmt, int numPlanes)
{
    m_pixFmt = pixFmt;
    m_numPlanes = numPlanes;
    m_hasColor = true;
    return NO_ERROR;
}

status_t G800FExynosCameraNode::setBufferType(int numPlanes, v4l2_buf_type type, v4l2_memory memory)
{
    m_bufType = type;
    m_memory = memory;
    m_numPlanes = numPlanes;
    m_hasBufferType = true;
    return NO_ERROR;
}

status_t G800FExynosCameraNode::setFormat()
{
    if (m_fd < 0)
        return INVALID_OPERATION;
    if (!m_hasSize || !m_hasColor || !m_hasBufferType) {
        ALOGE("%s: size/color/buffer type not set for %s", __FUNCTION__, m_devPath);
        return INVALID_OPERATION;
    }

    // bytesperline=0 and sizeimage=0 are passed for ALL planes to S_FMT.
    // The kernel computes the actual buffer sizes itself:
    //   fimc_is_queue_set_format(): if bytesperline==0, set 0
    //   fimc_is_set_plane_size(): for bytesperline==0 → get_plane_size_flite()
    //   SPARE_SIZE = 32*1024 for the metadata plane
    //
    // Our previous implementation computed bytesperline itself
    // (e.g. width*3/2 for SBGGR12), which produced DIFFERENT values
    // than the kernel:
    //   Our bpl:  3280 * 3/2 = 4920  → size = 4920 * 2458 = 12,093,360
    //   Kernel bpl: (3280+9)/10*10*8/5 = 5248 → size = 5248 * 2458 = 12,899,584
    // This led to a too-small plane-0 size and possibly POLLERR.
    v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = m_bufType;

    if (m_bufType == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE ||
        m_bufType == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
        v4l2_pix_format_mplane& mp = fmt.fmt.pix_mp;
        mp.width = m_width;
        mp.height = m_height;
        mp.pixelformat = m_pixFmt;
        mp.field = V4L2_FIELD_NONE;
        mp.colorspace = V4L2_COLORSPACE_JPEG;
        mp.num_planes = m_numPlanes;
        // bytesperline=0 and sizeimage=0 for all planes —
        // the kernel computes the sizes itself.
        // plane_fmt[i] is already 0 from the memset.
    } else {
        v4l2_pix_format& pix = fmt.fmt.pix;
        pix.width = m_width;
        pix.height = m_height;
        pix.pixelformat = m_pixFmt;
        pix.field = V4L2_FIELD_NONE;
        pix.colorspace = V4L2_COLORSPACE_JPEG;
        // bytesperline=0 and sizeimage=0 — kernel computes them itself.
    }

    ALOGI("%s: s_fmt %s type=%d %dx%d fmt=0x%08x planes=%d bpl=0 size=0",
          __FUNCTION__, m_devPath, m_bufType, m_width, m_height,
          m_pixFmt, m_numPlanes);

    if (exynos_v4l2_s_fmt(m_fd, &fmt) < 0) {
        ALOGE("%s: exynos_v4l2_s_fmt failed on %s", __FUNCTION__, m_devPath);
        return UNKNOWN_ERROR;
    }

    // Read back negotiated format with G_FMT to capture the kernel's
    // bytesperline per plane.  The stride is obtained from the Gralloc
    // buffer manager (getBufStride → this+0xcf4) in the vendor path; here it
    // is obtained directly from the V4L2 node.  The copy path in handleScpBuffer uses
    // this to copy row-by-row when the kernel aligns wider than pw.
    v4l2_format gfmt;
    memset(&gfmt, 0, sizeof(gfmt));
    gfmt.type = m_bufType;
    if (exynos_v4l2_g_fmt(m_fd, &gfmt) == 0) {
        v4l2_pix_format_mplane &gmp = gfmt.fmt.pix_mp;
        ALOGI("%s: g_fmt %s type=%d %dx%d fmt=0x%08x planes=%d",
              __FUNCTION__, m_devPath, gfmt.type, gmp.width, gmp.height,
              gmp.pixelformat, gmp.num_planes);
        // NOTE: This kernel (Exynos 3470 FIMC-IS) regularly returns g_fmt
        // 0x0 for ALL nodes — even the working preview nodes.
        // This is NOT an error indicator. No error handling for g_fmt=0x0.
        for (int i = 0; i < (int)gmp.num_planes && i < VIDEO_MAX_PLANES; i++) {
            m_bytesPerLine[i] = gmp.plane_fmt[i].bytesperline;
            m_planeSizeImage[i] = gmp.plane_fmt[i].sizeimage;
            ALOGI("%s:   plane[%d] bpl=%u size=%u",
                  __FUNCTION__, i, gmp.plane_fmt[i].bytesperline,
                  gmp.plane_fmt[i].sizeimage);
        }
    } else {
        ALOGW("%s: g_fmt failed on %s (errno=%d)", __FUNCTION__, m_devPath, errno);
    }
    return NO_ERROR;
}

status_t G800FExynosCameraNode::setFormat(const v4l2_format *fmt)
{
    if (m_fd < 0) return INVALID_OPERATION;
    if (exynos_v4l2_s_fmt(m_fd, const_cast<v4l2_format*>(fmt)) < 0) {
        ALOGE("%s: exynos_v4l2_s_fmt failed on %s", __FUNCTION__, m_devPath);
        return UNKNOWN_ERROR;
    }
    return NO_ERROR;
}

status_t G800FExynosCameraNode::setCrop(const v4l2_crop *crop)
{
    if (m_fd < 0) return INVALID_OPERATION;
    if (exynos_v4l2_s_crop(m_fd, const_cast<v4l2_crop*>(crop)) < 0) {
        ALOGE("%s: exynos_v4l2_s_crop failed on %s", __FUNCTION__, m_devPath);
        return UNKNOWN_ERROR;
    }
    return NO_ERROR;
}

status_t G800FExynosCameraNode::setInput(int index)
{
    if (m_fd < 0) return INVALID_OPERATION;
    ALOGI("%s: s_input %s index=0x%x(%d)", __FUNCTION__, m_devPath, index, index);
    if (exynos_v4l2_s_input(m_fd, index) < 0) {
        ALOGE("%s: exynos_v4l2_s_input(0x%x) failed on %s errno=%d(%s)",
              __FUNCTION__, index, m_devPath, errno, strerror(errno));
        return UNKNOWN_ERROR;
    }
    return NO_ERROR;
}

status_t G800FExynosCameraNode::setParam(const v4l2_streamparm *parm)
{
    if (m_fd < 0) return INVALID_OPERATION;
    if (exynos_v4l2_s_parm(m_fd, const_cast<v4l2_streamparm*>(parm)) < 0) {
        ALOGE("%s: exynos_v4l2_s_parm failed on %s", __FUNCTION__, m_devPath);
        return UNKNOWN_ERROR;
    }
    return NO_ERROR;
}

status_t G800FExynosCameraNode::setControl(unsigned int id, int value)
{
    if (m_fd < 0) return INVALID_OPERATION;
    ALOGI("%s: s_ctrl %s id=0x%08x value=%d(0x%x)", __FUNCTION__, m_devPath, id, value, value);
    if (exynos_v4l2_s_ctrl(m_fd, id, value) < 0) {
        ALOGE("%s: exynos_v4l2_s_ctrl(0x%08x, %d) failed on %s errno=%d(%s)",
              __FUNCTION__, id, value, m_devPath, errno, strerror(errno));
        return UNKNOWN_ERROR;
    }
    return NO_ERROR;
}

status_t G800FExynosCameraNode::getControl(unsigned int id, int *value)
{
    if (m_fd < 0 || !value) return INVALID_OPERATION;
    if (exynos_v4l2_g_ctrl(m_fd, id, value) < 0) {
        ALOGE("%s: exynos_v4l2_g_ctrl(%u) failed on %s", __FUNCTION__, id, m_devPath);
        return UNKNOWN_ERROR;
    }
    return NO_ERROR;
}

status_t G800FExynosCameraNode::reqBuffers(int num)
{
    if (m_fd < 0) return INVALID_OPERATION;
    if (!m_hasBufferType) return INVALID_OPERATION;

    v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = (uint32_t)num;
    req.type = m_bufType;
    req.memory = m_memory;

    ALOGI("%s: reqbufs %s count=%d type=%d memory=%d",
          __FUNCTION__, m_devPath, num, m_bufType, m_memory);

    if (exynos_v4l2_reqbufs(m_fd, &req) < 0) {
        ALOGE("%s: exynos_v4l2_reqbufs(%d) failed on %s errno=%d(%s)",
              __FUNCTION__, num, m_devPath, errno, strerror(errno));
        return UNKNOWN_ERROR;
    }
    if (num > 0)
        m_hasBuffers = true;
    else
        m_hasBuffers = false;
    return NO_ERROR;
}

status_t G800FExynosCameraNode::reqBuffers(v4l2_requestbuffers *req)
{
    if (m_fd < 0) return INVALID_OPERATION;
    if (exynos_v4l2_reqbufs(m_fd, req) < 0) {
        ALOGE("%s: exynos_v4l2_reqbufs failed on %s", __FUNCTION__, m_devPath);
        return UNKNOWN_ERROR;
    }
    return NO_ERROR;
}

status_t G800FExynosCameraNode::releaseBuffers()
{
    if (m_fd < 0) return NO_ERROR;
    if (!m_hasBuffers) return NO_ERROR;  // nothing to release
    // REQBUFS(0) frees all vb2 buffers.  Must be called after streamoff.
    return reqBuffers(0);
}

status_t G800FExynosCameraNode::queryBuf(v4l2_buffer *buf)
{
    if (m_fd < 0) return INVALID_OPERATION;
    if (exynos_v4l2_querybuf(m_fd, buf) < 0) {
        ALOGE("%s: exynos_v4l2_querybuf failed on %s errno=%d(%s)",
              __FUNCTION__, m_devPath, errno, strerror(errno));
        return UNKNOWN_ERROR;
    }
    return NO_ERROR;
}

void* G800FExynosCameraNode::mmapBuf(unsigned long offset, size_t length)
{
    if (m_fd < 0) return NULL;
    void *p = mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, offset);
    if (p == MAP_FAILED) {
        ALOGE("%s: mmap(offset=0x%lx, len=%zu) failed on %s errno=%d(%s)",
              __FUNCTION__, offset, length, m_devPath, errno, strerror(errno));
        return NULL;
    }
    return p;
}

status_t G800FExynosCameraNode::qBuf(v4l2_buffer *buf)
{
    if (m_fd < 0) return INVALID_OPERATION;
    /* per-buffer log disabled: fires on every queued buffer.
    if (buf && buf->m.planes) {
        ALOGV("%s: qbuf %s idx=%d type=%d mem=%d planes=%d "
              "p0[len=%u] p1[len=%u]",
              __FUNCTION__, m_devPath, buf->index, buf->type, buf->memory, buf->length,
              buf->m.planes[0].length,
              (buf->length > 1) ? buf->m.planes[1].length : 0);
    }
    */
    if (exynos_v4l2_qbuf(m_fd, buf) < 0) {
        ALOGE("%s: exynos_v4l2_qbuf(idx=%d) failed on %s errno=%d(%s)",
              __FUNCTION__, buf ? buf->index : -1, m_devPath, errno, strerror(errno));
        return UNKNOWN_ERROR;
    }
    return NO_ERROR;
}

status_t G800FExynosCameraNode::dqBuf(v4l2_buffer *buf)
{
    /* exynos_v4l2_dqbuf() is blocking (fd is O_RDWR).
     *
     * But: the Exynos-3470 kernel (fimc_is_video_dqbuf @ line 1184) returns
     * -EINVAL (-22) IMMEDIATELY when no buffers are queued
     * (q->bufs_cnt == 0).  It only blocks when buffers are queued but
     * not yet ready.
     *
     * This case normally does not arise because:
     *   - 8+ buffers in the pipeline
     *   - m_putBuffer() keeps the flow going
     *
     * Our HAL3 can however get into a situation where all FLITE buffers
     * are in the ISP pipeline and none are queued.  Without handling,
     * dqbuf would return -EINVAL in a busy loop → watchdog → reboot.
     *
     * Solution: treat -EINVAL as retryable (sleep 5ms + -EAGAIN),
     * as is done for -EAGAIN too.  For other errors
     * (STREAMOFF, fd closed) pass the error through.
     */
    if (m_fd < 0) return INVALID_OPERATION;
    int ret = exynos_v4l2_dqbuf(m_fd, buf);
    if (ret < 0) {
        if (errno == EINVAL) {
            /* Queue empty — no buffers queued.  The kernel does not block
             * but returns -EINVAL.
             *
             * IMPORTANT (bugfix for OS freeze): If the stream was already
             * stopped (m_isStarted==false after STREAMOFF), EINVAL is NOT
             * buffer starvation but the expected behavior on a stopped
             * stream.  In this case do NOT retry — return -EINVAL directly
             * so the thread exits immediately instead of staying in a
             * busy loop.
             *
             * Only if the stream is still running (m_isStarted==true) is
             * EINVAL a temporary buffer starvation → sleep + -EAGAIN. */
            if (!m_isStarted) {
                return -EINVAL;
            }
            // Buffer starvation: kernel returns EINVAL when qcount=0
            // (all buffers in-flight, none ready).  10ms sleep reduces
            // the ioctl storm from ~200Hz to ~100Hz — enough for the FIMC-IS
            // kernel to recover instead of freezing.
            usleep(10000);
            return -EAGAIN;
        }
        if (errno == EAGAIN) {
            /* Buffer queued but not yet ready — should not happen with a
             * blocking fd, but pass it through for safety. */
            return -EAGAIN;
        }
        ALOGE("%s: exynos_v4l2_dqbuf failed on %s errno=%d(%s)",
              __FUNCTION__, m_devPath, errno, strerror(errno));
        return UNKNOWN_ERROR;
    }
    /* per-buffer log disabled: fires on every dequeued buffer.
    if (buf && buf->m.planes) {
        ALOGV("%s: dqbuf ok %s idx=%d p0[bu=%u,len=%u] p1[bu=%u,len=%u]",
              __FUNCTION__, m_devPath, buf->index,
              buf->m.planes[0].bytesused, buf->m.planes[0].length,
              (buf->length > 1) ? buf->m.planes[1].bytesused : 0,
              (buf->length > 1) ? buf->m.planes[1].length : 0);
    }
    */
    return NO_ERROR;
}

status_t G800FExynosCameraNode::dqBufTimeout(v4l2_buffer *buf, int timeoutMs)
{
    if (m_fd < 0) return INVALID_OPERATION;

    // Use poll() to wait for a buffer with timeout, avoiding indefinite blocking.
    // OUTPUT queues (e.g. ISP V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) signal
    // POLLOUT | POLLWRNORM when a done buffer is available for dequeue.
    // CAPTURE queues (e.g. FLITE, SCP) signal POLLIN | POLLRDNORM.
    // Request both to handle either queue type correctly.
    struct pollfd pfd;
    pfd.fd = m_fd;
    pfd.events = POLLIN | POLLOUT;
    pfd.revents = 0;

    int ret = poll(&pfd, 1, timeoutMs);
    if (ret < 0) {
        if (errno == EINTR) return TIMED_OUT;
        ALOGE("%s: poll failed on %s: errno=%d(%s)", __FUNCTION__, m_devPath, errno, strerror(errno));
        return UNKNOWN_ERROR;
    }
    if (ret == 0) {
        ALOGW("%s: poll timeout (%dms) on %s", __FUNCTION__, timeoutMs, m_devPath);
        return TIMED_OUT;
    }
    if (pfd.revents & POLLERR) {
        // POLLERR from vb2_poll() can mean either:
        // 1. list_empty(&q->queued_list) — no buffers queued to driver
        //    (buffer starvation, e.g. all FLITE buffers in ISP pipeline)
        // 2. A real driver error
        // Case 1 is normal when downstream pipe (ISP) hasn't returned buffers yet.
        // Return -EAGAIN so the caller can sleep and retry.
        //ALOGD("%s: poll POLLERR revents=0x%x on %s (likely buffer starvation, retryable)",
        //      __FUNCTION__, pfd.revents, m_devPath);
        return -EAGAIN;
    }
    if (!(pfd.revents & (POLLIN | POLLOUT))) {
        ALOGE("%s: poll unexpected revents=0x%x on %s", __FUNCTION__, pfd.revents, m_devPath);
        return UNKNOWN_ERROR;
    }

    if (exynos_v4l2_dqbuf(m_fd, buf) < 0) {
        if (errno == EAGAIN) return TIMED_OUT;
        ALOGE("%s: exynos_v4l2_dqbuf failed on %s errno=%d(%s)",
              __FUNCTION__, m_devPath, errno, strerror(errno));
        return UNKNOWN_ERROR;
    }
    /* per-buffer log disabled: fires on every dequeued buffer.
    if (buf && buf->m.planes) {
        ALOGV("%s: dqbuf ok %s idx=%d p0[bu=%u,len=%u] p1[bu=%u,len=%u]",
              __FUNCTION__, m_devPath, buf->index,
              buf->m.planes[0].bytesused, buf->m.planes[0].length,
              (buf->length > 1) ? buf->m.planes[1].bytesused : 0,
              (buf->length > 1) ? buf->m.planes[1].length : 0);
    }
    */
    return NO_ERROR;
}

status_t G800FExynosCameraNode::start(v4l2_buf_type type)
{
    if (m_fd < 0) return INVALID_OPERATION;
    if (m_isStarted) return NO_ERROR;
    if (type == 0) type = m_bufType;
    ALOGI("%s: streamon %s type=%d", __FUNCTION__, m_devPath, type);
    if (exynos_v4l2_streamon(m_fd, type) < 0) {
        ALOGE("%s: exynos_v4l2_streamon failed on %s errno=%d(%s)",
              __FUNCTION__, m_devPath, errno, strerror(errno));
        return UNKNOWN_ERROR;
    }
    m_isStarted = true;
    return NO_ERROR;
}

status_t G800FExynosCameraNode::stop(v4l2_buf_type type)
{
    if (m_fd < 0) return NO_ERROR;
    if (!m_isStarted) return NO_ERROR;
    if (type == 0) type = m_bufType;
    ALOGI("%s: streamoff %s type=%d", __FUNCTION__, m_devPath, type);
    if (exynos_v4l2_streamoff(m_fd, type) < 0) {
        ALOGE("%s: exynos_v4l2_streamoff failed on %s errno=%d(%s)",
              __FUNCTION__, m_devPath, errno, strerror(errno));
        return UNKNOWN_ERROR;
    }
    m_isStarted = false;
    return NO_ERROR;
}

} // namespace android
