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

#ifndef G800F_EXYNOS_CAMERA_NODE_H
#define G800F_EXYNOS_CAMERA_NODE_H

#include <utils/Errors.h>
#include <linux/videodev2.h>

namespace android {

/*
 * Minimal reimplementation of Samsung's ExynosCameraNode for the G800F.
 *
 * It wraps libexynosv4l2 (exynos_v4l2_*) for the FIMC-IS V4L2 nodes at
 * /dev/video100+, using the same V4L2 ioctls.
 *
 * The API mirrors ExynosCameraNode: store size, color format,
 * buffer type and plane count first, then call setFormat() to push a
 * v4l2_format to the driver, reqBuffers() to allocate, and qBuf()/dqBuf()
 * for each frame.
 */

class G800FExynosCameraNode {
public:
    G800FExynosCameraNode();
    ~G800FExynosCameraNode();

    status_t create(int videoNodeNum);
    status_t open();
    status_t close();
    bool     isOpen() const { return m_fd >= 0; }

    int      getFd() const;
    int      getNodeNum() const;
    v4l2_buf_type getBufferType() const;
    const char* getDevPath() const { return m_devPath; }

    status_t setSize(int w, int h);
    status_t setColorFormat(int v4l2PixFmt, int numPlanes);
    status_t setBufferType(int numPlanes, v4l2_buf_type type, v4l2_memory memory);
    status_t setFormat();                 // build & push v4l2_format from stored values
    status_t setFormat(const v4l2_format *fmt); // caller-supplied v4l2_format
    status_t setCrop(const v4l2_crop *crop);
    status_t setInput(int index);
    status_t setParam(const v4l2_streamparm *parm);
    status_t setControl(unsigned int id, int value);
    status_t getControl(unsigned int id, int *value);

    status_t reqBuffers(int num);         // convenience wrapper
    status_t reqBuffers(v4l2_requestbuffers *req);
    bool     hasBuffers() const { return m_hasBuffers; }
    // Release previously allocated buffers (REQBUFS(0)).  Only safe when
    // the node is NOT streaming (call stop() first).  No-op if no buffers
    // were ever allocated.
    status_t releaseBuffers();
    status_t queryBuf(v4l2_buffer *buf);
    void*    mmapBuf(unsigned long offset, size_t length);  // mmap a kernel-allocated buffer plane
    status_t qBuf(v4l2_buffer *buf);
    status_t dqBuf(v4l2_buffer *buf);
    status_t dqBufTimeout(v4l2_buffer *buf, int timeoutMs);  // non-blocking with poll timeout

    status_t start(v4l2_buf_type type = (v4l2_buf_type)0);
    status_t stop(v4l2_buf_type type = (v4l2_buf_type)0);

    // Kernel-negotiated bytes-per-line per plane (from VIDIOC_G_FMT after S_FMT).
    // Obtained directly from the V4L2 node via G_FMT.
    // Returns 0 if not yet set (caller should fall back to width).
    unsigned int getBytesPerLine(int plane) const {
        return (plane >= 0 && plane < VIDEO_MAX_PLANES) ? m_bytesPerLine[plane] : 0;
    }
    // Kernel-negotiated plane size (sizeimage from G_FMT).
    // Used for DMABUF allocation where userspace must allocate
    // the same size the kernel expects.
    unsigned int getPlaneSizeImage(int plane) const {
        return (plane >= 0 && plane < VIDEO_MAX_PLANES) ? m_planeSizeImage[plane] : 0;
    }

private:
    int      m_fd;
    int      m_nodeNum;
    char     m_devPath[32];
    bool     m_isCreated;
    bool     m_isStarted;
    bool     m_hasBuffers;  // true after first successful reqBuffers(N>0)

    // Stored format/buffer parameters for setFormat()
    int      m_width;
    int      m_height;
    int      m_pixFmt;
    int      m_numPlanes;
    v4l2_buf_type  m_bufType;
    v4l2_memory    m_memory;
    bool     m_hasSize;
    bool     m_hasColor;
    bool     m_hasBufferType;

    // Kernel-negotiated bytes-per-line per plane (filled by setFormat after G_FMT).
    unsigned int m_bytesPerLine[VIDEO_MAX_PLANES];
    // Kernel-negotiated plane size (sizeimage from G_FMT).
    unsigned int m_planeSizeImage[VIDEO_MAX_PLANES];

    G800FExynosCameraNode(const G800FExynosCameraNode&);
    G800FExynosCameraNode& operator=(const G800FExynosCameraNode&);
};

} // namespace android

#endif // G800F_EXYNOS_CAMERA_NODE_H
