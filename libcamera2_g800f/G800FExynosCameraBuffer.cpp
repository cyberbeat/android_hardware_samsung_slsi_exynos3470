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

#define LOG_TAG "G800FExynosCameraBuffer"
#include <log/log.h>

#include "G800FExynosCameraBuffer.h"
#include <ion.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>

namespace android {

G800FExynosCameraBuffer::G800FExynosCameraBuffer()
    : m_ionClient(-1),
      m_ionBuffer(-1),
      m_v4l2Fd(-1),
      m_isMmap(false),
      m_isDmabuf(false),
      m_dmaBufFds(NULL),
      m_vaddr(NULL),
      m_totalSize(0),
      m_numPlanes(0),
      m_planeSize(NULL),
      m_planeStride(NULL),
      m_planeVaddr(NULL)
{
}

G800FExynosCameraBuffer::~G800FExynosCameraBuffer()
{
    release();
}

status_t G800FExynosCameraBuffer::alloc(int numPlanes, const size_t* planeSizes,
                                        unsigned int heapMask, unsigned int flags)
{
    if (numPlanes <= 0 || planeSizes == NULL)
        return BAD_VALUE;
    if (m_ionClient >= 0)
        release();

    m_ionClient = ion_client_create();
    if (m_ionClient < 0) {
        ALOGE("%s: ion_client_create failed", __FUNCTION__);
        return UNKNOWN_ERROR;
    }

    m_numPlanes = numPlanes;
    m_planeSize = new size_t[numPlanes];
    m_planeStride = new size_t[numPlanes];
    m_planeVaddr = new void*[numPlanes];

    size_t total = 0;
    for (int i = 0; i < numPlanes; i++) {
        m_planeSize[i] = planeSizes[i];
        m_planeStride[i] = planeSizes[i]; // simple contiguous packing
        total += planeSizes[i];
        m_planeVaddr[i] = NULL;
    }

    // Round up to a whole page size so ion_map succeeds.
    size_t pageSize = sysconf(_SC_PAGESIZE);
    if (pageSize == 0)
        pageSize = 4096;
    m_totalSize = (total + pageSize - 1) & ~(pageSize - 1);

    m_ionBuffer = ion_alloc(m_ionClient, m_totalSize, pageSize, heapMask, flags);
    if (m_ionBuffer < 0) {
        ALOGE("%s: ion_alloc(%zu) failed: %d", __FUNCTION__, m_totalSize, m_ionBuffer);
        release();
        return UNKNOWN_ERROR;
    }

    m_vaddr = ion_map(m_ionBuffer, m_totalSize, 0);
    if (m_vaddr == reinterpret_cast<void*>(-1) || m_vaddr == NULL) {
        ALOGE("%s: ion_map failed", __FUNCTION__);
        release();
        return UNKNOWN_ERROR;
    }

    // Set per-plane userptrs (contiguous block).
    uint8_t* base = reinterpret_cast<uint8_t*>(m_vaddr);
    size_t offset = 0;
    for (int i = 0; i < numPlanes; i++) {
        m_planeVaddr[i] = base + offset;
        offset += m_planeSize[i];
    }

    return NO_ERROR;
}

status_t G800FExynosCameraBuffer::allocMmap(int fd, int index, int numPlanes, int bufType)
{
    /* MMAP mode: kernel allocates the buffer via vb2-ION.  We call
     * VIDIOC_QUERYBUF to get the per-plane mem_offset and length, then
     * mmap() each plane to get a user pointer.  The kernel retains the
     * kernel virtual address (kvaddr) and device virtual address (dvaddr),
     * which the FIMC-IS ISP/SCC/SCP nodes require for shot_ext/stream
     * metadata.  This is the MMAP buffer mode (V4L2_MEMORY_MMAP). */
    if (fd < 0 || index < 0 || numPlanes <= 0)
        return BAD_VALUE;
    if (m_ionClient >= 0 || m_isMmap)
        release();

    m_isMmap = true;
    m_v4l2Fd = fd;
    m_numPlanes = numPlanes;
    m_planeSize = new size_t[numPlanes];
    m_planeStride = new size_t[numPlanes];
    m_planeVaddr = new void*[numPlanes];

    v4l2_buffer vbuf;
    v4l2_plane planes[VIDEO_MAX_PLANES];
    memset(&vbuf, 0, sizeof(vbuf));
    memset(planes, 0, sizeof(planes));
    vbuf.type = (enum v4l2_buf_type)bufType;
    vbuf.memory = V4L2_MEMORY_MMAP;
    vbuf.index = index;
    vbuf.length = numPlanes;
    vbuf.m.planes = planes;

    if (ioctl(fd, VIDIOC_QUERYBUF, &vbuf) < 0) {
        ALOGE("%s: VIDIOC_QUERYBUF idx=%d failed: %d(%s)",
              __FUNCTION__, index, errno, strerror(errno));
        release();
        return UNKNOWN_ERROR;
    }

    size_t total = 0;
    for (int i = 0; i < numPlanes; i++) {
        size_t len = planes[i].length;
        unsigned long off = planes[i].m.mem_offset;
        if (len == 0) {
            ALOGE("%s: plane %d length=0", __FUNCTION__, i);
            release();
            return UNKNOWN_ERROR;
        }
        void *p = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, off);
        if (p == MAP_FAILED) {
            ALOGE("%s: mmap plane %d off=0x%lx len=%zu failed: %d(%s)",
                  __FUNCTION__, i, off, len, errno, strerror(errno));
            release();
            return UNKNOWN_ERROR;
        }
        m_planeVaddr[i] = p;
        m_planeSize[i] = len;
        m_planeStride[i] = len;
        total += len;
        // DIAG: log per-plane mmap details
        ALOGI("%s: idx=%d plane[%d] off=0x%lx len=%zu vaddr=%p",
              __FUNCTION__, index, i, off, len, p);
    }

    m_totalSize = total;
    m_vaddr = m_planeVaddr[0]; // first plane base

    ALOGI("%s: mmap idx=%d planes=%d total=%zu p0=%p len=%zu p1=%p len=%zu",
          __FUNCTION__, index, numPlanes, total,
          m_planeVaddr[0], m_planeSize[0],
          (numPlanes > 1) ? m_planeVaddr[1] : NULL,
          (numPlanes > 1) ? m_planeSize[1] : 0);
    return NO_ERROR;
}

status_t G800FExynosCameraBuffer::allocDmabuf(int numPlanes,
                                               const size_t* planeSizes,
                                               unsigned int heapMask,
                                               unsigned int flags)
{
    /* DMABUF mode: userspace allocates non-cacheable ION buffers and
     * exports them as dma_buf fds.  The fds are passed to V4L2 with
     * V4L2_MEMORY_DMABUF.  The kernel's vb2 DMABUF path performs
     * vb2_ion_sync_for_device/sync_for_cpu on each qbuf/dqbuf cycle,
     * providing the cache coherency that the FIMC-IS SCP MMAP path lacks.
     *
     * Each plane gets its own ION buffer + dma_buf fd, because V4L2
     * DMABUF requires one fd per plane.
     */
    if (numPlanes <= 0 || planeSizes == NULL)
        return BAD_VALUE;
    if (m_ionClient >= 0 || m_isMmap || m_isDmabuf)
        release();

    m_ionClient = ion_client_create();
    if (m_ionClient < 0) {
        ALOGE("%s: ion_client_create failed", __FUNCTION__);
        return UNKNOWN_ERROR;
    }

    m_isDmabuf = true;
    m_numPlanes = numPlanes;
    m_planeSize = new size_t[numPlanes];
    m_planeStride = new size_t[numPlanes];
    m_planeVaddr = new void*[numPlanes];
    m_dmaBufFds = new int[numPlanes];

    size_t pageSize = sysconf(_SC_PAGESIZE);
    if (pageSize == 0) pageSize = 4096;

    size_t total = 0;
    for (int i = 0; i < numPlanes; i++) {
        m_planeSize[i] = planeSizes[i];
        m_planeStride[i] = planeSizes[i];
        m_planeVaddr[i] = NULL;
        m_dmaBufFds[i] = -1;

        size_t allocSize = (planeSizes[i] + pageSize - 1) & ~(pageSize - 1);

        /* flags=0 → non-cacheable (writecombine).
         * ion_alloc() in libion_exynos does ION_IOC_SHARE internally
         * and returns a dma_buf fd. */
        ion_buffer buf = ion_alloc(m_ionClient, allocSize, pageSize,
                                    heapMask, flags);
        if (buf < 0) {
            ALOGE("%s: ion_alloc plane %d (%zu bytes) failed: %d",
                  __FUNCTION__, i, allocSize, buf);
            for (int j = 0; j < i; j++) {
                if (m_planeVaddr[j]) ion_unmap(m_planeVaddr[j], m_planeSize[j]);
                if (m_dmaBufFds[j] >= 0) ion_free(m_dmaBufFds[j]);
            }
            release();
            return UNKNOWN_ERROR;
        }
        m_dmaBufFds[i] = buf;

        void* p = ion_map(buf, allocSize, 0);
        if (p == reinterpret_cast<void*>(-1) || p == NULL) {
            ALOGE("%s: ion_map plane %d failed", __FUNCTION__, i);
            ion_free(buf);
            m_dmaBufFds[i] = -1;
            for (int j = 0; j < i; j++) {
                if (m_planeVaddr[j]) ion_unmap(m_planeVaddr[j], m_planeSize[j]);
                if (m_dmaBufFds[j] >= 0) ion_free(m_dmaBufFds[j]);
            }
            release();
            return UNKNOWN_ERROR;
        }
        m_planeVaddr[i] = p;
        total += planeSizes[i];

        ALOGI("%s: dmabuf plane[%d] size=%zu fd=%d vaddr=%p",
              __FUNCTION__, i, planeSizes[i], m_dmaBufFds[i], p);
    }

    m_totalSize = total;
    m_vaddr = m_planeVaddr[0];
    m_ionBuffer = m_dmaBufFds[0];

    ALOGI("%s: dmabuf planes=%d total=%zu p0_fd=%d p0=%p len=%zu",
          __FUNCTION__, numPlanes, total,
          m_dmaBufFds[0], m_planeVaddr[0], m_planeSize[0]);
    return NO_ERROR;
}

int G800FExynosCameraBuffer::dmaBufFd(int plane) const
{
    if (!m_isDmabuf || !m_dmaBufFds || plane < 0 || plane >= m_numPlanes)
        return -1;
    return m_dmaBufFds[plane];
}

void G800FExynosCameraBuffer::release()
{
    if (m_isMmap) {
        // MMAP mode: munmap each plane individually
        if (m_planeVaddr && m_planeSize) {
            for (int i = 0; i < m_numPlanes; i++) {
                if (m_planeVaddr[i] && m_planeSize[i] > 0)
                    munmap(m_planeVaddr[i], m_planeSize[i]);
            }
        }
        m_isMmap = false;
        m_v4l2Fd = -1;
        m_vaddr = NULL;
        m_totalSize = 0;
    } else if (m_isDmabuf) {
        // DMABUF mode: unmap + free each plane's dma_buf
        if (m_planeVaddr && m_planeSize && m_dmaBufFds) {
            for (int i = 0; i < m_numPlanes; i++) {
                if (m_planeVaddr[i] && m_planeSize[i] > 0)
                    ion_unmap(m_planeVaddr[i], m_planeSize[i]);
                if (m_dmaBufFds[i] >= 0)
                    ion_free(m_dmaBufFds[i]);
            }
        }
        delete[] m_dmaBufFds;
        m_dmaBufFds = NULL;
        m_isDmabuf = false;
        m_vaddr = NULL;
        m_totalSize = 0;
        m_ionBuffer = -1;
        if (m_ionClient >= 0) {
            ion_client_destroy(m_ionClient);
            m_ionClient = -1;
        }
    } else {
        // ION mode (USERPTR)
        if (m_vaddr && m_totalSize > 0) {
            ion_unmap(m_vaddr, m_totalSize);
            m_vaddr = NULL;
        }
        if (m_ionBuffer >= 0) {
            ion_free(m_ionBuffer);
            m_ionBuffer = -1;
        }
        if (m_ionClient >= 0) {
            ion_client_destroy(m_ionClient);
            m_ionClient = -1;
        }
    }
    delete[] m_planeSize;
    delete[] m_planeStride;
    delete[] m_planeVaddr;
    m_planeSize = NULL;
    m_planeStride = NULL;
    m_planeVaddr = NULL;
    m_numPlanes = 0;
    m_totalSize = 0;
}

bool G800FExynosCameraBuffer::isValid() const
{
    if (m_isMmap)
        return m_vaddr != NULL && m_planeVaddr != NULL;
    if (m_isDmabuf)
        return m_dmaBufFds != NULL && m_planeVaddr != NULL;
    return m_ionClient >= 0 && m_ionBuffer >= 0 && m_vaddr != NULL;
}

int G800FExynosCameraBuffer::fd() const
{
    return m_ionBuffer;
}

void* G800FExynosCameraBuffer::vaddr() const
{
    return m_vaddr;
}

size_t G800FExynosCameraBuffer::totalSize() const
{
    return m_totalSize;
}

int G800FExynosCameraBuffer::numPlanes() const
{
    return m_numPlanes;
}

void* G800FExynosCameraBuffer::planeVaddr(int index) const
{
    if (index < 0 || index >= m_numPlanes || m_planeVaddr == NULL)
        return NULL;
    return m_planeVaddr[index];
}

size_t G800FExynosCameraBuffer::planeSize(int index) const
{
    if (index < 0 || index >= m_numPlanes || m_planeSize == NULL)
        return 0;
    return m_planeSize[index];
}

size_t G800FExynosCameraBuffer::planeStride(int index) const
{
    if (index < 0 || index >= m_numPlanes || m_planeStride == NULL)
        return 0;
    return m_planeStride[index];
}

} // namespace android
