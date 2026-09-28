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

#ifndef G800F_EXYNOS_CAMERA_BUFFER_H
#define G800F_EXYNOS_CAMERA_BUFFER_H

#include <utils/Errors.h>
#include <linux/videodev2.h>

namespace android {

/*
 * V4L2 buffer holder for the G800F Camera2 HAL.
 *
 * Two allocation modes:
 * 1. ION (USERPTR): allocates one contiguous ION block, maps it, and
 *    provides per-plane userptrs.  Used for legacy USERPTR mode.
 * 2. MMAP: the kernel allocates the buffer (via vb2-ION).  We call
 *    QUERYBUF + mmap to get user pointers.  This is the MMAP
 *    mode — the kernel has both kvaddr and dvaddr, which is required
 *    for ISP/SCC/SCP nodes that need shot_ext/stream metadata in
 *    kernel-mapped memory.
 */

class G800FExynosCameraBuffer {
public:
    G800FExynosCameraBuffer();
    ~G800FExynosCameraBuffer();

    // ION-backed allocation (USERPTR mode)
    status_t alloc(int numPlanes, const size_t* planeSizes,
                   unsigned int heapMask = (1 << 5), // ION_HEAP_EXYNOS_MASK
                   unsigned int flags = 0);

    // MMAP mode: kernel allocates, we mmap.  Call after reqbufs(MMAP).
    // fd = V4L2 device fd, index = buffer index, numPlanes = plane count,
    // bufType = V4L2 buffer type (needed for QUERYBUF).
    status_t allocMmap(int fd, int index, int numPlanes, int bufType);

    // DMABUF mode: userspace allocates ION buffer (non-cacheable, flags=0),
    // exports it as dma_buf fd, and maps it for CPU access.
    // The dma_buf fd is passed to V4L2 qbuf with V4L2_MEMORY_DMABUF.
    // The kernel's vb2_ion_map_dmabuf/unmap_dmabuf will perform
    // vb2_ion_sync_for_device/sync_for_cpu automatically on each
    // qbuf/dqbuf cycle — this is the cache-coherency fix that the
    // FIMC-IS SCP MMAP path lacks.
    status_t allocDmabuf(int numPlanes, const size_t* planeSizes,
                         unsigned int heapMask = (1 << 5),
                         unsigned int flags = 0);

    void     release();

    bool     isValid() const;
    int      fd() const;
    void*    vaddr() const;
    size_t   totalSize() const;
    int      numPlanes() const;
    void*    planeVaddr(int index) const;
    size_t   planeSize(int index) const;
    size_t   planeStride(int index) const;
    int      dmaBufFd(int plane) const;  // per-plane dma_buf fd for V4L2 DMABUF

private:
    int      m_ionClient;   // ION mode
    int      m_ionBuffer;   // ION mode (also dma_buf fd for DMABUF mode)
    int      m_v4l2Fd;      // MMAP mode (for munmap, not close)
    bool     m_isMmap;      // true = MMAP mode, false = ION mode
    bool     m_isDmabuf;    // true = DMABUF mode (ION-allocated, non-cacheable)
    int*     m_dmaBufFds;   // per-plane dma_buf fds (DMABUF mode)
    void*    m_vaddr;
    size_t   m_totalSize;
    int      m_numPlanes;
    size_t*  m_planeSize;
    size_t*  m_planeStride;
    void**   m_planeVaddr;

    G800FExynosCameraBuffer(const G800FExynosCameraBuffer&);
    G800FExynosCameraBuffer& operator=(const G800FExynosCameraBuffer&);
};

} // namespace android

#endif // G800F_EXYNOS_CAMERA_BUFFER_H
