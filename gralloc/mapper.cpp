/*
 * Copyright (C) 2013 The Android Open Source Project
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

#include <limits.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cutils/log.h>
#include <cutils/atomic.h>

#include <hardware/hardware.h>
#include <hardware/gralloc.h>

#include "gralloc_priv.h"
#include "exynos_format.h"

#include <ion/ion.h>
#include <linux/ion.h>

/*****************************************************************************/

static int gralloc_map(gralloc_module_t const* module, buffer_handle_t handle)
{
    size_t chroma_vstride = 0;
    size_t chroma_size = 0;
    size_t ext_size = 256;

    private_handle_t *hnd = (private_handle_t*)handle;

    switch (hnd->format) {
    case HAL_PIXEL_FORMAT_EXYNOS_YCbCr_420_SP_M_TILED:
        chroma_vstride = ALIGN(hnd->height / 2, 32);
        chroma_size = chroma_vstride * hnd->stride + ext_size;
        break;
    case HAL_PIXEL_FORMAT_EXYNOS_YCrCb_420_SP_M:
    case HAL_PIXEL_FORMAT_EXYNOS_YCrCb_420_SP_M_FULL:
    case HAL_PIXEL_FORMAT_EXYNOS_YCbCr_420_SP_M:
        chroma_size = hnd->stride * ALIGN(hnd->vstride / 2, 8) + ext_size;
        break;
    case HAL_PIXEL_FORMAT_EXYNOS_YV12_M:
    case HAL_PIXEL_FORMAT_EXYNOS_YCbCr_420_P_M:
        chroma_size = (hnd->vstride / 2) * ALIGN(hnd->stride / 2, 16) + ext_size;
        break;
    default:
        ALOGV("%s: unknown format: 0x%x", __func__, hnd->format);
        break;
    }

    void* mappedAddress = mmap(0, hnd->size, PROT_READ|PROT_WRITE, MAP_SHARED,
                               hnd->fd, 0);
    if (mappedAddress == MAP_FAILED) {
        ALOGE("%s: could not mmap %s", __func__, strerror(errno));
        return -errno;
    }
    ALOGV("%s: base %p %d %d %d %d\n", __func__, mappedAddress, hnd->size,
          hnd->width, hnd->height, hnd->stride);
    hnd->base = mappedAddress;

    if (hnd->fd1 >= 0) {
        void *mappedAddress1 = (void*)mmap(0, chroma_size, PROT_READ|PROT_WRITE,
                                            MAP_SHARED, hnd->fd1, 0);
        hnd->base1 = mappedAddress1;
    }
    if (hnd->fd2 >= 0) {
        void *mappedAddress2 = (void*)mmap(0, chroma_size, PROT_READ|PROT_WRITE,
                                            MAP_SHARED, hnd->fd2, 0);
        hnd->base2 = mappedAddress2;
    }

    return 0;
}

static int gralloc_unmap(gralloc_module_t const* module, buffer_handle_t handle)
{
    private_handle_t* hnd = (private_handle_t*)handle;
    size_t chroma_vstride = 0;
    size_t chroma_size = 0;
    size_t ext_size = 256;

    switch (hnd->format) {
    case HAL_PIXEL_FORMAT_EXYNOS_YCbCr_420_SP_M_TILED:
        chroma_vstride = ALIGN(hnd->height / 2, 32);
        chroma_size = chroma_vstride * hnd->stride + ext_size;
        break;
    case HAL_PIXEL_FORMAT_EXYNOS_YCrCb_420_SP_M:
    case HAL_PIXEL_FORMAT_EXYNOS_YCrCb_420_SP_M_FULL:
    case HAL_PIXEL_FORMAT_EXYNOS_YCbCr_420_SP_M:
        chroma_size = hnd->stride * ALIGN(hnd->vstride / 2, 8) + ext_size;
        break;
    case HAL_PIXEL_FORMAT_EXYNOS_YV12_M:
    case HAL_PIXEL_FORMAT_EXYNOS_YCbCr_420_P_M:
        chroma_size = (hnd->vstride / 2) * ALIGN(hnd->stride / 2, 16) + ext_size;
        break;
    default:
        ALOGV("%s: unknown format: 0x%x", __func__, hnd->format);
        break;
    }

    if (!hnd->base)
        return 0;

    if (munmap(hnd->base, hnd->size) < 0) {
        ALOGE("%s :could not unmap %s %p %d", __func__, strerror(errno),
              hnd->base, hnd->size);
    }
    ALOGV("%s: base %p %d %d %d %d\n", __func__, hnd->base, hnd->size,
          hnd->width, hnd->height, hnd->stride);
    hnd->base = 0;
    if (hnd->fd1 >= 0) {
        if (!hnd->base1)
            return 0;
        if (munmap(hnd->base1, chroma_size) < 0) {
            ALOGE("%s :could not unmap %s %p %d", __func__, strerror(errno),
                  hnd->base1, chroma_size);
        }
        hnd->base1 = 0;
    }
    if (hnd->fd2 >= 0) {
        if (!hnd->base2)
            return 0;
        if (munmap(hnd->base2, chroma_size) < 0) {
            ALOGE("%s :could not unmap %s %p %d", __func__, strerror(errno),
                  hnd->base2, chroma_size);
        }
        hnd->base2 = 0;
    }
    return 0;
}

/*****************************************************************************/

int getIonFd(gralloc_module_t const *module)
{
    private_module_t* m = const_cast<private_module_t*>(reinterpret_cast<const private_module_t*>(module));
    ALOGI("getIonFd: ionfd=%d", m->ionfd);
    if (m->ionfd <= 0)
        m->ionfd = ion_open();
    return m->ionfd;
}

static pthread_mutex_t sMapLock = PTHREAD_MUTEX_INITIALIZER;

/*****************************************************************************/

int gralloc_register_buffer(gralloc_module_t const* module,
                            buffer_handle_t handle)
{
    if (private_handle_t::validate(handle) < 0)
        return -EINVAL;

    private_handle_t* hnd = (private_handle_t*)handle;
    ALOGI("gralloc_register_buffer: fd=%d fd1=%d fd2=%d size=%d format=%d w=%d h=%d",
          hnd->fd, hnd->fd1, hnd->fd2, hnd->size, hnd->format, hnd->width, hnd->height);

    int ret;
    ret = ion_import(getIonFd(module), hnd->fd, &hnd->handle);
    if (ret)
        ALOGE("error importing handle %d %x\n", hnd->fd, hnd->format);
    if (hnd->fd1 >= 0) {
        ret = ion_import(getIonFd(module), hnd->fd1, &hnd->handle1);
        if (ret)
            ALOGE("error importing handle1 %d %x\n", hnd->fd1, hnd->format);
    }
    if (hnd->fd2 >= 0) {
        ret = ion_import(getIonFd(module), hnd->fd2, &hnd->handle2);
        if (ret)
            ALOGE("error importing handle2 %d %x\n", hnd->fd2, hnd->format);
    }


    gralloc_map(module, handle);
    return ret;
}

int gralloc_unregister_buffer(gralloc_module_t const* module,
                              buffer_handle_t handle)
{
    if (private_handle_t::validate(handle) < 0)
        return -EINVAL;

    private_handle_t* hnd = (private_handle_t*)handle;
    ALOGV("%s: base %p %d %d %d %d\n", __func__, hnd->base, hnd->size,
          hnd->width, hnd->height, hnd->stride);

    gralloc_unmap(module, handle);

    if (hnd->handle)
        ion_free(getIonFd(module), hnd->handle);
    if (hnd->handle1)
        ion_free(getIonFd(module), hnd->handle1);
    if (hnd->handle2)
        ion_free(getIonFd(module), hnd->handle2);

    return 0;
}

int gralloc_lock(gralloc_module_t const* module,
                 buffer_handle_t handle, int usage,
                 int l, int t, int w, int h,
                 void** vaddr)
{
    // this is called when a buffer is being locked for software
    // access. in thin implementation we have nothing to do since
    // not synchronization with the h/w is needed.
    // typically this is used to wait for the h/w to finish with
    // this buffer if relevant. the data cache may need to be
    // flushed or invalidated depending on the usage bits and the
    // hardware.

    if (private_handle_t::validate(handle) < 0)
        return -EINVAL;

    private_handle_t* hnd = (private_handle_t*)handle;
    if (!hnd->base)
        gralloc_map(module, hnd);
    *vaddr = (void*)hnd->base;

    // NOTE: The standard gralloc0 lock() API only returns a single pointer
    // via *vaddr (the Y/base plane).  Multi-plane YUV chroma pointers must
    // NOT be written to vaddr[1]/vaddr[2] — the caller typically passes a
    // pointer to a single void* on its stack (e.g. Gralloc0HalImpl::lock
    // passes &data).  Writing vaddr[1]/vaddr[2] overwrites the caller's
    // stack canary → __stack_chk_fail → SIGABRT.
    //
    // Callers that need chroma plane pointers must use lock_ycbcr() or
    // derive them from the private_handle_t fields (base1/base2 for
    // multi-plane, or offsets from base for single-plane NV21/YV12).

    return 0;
}

int gralloc_unlock(gralloc_module_t const* module,
                   buffer_handle_t handle)
{
    // we're done with a software buffer. nothing to do in this
    // implementation. typically this is used to flush the data cache.
    if (private_handle_t::validate(handle) < 0)
        return -EINVAL;

    private_handle_t* hnd = (private_handle_t*)handle;

    if (!((hnd->usage & GRALLOC_USAGE_SW_READ_MASK) == GRALLOC_USAGE_SW_READ_OFTEN))
        return 0;

    ion_sync_fd(getIonFd(module), hnd->fd);
    if (hnd->fd1 >= 0)
        ion_sync_fd(getIonFd(module), hnd->fd1);
    if (hnd->fd2 >= 0)
        ion_sync_fd(getIonFd(module), hnd->fd2);

    return 0;
}

int gralloc_lock_ycbcr(gralloc_module_t const* module,
                       buffer_handle_t handle, int usage,
                       int l, int t, int w, int h,
                       struct android_ycbcr *ycbcr)
{
    if (private_handle_t::validate(handle) < 0)
        return -EINVAL;

    private_handle_t* hnd = (private_handle_t*)handle;
    if (!hnd->base)
        gralloc_map(module, hnd);

    memset(ycbcr, 0, sizeof(*ycbcr));

    if (hnd->format == HAL_PIXEL_FORMAT_YCrCb_420_SP) {
        // NV21: Y plane + interleaved VU plane (single buffer)
        // Layout: [Y: stride*vstride][VU: stride*(vstride/2)]
        // NV21 = V comes first in each chroma pair, then U
        uint8_t *base = (uint8_t *)hnd->base;
        size_t ySize = hnd->stride * hnd->vstride;
        ycbcr->y = base;
        ycbcr->ystride = hnd->stride;
        ycbcr->cstride = hnd->stride;
        ycbcr->chroma_step = 2;  // interleaved VU pairs
        // NV21: Cr (V) first, then Cb (U)
        ycbcr->cr = base + ySize;       // V at even offsets
        ycbcr->cb = base + ySize + 1;   // U at odd offsets
        return 0;
    }

    if (hnd->format == HAL_PIXEL_FORMAT_YCbCr_420_888 ||
        hnd->format == HAL_PIXEL_FORMAT_YV12) {
        // Single-plane YV12 layout: [Y: stride*h][Cb: cstride*h][Cr: cstride*h]
        // cstride = ALIGN(stride/2, 16)
        uint8_t *base = (uint8_t *)hnd->base;
        size_t ySize = hnd->stride * hnd->vstride;
        size_t cStride = ALIGN(hnd->stride / 2, 16);
        ycbcr->y = base;
        ycbcr->ystride = hnd->stride;
        ycbcr->cstride = cStride;
        ycbcr->chroma_step = 1;  // separate Cb/Cr planes
        // YV12: Cb plane first, then Cr plane
        ycbcr->cb = base + ySize;
        ycbcr->cr = base + ySize + cStride * hnd->vstride;
        return 0;
    }

    // Multi-plane formats
    if (hnd->fd1 >= 0) {
        ycbcr->y = (void *)hnd->base;
        ycbcr->ystride = hnd->stride;
        if (hnd->fd2 >= 0) {
            // 3-plane (YV12): Cb and Cr separate
            ycbcr->cb = (void *)hnd->base1;
            ycbcr->cr = (void *)hnd->base2;
            ycbcr->cstride = hnd->stride / 2;
            ycbcr->chroma_step = 1;
        } else {
            // 2-plane: interleaved chroma
            // NV21M (EXYNOS_YCrCb_420_SP_M / _M_FULL): VU order → cr at even, cb at odd
            // NV12M (EXYNOS_YCbCr_420_SP_M): UV order → cb at even, cr at odd
            ycbcr->cstride = hnd->stride;
            ycbcr->chroma_step = 2;
            if (hnd->format == HAL_PIXEL_FORMAT_EXYNOS_YCrCb_420_SP_M ||
                hnd->format == HAL_PIXEL_FORMAT_EXYNOS_YCrCb_420_SP_M_FULL) {
                // NV21M: V first (even), U second (odd)
                ycbcr->cr = (void *)hnd->base1;
                ycbcr->cb = (uint8_t *)hnd->base1 + 1;
            } else {
                // NV12M: U first (even), V second (odd)
                ycbcr->cb = (void *)hnd->base1;
                ycbcr->cr = (uint8_t *)hnd->base1 + 1;
            }
        }
        return 0;
    }

    return -EINVAL;
}

int gralloc_lockAsync(gralloc_module_t const* module,
                      buffer_handle_t handle, int usage,
                      int l, int t, int w, int h,
                      void** vaddr, int fenceFd)
{
    if (fenceFd >= 0)
        close(fenceFd);
    return gralloc_lock(module, handle, usage, l, t, w, h, vaddr);
}

int gralloc_unlockAsync(gralloc_module_t const* module,
                        buffer_handle_t handle, int* fenceFd)
{
    *fenceFd = -1;
    return gralloc_unlock(module, handle);
}

int gralloc_lockAsync_ycbcr(gralloc_module_t const* module,
                            buffer_handle_t handle, int usage,
                            int l, int t, int w, int h,
                            struct android_ycbcr *ycbcr, int fenceFd)
{
    if (fenceFd >= 0)
        close(fenceFd);
    return gralloc_lock_ycbcr(module, handle, usage, l, t, w, h, ycbcr);
}
