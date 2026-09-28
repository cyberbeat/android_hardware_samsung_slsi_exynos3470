/*
 * Copyright 2012 Samsung Electronics S.LSI Co. LTD
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

/*
 * @file        Exynos_OSAL_Android.cpp
 * @brief
 * @author      Seungbeom Kim (sbcrux.kim@samsung.com)
 * @author      Hyeyeon Chung (hyeon.chung@samsung.com)
 * @author      Yunji Kim (yunji.kim@samsung.com)
 * @author      Jinsung Yang (jsgood.yang@samsung.com)
 * @version     2.0.0
 * @history
 *   2012.02.20 : Create
 */

#include <stdio.h>
#include <stdlib.h>

#include <vndk/window.h>
#include <ui/GraphicBuffer.h>
#include <ui/GraphicBufferMapper.h>
#include <ui/Rect.h>
#include <media/hardware/HardwareAPI.h>
#include <hardware/hardware.h>
#include <media/hardware/OMXPluginBase.h>
#include <media/hardware/MetadataBufferType.h>
#include <gralloc_priv.h>

#include "Exynos_OSAL_Mutex.h"
#include "Exynos_OSAL_Semaphore.h"
#include "Exynos_OMX_Baseport.h"
#include "Exynos_OMX_Basecomponent.h"
#include "Exynos_OMX_Macros.h"
#include "Exynos_OMX_Vdec.h"
#include "Exynos_OMX_Venc.h"
#include "Exynos_OSAL_Android.h"
#include "exynos_format.h"
#include "ion.h"

#include <sys/mman.h>
#include <pthread.h>
#include <stdint.h>

#undef  EXYNOS_LOG_TAG
#define EXYNOS_LOG_TAG    "Exynos_OSAL_Android"
#define EXYNOS_LOG_OFF
#include "Exynos_OSAL_Log.h"

using namespace android;

/*
 * HAL3 single-plane NV21 → NV12M chroma buffer management.
 *
 * Problem: HAL3 allocates video buffers as single-plane NV21 with
 * fd1=-1 (no separate chroma fd). The MFC hardware encoder expects
 * 2-plane NV12M with separate dmabuf fds for Y and UV planes.
 *
 * Solution: When LockANBHandle detects fd1<0 (single-plane NV21 from
 * HAL3), allocate a separate ION buffer for the chroma plane, copy
 * the chroma data from the NV21 buffer with VU→UV byte reordering
 * (NV21 YCrCb → NV12 YCbCr), and return the chroma fd as plane 1.
 *
 * Lifetime: The chroma buffer is registered in a side-table keyed by
 * fd. It survives Lock→Unlock→Enqueue→MFC→Dequeue. It is freed by
 * Exynos_OSAL_Hal3Chroma_Release(fd) called from the OMX component
 * layer after V4L2 DQBUF, when the MFC has released the buffer.
 */
#define HAL3_CHROMA_REGISTRY_SIZE 32
struct Hal3ChromaEntry {
    int     chroma_fd;    /* dmabuf fd for chroma plane */
    void   *chroma_addr;  /* mmap'd address of chroma plane */
    size_t  chroma_size;  /* allocated size in bytes */
};
static struct Hal3ChromaEntry g_hal3_chroma_registry[HAL3_CHROMA_REGISTRY_SIZE];
static pthread_mutex_t g_hal3_chroma_mutex = PTHREAD_MUTEX_INITIALIZER;
static ion_client g_hal3_ion_client = -1;

#ifdef __cplusplus
extern "C" {
#endif

int getIonFd(gralloc_module_t const *module)
{
    private_module_t* m = const_cast<private_module_t*>(reinterpret_cast<const private_module_t*>(module));
    return m->ionfd;
}

OMX_ERRORTYPE Exynos_OSAL_LockANBHandle(
    OMX_IN OMX_U32 handle,
    OMX_IN OMX_U32 width,
    OMX_IN OMX_U32 height,
    OMX_IN OMX_COLOR_FORMATTYPE format,
    OMX_OUT OMX_U32 *pStride,
    OMX_OUT OMX_PTR planes)
{
    FunctionIn();

    OMX_ERRORTYPE ret = OMX_ErrorNone;
    GraphicBufferMapper &mapper = GraphicBufferMapper::get();
    buffer_handle_t bufferHandle = (buffer_handle_t) handle;
    private_handle_t *priv_hnd = (private_handle_t *) bufferHandle;
    Rect bounds((uint32_t)width, (uint32_t)height);
    ExynosVideoPlane *vplanes = (ExynosVideoPlane *) planes;
    void *vaddr[MAX_BUFFER_PLANE];

    Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "%s: handle: 0x%x", __func__, handle);

    int usage = 0;

    switch ((OMX_U32)format) {
    case OMX_COLOR_FormatYUV420Planar:
    case OMX_COLOR_FormatYUV420SemiPlanar:
    case OMX_SEC_COLOR_FormatNV12Tiled:
        usage = GRALLOC_USAGE_SW_READ_OFTEN | GRALLOC_USAGE_SW_WRITE_OFTEN;
        break;
    default:
        usage = GRALLOC_USAGE_SW_READ_OFTEN | GRALLOC_USAGE_SW_WRITE_OFTEN;
        break;
    }

    if (mapper.lock(bufferHandle, usage, bounds, vaddr) != 0) {
        Exynos_OSAL_Log(EXYNOS_LOG_ERROR, "%s: mapper.lock() fail", __func__);
        ret = OMX_ErrorUndefined;
        goto EXIT;
    }

    vplanes[0].fd = priv_hnd->fd;
    vplanes[0].offset = 0;
    vplanes[0].addr = vaddr[0];
    vplanes[0].allocSize = 0;
    vplanes[0].dataSize = 0;

    if (priv_hnd->fd1 < 0) {
        /*
         * HAL3 single-plane NV21 path.
         *
         * The framework allocated this buffer with sNumFds=1, so fd1=-1
         * after Binder transfer. The MFC encoder requires 2-plane NV12M
         * with separate dmabuf fds. Allocate a chroma ION buffer, copy
         * the interleaved chroma data from the NV21 buffer, and reorder
         * VU→UV (NV21 YCrCb → NV12 YCbCr) for MFC compatibility.
         *
         * The chroma buffer is registered in g_hal3_chroma_registry.
         * It is freed by Exynos_OSAL_Hal3Chroma_Release(fd) after the
         * MFC has dequeued the input buffer (V4L2 DQBUF).
         */
        size_t y_size = (size_t)priv_hnd->stride * (size_t)priv_hnd->vstride;
        size_t chroma_copy_size = y_size / 2;
        /*
         * The MFC encoder calculates nAllocLen[1] = ALIGN(ALIGN_TO_16B(w) *
         * ALIGN_TO_16B(h) / 2, 256) and passes it as buf.m.planes[1].length
         * to V4L2 QBUF. The kernel validates length <= dmabuf_size.
         * If the chroma dmabuf is smaller than nAllocLen[1], QBUF returns
         * EFAULT (14 - Bad address).
         *
         * For 1920x1080: stride=1920, vstride=1080, but ALIGN_TO_16B(1080)=1088,
         * so nAllocLen[1] = ALIGN(1920*1088/2, 256) = 1044480, while
         * chroma_copy_size = 1920*1080/2 = 1036800. The 7680-byte gap causes
         * EFAULT. Allocate the aligned size to match MFC expectations.
         */
        size_t aligned_h = ((size_t)priv_hnd->vstride + 15) & ~15;
        size_t chroma_size = (size_t)priv_hnd->stride * aligned_h / 2;
        chroma_size = (chroma_size + 255) & ~255;  /* ALIGN to 256 */

        pthread_mutex_lock(&g_hal3_chroma_mutex);

        if (g_hal3_ion_client < 0) {
            g_hal3_ion_client = ion_client_create();
            /* Initialize registry fds to -1 (static init gives 0 = stdin) */
            for (int i = 0; i < HAL3_CHROMA_REGISTRY_SIZE; i++)
                g_hal3_chroma_registry[i].chroma_fd = -1;
        }

        /* Find a free registry slot */
        struct Hal3ChromaEntry *entry = NULL;
        for (int i = 0; i < HAL3_CHROMA_REGISTRY_SIZE; i++) {
            if (g_hal3_chroma_registry[i].chroma_fd < 0) {
                entry = &g_hal3_chroma_registry[i];
                break;
            }
        }
        if (entry == NULL) {
            Exynos_OSAL_Log(EXYNOS_LOG_ERROR,
                "HAL3 chroma registry full (%d entries)", HAL3_CHROMA_REGISTRY_SIZE);
            pthread_mutex_unlock(&g_hal3_chroma_mutex);
            ret = OMX_ErrorUndefined;
            goto EXIT;
        }

        /* Allocate chroma ION buffer */
        entry->chroma_fd = ion_alloc(g_hal3_ion_client, chroma_size, 0,
                                     ION_HEAP_SYSTEM_MASK, 0);
        if (entry->chroma_fd < 0) {
            Exynos_OSAL_Log(EXYNOS_LOG_ERROR,
                "HAL3 chroma ion_alloc failed: %d", entry->chroma_fd);
            entry->chroma_fd = -1;
            entry->chroma_addr = NULL;
            entry->chroma_size = 0;
            pthread_mutex_unlock(&g_hal3_chroma_mutex);
            ret = OMX_ErrorUndefined;
            goto EXIT;
        }
        entry->chroma_addr = mmap(NULL, chroma_size,
                                   PROT_READ | PROT_WRITE, MAP_SHARED,
                                   entry->chroma_fd, 0);
        if (entry->chroma_addr == MAP_FAILED) {
            Exynos_OSAL_Log(EXYNOS_LOG_ERROR,
                "HAL3 chroma mmap failed (size=%zu)", chroma_size);
            ion_free(entry->chroma_fd);
            entry->chroma_fd = -1;
            entry->chroma_addr = NULL;
            entry->chroma_size = 0;
            pthread_mutex_unlock(&g_hal3_chroma_mutex);
            ret = OMX_ErrorUndefined;
            goto EXIT;
        }
        entry->chroma_size = chroma_size;

        /*
         * Copy chroma: NV21 (VU interleaved) → NV12 (UV interleaved).
         * Source chroma starts at offset stride*vstride in the NV21
         * buffer (see grallocGetPlanePtrs in G800FCamera2Device.cpp).
         * Swap every byte pair: V,U → U,V.
         */
        {
            uint8_t *src = (uint8_t *)vaddr[0] + y_size;
            uint8_t *dst = (uint8_t *)entry->chroma_addr;
            size_t i;
            /* Only copy chroma_copy_size bytes (actual chroma data);
             * chroma_size may be larger due to alignment padding. */
            for (i = 0; i + 4 <= chroma_copy_size; i += 4) {
                uint32_t v = *(uint32_t *)(src + i);
                *(uint32_t *)(dst + i) =
                    ((v >> 8) & 0x00FF00FF) | ((v & 0x00FF00FF) << 8);
            }
            for (; i < chroma_copy_size; i += 2) {
                dst[i]     = src[i + 1];
                dst[i + 1] = src[i];
            }
        }

        /* Flush CPU cache so MFC DMA can read the chroma data */
        ion_sync(g_hal3_ion_client, entry->chroma_fd);

        vplanes[1].fd = entry->chroma_fd;
        vplanes[1].offset = 0;
        vplanes[1].addr = entry->chroma_addr;
        vplanes[1].allocSize = (unsigned int)chroma_size;
        vplanes[1].dataSize = (unsigned int)chroma_size;

        vplanes[2].fd = -1;
        vplanes[2].offset = 0;
        vplanes[2].addr = NULL;
        vplanes[2].allocSize = 0;
        vplanes[2].dataSize = 0;

        Exynos_OSAL_Log(EXYNOS_LOG_INFO,
            "HAL3 NV21→NV12M: chroma fd=%d size=%zu "
            "(Y fd=%d stride=%d vstride=%d)",
            entry->chroma_fd, chroma_size,
            priv_hnd->fd, priv_hnd->stride, priv_hnd->vstride);

        pthread_mutex_unlock(&g_hal3_chroma_mutex);
    } else {
        vplanes[1].fd = priv_hnd->fd1;
        vplanes[1].offset = 0;
        vplanes[1].addr = vaddr[1];
        vplanes[1].allocSize = 0;
        vplanes[1].dataSize = 0;
        vplanes[2].fd = priv_hnd->fd2;
        vplanes[2].offset = 0;
        vplanes[2].addr = vaddr[2];
        vplanes[2].allocSize = 0;
        vplanes[2].dataSize = 0;
    }

    *pStride = priv_hnd->stride;

    Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "%s: buffer locked: 0x%x", __func__, *vaddr);

EXIT:
    FunctionOut();

    return ret;
}

OMX_ERRORTYPE Exynos_OSAL_UnlockANBHandle(OMX_IN OMX_U32 handle)
{
    FunctionIn();

    OMX_ERRORTYPE ret = OMX_ErrorNone;
    GraphicBufferMapper &mapper = GraphicBufferMapper::get();
    buffer_handle_t bufferHandle = (buffer_handle_t) handle;

    Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "%s: handle: 0x%x", __func__, handle);

    if (mapper.unlock(bufferHandle) != 0) {
        Exynos_OSAL_Log(EXYNOS_LOG_ERROR, "%s: mapper.unlock() fail", __func__);
        ret = OMX_ErrorUndefined;
        goto EXIT;
    }

    Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "%s: buffer unlocked: 0x%x", __func__, handle);

EXIT:
    FunctionOut();

    return ret;
}

/*
 * Release a HAL3 chroma buffer by fd.
 *
 * Called from the OMX component layer after V4L2 DQBUF, when the MFC
 * has finished processing the input buffer. If the fd was registered
 * by LockANBHandle as a HAL3 chroma buffer, it is munmap'd and ion_free'd.
 * If the fd is not in the registry (e.g. a real gralloc multi-plane fd),
 * this is a no-op.
 */
void Exynos_OSAL_Hal3Chroma_Release(int fd)
{
    if (fd < 0)
        return;

    pthread_mutex_lock(&g_hal3_chroma_mutex);

    for (int i = 0; i < HAL3_CHROMA_REGISTRY_SIZE; i++) {
        if (g_hal3_chroma_registry[i].chroma_fd == fd) {
            Exynos_OSAL_Log(EXYNOS_LOG_INFO,
                "HAL3 chroma release fd=%d size=%zu",
                fd, g_hal3_chroma_registry[i].chroma_size);
            if (g_hal3_chroma_registry[i].chroma_addr &&
                g_hal3_chroma_registry[i].chroma_addr != MAP_FAILED)
                munmap(g_hal3_chroma_registry[i].chroma_addr,
                       g_hal3_chroma_registry[i].chroma_size);
            ion_free(fd);
            g_hal3_chroma_registry[i].chroma_fd = -1;
            g_hal3_chroma_registry[i].chroma_addr = NULL;
            g_hal3_chroma_registry[i].chroma_size = 0;
            break;
        }
    }

    pthread_mutex_unlock(&g_hal3_chroma_mutex);
}

OMX_COLOR_FORMATTYPE Exynos_OSAL_GetANBColorFormat(OMX_IN OMX_U32 handle)
{
    FunctionIn();

    OMX_COLOR_FORMATTYPE ret = OMX_COLOR_FormatUnused;
    private_handle_t *priv_hnd = (private_handle_t *) handle;

    ret = Exynos_OSAL_Hal2OMXPixelFormat(priv_hnd->format);
    Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "ColorFormat: 0x%x", ret);

    FunctionOut();

    return ret;
}

OMX_U32 Exynos_OSAL_GetANBStride(OMX_IN OMX_U32 handle)
{
    FunctionIn();

    OMX_U32 nStride = 0;
    private_handle_t *priv_hnd = (private_handle_t *) handle;

    nStride = priv_hnd->stride;

    FunctionOut();

    return nStride;
}

OMX_ERRORTYPE Exynos_OSAL_LockMetaData(
    OMX_IN OMX_PTR pBuffer,
    OMX_IN OMX_U32 width,
    OMX_IN OMX_U32 height,
    OMX_IN OMX_COLOR_FORMATTYPE format,
    OMX_OUT OMX_U32 *pStride,
    OMX_OUT OMX_PTR planes)
{
    FunctionIn();

    OMX_ERRORTYPE ret = OMX_ErrorNone;
    OMX_PTR pBuf;

    ret = Exynos_OSAL_GetInfoFromMetaData((OMX_BYTE)pBuffer, &pBuf);
    if (ret == OMX_ErrorNone) {
        ret = Exynos_OSAL_LockANBHandle((OMX_U32)pBuf, width, height, format, pStride, planes);
    }

    FunctionOut();

    return ret;
}

OMX_ERRORTYPE Exynos_OSAL_UnlockMetaData(OMX_IN OMX_PTR pBuffer)
{
    FunctionIn();

    OMX_ERRORTYPE ret = OMX_ErrorNone;
    OMX_PTR pBuf;

    ret = Exynos_OSAL_GetInfoFromMetaData((OMX_BYTE)pBuffer, &pBuf);
    if (ret == OMX_ErrorNone)
        ret = Exynos_OSAL_UnlockANBHandle((OMX_U32)pBuf);

    FunctionOut();

    return ret;
}

OMX_HANDLETYPE Exynos_OSAL_RefANB_Create()
{
    int i = 0;
    EXYNOS_OMX_REF_HANDLE *phREF = NULL;
    gralloc_module_t      *module = NULL;

    OMX_ERRORTYPE err = OMX_ErrorNone;

    FunctionIn();

    phREF = (EXYNOS_OMX_REF_HANDLE *) Exynos_OSAL_Malloc(sizeof(EXYNOS_OMX_REF_HANDLE));
    if (phREF == NULL)
        goto EXIT;

    Exynos_OSAL_Memset(phREF, 0, sizeof(EXYNOS_OMX_REF_HANDLE));
    for (i = 0; i < MAX_BUFFER_REF; i++) {
        phREF->SharedBuffer[i].BufferFd  = -1;
        phREF->SharedBuffer[i].BufferFd1 = -1;
        phREF->SharedBuffer[i].BufferFd2 = -1;
    }

    hw_get_module(GRALLOC_HARDWARE_MODULE_ID, (const hw_module_t **)&module);
    phREF->pGrallocModule = (OMX_PTR)module;

    err = Exynos_OSAL_MutexCreate(&phREF->hMutex);
    if (err != OMX_ErrorNone) {
        Exynos_OSAL_Free(phREF);
        phREF = NULL;
    }

EXIT:
    FunctionOut();

    return ((OMX_HANDLETYPE)phREF);
}

OMX_ERRORTYPE Exynos_OSAL_RefANB_Reset(OMX_HANDLETYPE hREF)
{
    int i = 0;
    OMX_ERRORTYPE ret = OMX_ErrorNone;
    EXYNOS_OMX_REF_HANDLE *phREF = (EXYNOS_OMX_REF_HANDLE *)hREF;
    gralloc_module_t* module = NULL;

    FunctionIn();

    if (phREF == NULL) {
        ret = OMX_ErrorBadParameter;
        goto EXIT;
    }

    module = (gralloc_module_t *)phREF->pGrallocModule;

    Exynos_OSAL_MutexLock(phREF->hMutex);
    for (i = 0; i < MAX_BUFFER_REF; i++) {
        if (phREF->SharedBuffer[i].BufferFd > -1) {
            while(phREF->SharedBuffer[i].cnt > 0) {
                if (phREF->SharedBuffer[i].BufferFd > -1)
                    ion_decRef(getIonFd(module), phREF->SharedBuffer[i].pIonHandle);
                if (phREF->SharedBuffer[i].BufferFd1 > -1)
                    ion_decRef(getIonFd(module), phREF->SharedBuffer[i].pIonHandle1);
                if (phREF->SharedBuffer[i].BufferFd2 > -1)
                    ion_decRef(getIonFd(module), phREF->SharedBuffer[i].pIonHandle2);
                phREF->SharedBuffer[i].cnt--;
            }
            phREF->SharedBuffer[i].BufferFd    = -1;
            phREF->SharedBuffer[i].BufferFd1   = -1;
            phREF->SharedBuffer[i].BufferFd2   = -1;
            phREF->SharedBuffer[i].pIonHandle  = NULL;
            phREF->SharedBuffer[i].pIonHandle1 = NULL;
            phREF->SharedBuffer[i].pIonHandle2 = NULL;
        }
    }
    Exynos_OSAL_MutexUnlock(phREF->hMutex);

EXIT:
    FunctionOut();

    return ret;
}

OMX_ERRORTYPE Exynos_OSAL_RefANB_Terminate(OMX_HANDLETYPE hREF)
{
    OMX_ERRORTYPE ret = OMX_ErrorNone;
    EXYNOS_OMX_REF_HANDLE *phREF = (EXYNOS_OMX_REF_HANDLE *)hREF;
    FunctionIn();

    if (phREF == NULL) {
        ret = OMX_ErrorBadParameter;
        goto EXIT;
    }

    Exynos_OSAL_RefANB_Reset(phREF);

    phREF->pGrallocModule = NULL;

    ret = Exynos_OSAL_MutexTerminate(phREF->hMutex);
    if (ret != OMX_ErrorNone)
        goto EXIT;

    Exynos_OSAL_Free(phREF);
    phREF = NULL;

EXIT:
    FunctionOut();

    return ret;
}

OMX_ERRORTYPE Exynos_OSAL_RefANB_Increase(OMX_HANDLETYPE hREF, OMX_PTR pBuffer)
{
    int i;
    OMX_ERRORTYPE ret = OMX_ErrorNone;
    buffer_handle_t bufferHandle = (buffer_handle_t) pBuffer; //pANB->handle
    private_handle_t *priv_hnd = (private_handle_t *) bufferHandle;
    EXYNOS_OMX_REF_HANDLE *phREF = (EXYNOS_OMX_REF_HANDLE *)hREF;
    gralloc_module_t* module = NULL;

    unsigned long *pIonHandle;
    unsigned long *pIonHandle1;
    unsigned long *pIonHandle2;

    FunctionIn();

    if (phREF == NULL) {
        ret = OMX_ErrorBadParameter;
        goto EXIT;
    }

    module = (gralloc_module_t *)phREF->pGrallocModule;

    Exynos_OSAL_MutexLock(phREF->hMutex);

    if (priv_hnd->fd >= 0) {
        ion_incRef(getIonFd(module), priv_hnd->fd, &pIonHandle);
    }
    if (priv_hnd->fd1 >= 0) {
        ion_incRef(getIonFd(module), priv_hnd->fd1, &pIonHandle1);
    }
    if (priv_hnd->fd2 >= 0) {
        ion_incRef(getIonFd(module), priv_hnd->fd2, &pIonHandle2);
    }

    for (i = 0; i < MAX_BUFFER_REF; i++) {
        if (phREF->SharedBuffer[i].BufferFd == priv_hnd->fd) {
            phREF->SharedBuffer[i].cnt++;
            break;
        }
    }

    if (i >=  MAX_BUFFER_REF) {
        for (i = 0; i < MAX_BUFFER_REF; i++) {
            if (phREF->SharedBuffer[i].BufferFd == -1) {
                phREF->SharedBuffer[i].BufferFd    = priv_hnd->fd;
                phREF->SharedBuffer[i].BufferFd1   = priv_hnd->fd1;
                phREF->SharedBuffer[i].BufferFd2   = priv_hnd->fd2;
                phREF->SharedBuffer[i].pIonHandle  = pIonHandle;
                phREF->SharedBuffer[i].pIonHandle1 = pIonHandle1;
                phREF->SharedBuffer[i].pIonHandle2 = pIonHandle2;
                phREF->SharedBuffer[i].cnt++;
                break;
            }
        }
    }

    Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "inc fd:%d cnt:%d", phREF->SharedBuffer[i].BufferFd, phREF->SharedBuffer[i].cnt);

    Exynos_OSAL_MutexUnlock(phREF->hMutex);

    if (i >=  MAX_BUFFER_REF) {
        ret = OMX_ErrorUndefined;
    }

EXIT:
    FunctionOut();

    return ret;
}

OMX_ERRORTYPE Exynos_OSAL_RefANB_Decrease(OMX_HANDLETYPE hREF, OMX_U32 BufferFd)
{
    int i;
    OMX_ERRORTYPE ret = OMX_ErrorNone;
    EXYNOS_OMX_REF_HANDLE *phREF = (EXYNOS_OMX_REF_HANDLE *)hREF;
    gralloc_module_t* module = NULL;

    FunctionIn();

    if ((phREF == NULL) || (BufferFd < 0)) {
        ret = OMX_ErrorBadParameter;
        goto EXIT;
    }

    module = (gralloc_module_t *)phREF->pGrallocModule;

    Exynos_OSAL_MutexLock(phREF->hMutex);

    for (i = 0; i < MAX_BUFFER_REF; i++) {
        if (phREF->SharedBuffer[i].BufferFd == BufferFd) {
            if (phREF->SharedBuffer[i].BufferFd > -1)
                ion_decRef(getIonFd(module), phREF->SharedBuffer[i].pIonHandle);
            if (phREF->SharedBuffer[i].BufferFd1 > -1)
                ion_decRef(getIonFd(module), phREF->SharedBuffer[i].pIonHandle1);
            if (phREF->SharedBuffer[i].BufferFd2 > -1)
                ion_decRef(getIonFd(module), phREF->SharedBuffer[i].pIonHandle2);
            phREF->SharedBuffer[i].cnt--;
            if (phREF->SharedBuffer[i].cnt == 0) {
                phREF->SharedBuffer[i].BufferFd    = -1;
                phREF->SharedBuffer[i].BufferFd1   = -1;
                phREF->SharedBuffer[i].BufferFd2   = -1;
                phREF->SharedBuffer[i].pIonHandle  = NULL;
                phREF->SharedBuffer[i].pIonHandle1 = NULL;
                phREF->SharedBuffer[i].pIonHandle2 = NULL;
            }
            break;
        }
    }
    Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "dec fd:%d cnt:%d", phREF->SharedBuffer[i].BufferFd, phREF->SharedBuffer[i].cnt);

    Exynos_OSAL_MutexUnlock(phREF->hMutex);

    if (i >=  MAX_BUFFER_REF) {
        ret = OMX_ErrorUndefined;
        goto EXIT;
    }

EXIT:
    FunctionOut();

    return ret;
}

OMX_ERRORTYPE useAndroidNativeBuffer(
    EXYNOS_OMX_BASEPORT      *pExynosPort,
    OMX_BUFFERHEADERTYPE **ppBufferHdr,
    OMX_U32                nPortIndex,
    OMX_PTR                pAppPrivate,
    OMX_U32                nSizeBytes,
    OMX_U8                *pBuffer)
{
    OMX_ERRORTYPE         ret = OMX_ErrorNone;
    OMX_BUFFERHEADERTYPE *temp_bufferHeader = NULL;
    unsigned int          i = 0;
    OMX_U32               width, height;
    OMX_U32               stride;
    ExynosVideoPlane      planes[MAX_BUFFER_PLANE];

    FunctionIn();

    if (pExynosPort == NULL) {
        ret = OMX_ErrorBadParameter;
        goto EXIT;
    }
    if (pExynosPort->portState != OMX_StateIdle) {
        ret = OMX_ErrorIncorrectStateOperation;
        goto EXIT;
    }
    if (CHECK_PORT_TUNNELED(pExynosPort) && CHECK_PORT_BUFFER_SUPPLIER(pExynosPort)) {
        ret = OMX_ErrorBadPortIndex;
        goto EXIT;
    }

    temp_bufferHeader = (OMX_BUFFERHEADERTYPE *)Exynos_OSAL_Malloc(sizeof(OMX_BUFFERHEADERTYPE));
    if (temp_bufferHeader == NULL) {
        ret = OMX_ErrorInsufficientResources;
        goto EXIT;
    }
    Exynos_OSAL_Memset(temp_bufferHeader, 0, sizeof(OMX_BUFFERHEADERTYPE));

    for (i = 0; i < pExynosPort->portDefinition.nBufferCountActual; i++) {
        if (pExynosPort->bufferStateAllocate[i] == BUFFER_STATE_FREE) {
            pExynosPort->extendBufferHeader[i].OMXBufferHeader = temp_bufferHeader;
            pExynosPort->bufferStateAllocate[i] = (BUFFER_STATE_ASSIGNED | HEADER_STATE_ALLOCATED);
            INIT_SET_SIZE_VERSION(temp_bufferHeader, OMX_BUFFERHEADERTYPE);
            android_native_buffer_t *pANB = (android_native_buffer_t *) pBuffer;
            temp_bufferHeader->pBuffer = (OMX_U8 *)pANB->handle;
            temp_bufferHeader->nAllocLen      = nSizeBytes;
            temp_bufferHeader->pAppPrivate    = pAppPrivate;
            if (nPortIndex == INPUT_PORT_INDEX)
                temp_bufferHeader->nInputPortIndex = INPUT_PORT_INDEX;
            else
                temp_bufferHeader->nOutputPortIndex = OUTPUT_PORT_INDEX;

            width = pExynosPort->portDefinition.format.video.nFrameWidth;
            height = pExynosPort->portDefinition.format.video.nFrameHeight;
            Exynos_OSAL_LockANBHandle((OMX_U32)temp_bufferHeader->pBuffer, width, height,
                                pExynosPort->portDefinition.format.video.eColorFormat,
                                &stride, planes);
            pExynosPort->extendBufferHeader[i].buf_fd[0] = planes[0].fd;
            pExynosPort->extendBufferHeader[i].pYUVBuf[0] = planes[0].addr;
            pExynosPort->extendBufferHeader[i].buf_fd[1] = planes[1].fd;
            pExynosPort->extendBufferHeader[i].pYUVBuf[1] = planes[1].addr;
            pExynosPort->extendBufferHeader[i].buf_fd[2] = planes[2].fd;
            pExynosPort->extendBufferHeader[i].pYUVBuf[2] = planes[2].addr;
            Exynos_OSAL_UnlockANBHandle((OMX_U32)temp_bufferHeader->pBuffer);
            Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "useAndroidNativeBuffer: buf %d pYUVBuf[0]:0x%x (fd:%d), pYUVBuf[1]:0x%x (fd:%d)",
                            i, pExynosPort->extendBufferHeader[i].pYUVBuf[0], planes[0].fd,
                            pExynosPort->extendBufferHeader[i].pYUVBuf[1], planes[1].fd);

            pExynosPort->assignedBufferNum++;
            if (pExynosPort->assignedBufferNum == pExynosPort->portDefinition.nBufferCountActual) {
                pExynosPort->portDefinition.bPopulated = OMX_TRUE;
                /* Exynos_OSAL_MutexLock(pExynosComponent->compMutex); */
                Exynos_OSAL_SemaphorePost(pExynosPort->loadedResource);
                /* Exynos_OSAL_MutexUnlock(pExynosComponent->compMutex); */
            }
            *ppBufferHdr = temp_bufferHeader;
            ret = OMX_ErrorNone;

            goto EXIT;
        }
    }

    Exynos_OSAL_Free(temp_bufferHeader);
    ret = OMX_ErrorInsufficientResources;

EXIT:
    FunctionOut();

    return ret;
}

OMX_ERRORTYPE Exynos_OSAL_GetANBParameter(
    OMX_IN OMX_HANDLETYPE hComponent,
    OMX_IN OMX_INDEXTYPE  nIndex,
    OMX_INOUT OMX_PTR     ComponentParameterStructure)
{
    OMX_ERRORTYPE          ret = OMX_ErrorNone;
    OMX_COMPONENTTYPE     *pOMXComponent = NULL;
    EXYNOS_OMX_BASECOMPONENT *pExynosComponent = NULL;

    FunctionIn();

    if (hComponent == NULL) {
        ret = OMX_ErrorBadParameter;
        goto EXIT;
    }

    pOMXComponent = (OMX_COMPONENTTYPE *)hComponent;
    ret = Exynos_OMX_Check_SizeVersion(pOMXComponent, sizeof(OMX_COMPONENTTYPE));
    if (ret != OMX_ErrorNone) {
        goto EXIT;
    }

    if (pOMXComponent->pComponentPrivate == NULL) {
        ret = OMX_ErrorBadParameter;
        goto EXIT;
    }

    pExynosComponent = (EXYNOS_OMX_BASECOMPONENT *)pOMXComponent->pComponentPrivate;
    if (pExynosComponent->currentState == OMX_StateInvalid ) {
        ret = OMX_ErrorInvalidState;
        goto EXIT;
    }

    if (ComponentParameterStructure == NULL) {
        ret = OMX_ErrorBadParameter;
        goto EXIT;
    }

    switch ((EXYNOS_OMX_INDEXTYPE)nIndex) {
    case OMX_IndexParamGetAndroidNativeBuffer:
    {
        GetAndroidNativeBufferUsageParams *pANBParams = (GetAndroidNativeBufferUsageParams *) ComponentParameterStructure;
        OMX_U32 portIndex = pANBParams->nPortIndex;

        Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "%s: OMX_IndexParamGetAndroidNativeBuffer", __func__);

        ret = Exynos_OMX_Check_SizeVersion(pANBParams, sizeof(GetAndroidNativeBufferUsageParams));
        if (ret != OMX_ErrorNone) {
            Exynos_OSAL_Log(EXYNOS_LOG_ERROR, "%s: Exynos_OMX_Check_SizeVersion(GetAndroidNativeBufferUsageParams) is failed", __func__);
            goto EXIT;
        }

        if (portIndex >= pExynosComponent->portParam.nPorts) {
            ret = OMX_ErrorBadPortIndex;
            goto EXIT;
        }

        /* NOTE: OMX_IndexParamGetAndroidNativeBuffer returns original 'nUsage' without any
         * modifications since currently not defined what the 'nUsage' is for.
         */
        pANBParams->nUsage |= (GRALLOC_USAGE_HW_TEXTURE | GRALLOC_USAGE_EXTERNAL_DISP);
    }
        break;

    default:
    {
        Exynos_OSAL_Log(EXYNOS_LOG_ERROR, "%s: Unsupported index (%d)", __func__, nIndex);
        ret = OMX_ErrorUnsupportedIndex;
        goto EXIT;
    }
        break;
    }

EXIT:
    FunctionOut();

    return ret;
}

OMX_ERRORTYPE Exynos_OSAL_SetANBParameter(
    OMX_IN OMX_HANDLETYPE hComponent,
    OMX_IN OMX_INDEXTYPE  nIndex,
    OMX_IN OMX_PTR        ComponentParameterStructure)
{
    OMX_ERRORTYPE          ret = OMX_ErrorNone;
    OMX_COMPONENTTYPE     *pOMXComponent = NULL;
    EXYNOS_OMX_BASECOMPONENT *pExynosComponent = NULL;

    FunctionIn();

    if (hComponent == NULL) {
        ret = OMX_ErrorBadParameter;
        goto EXIT;
    }

    pOMXComponent = (OMX_COMPONENTTYPE *)hComponent;
    ret = Exynos_OMX_Check_SizeVersion(pOMXComponent, sizeof(OMX_COMPONENTTYPE));
    if (ret != OMX_ErrorNone) {
        goto EXIT;
    }

    if (pOMXComponent->pComponentPrivate == NULL) {
        ret = OMX_ErrorBadParameter;
        goto EXIT;
    }

    pExynosComponent = (EXYNOS_OMX_BASECOMPONENT *)pOMXComponent->pComponentPrivate;
    if (pExynosComponent->currentState == OMX_StateInvalid ) {
        ret = OMX_ErrorInvalidState;
        goto EXIT;
    }

    if (ComponentParameterStructure == NULL) {
        ret = OMX_ErrorBadParameter;
        goto EXIT;
    }

    switch ((EXYNOS_OMX_INDEXTYPE)nIndex) {
    case OMX_IndexParamEnableAndroidBuffers:
    {
        EXYNOS_OMX_VIDEODEC_COMPONENT *pVideoDec = (EXYNOS_OMX_VIDEODEC_COMPONENT *)pExynosComponent->hComponentHandle;
        (void)pVideoDec;
        EnableAndroidNativeBuffersParams *pANBParams = (EnableAndroidNativeBuffersParams *) ComponentParameterStructure;
        OMX_U32 portIndex = pANBParams->nPortIndex;
        EXYNOS_OMX_BASEPORT *pExynosPort = NULL;

        Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "%s: OMX_IndexParamEnableAndroidNativeBuffers", __func__);

        ret = Exynos_OMX_Check_SizeVersion(pANBParams, sizeof(EnableAndroidNativeBuffersParams));
        if (ret != OMX_ErrorNone) {
            Exynos_OSAL_Log(EXYNOS_LOG_ERROR, "%s: Exynos_OMX_Check_SizeVersion(EnableAndroidNativeBuffersParams) is failed", __func__);
            goto EXIT;
        }

        if (portIndex >= pExynosComponent->portParam.nPorts) {
            ret = OMX_ErrorBadPortIndex;
            goto EXIT;
        }

        pExynosPort = &pExynosComponent->pExynosPort[portIndex];
        if (CHECK_PORT_TUNNELED(pExynosPort) && CHECK_PORT_BUFFER_SUPPLIER(pExynosPort)) {
            ret = OMX_ErrorBadPortIndex;
            goto EXIT;
        }

        /* ANB and DPB Buffer Sharing */
        if (pExynosPort->bStoreMetaData != OMX_TRUE)
            pExynosPort->bIsANBEnabled = pANBParams->enable;
        if ((portIndex == OUTPUT_PORT_INDEX) &&
            (pExynosPort->bIsANBEnabled == OMX_TRUE) &&
            ((pExynosPort->bufferProcessType & BUFFER_ANBSHARE) == BUFFER_ANBSHARE)) {
            pExynosPort->bufferProcessType = BUFFER_SHARE;
            pExynosPort->portDefinition.format.video.eColorFormat = (OMX_COLOR_FORMATTYPE)OMX_SEC_COLOR_FormatNV12Tiled;
            Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "OMX_IndexParamEnableAndroidBuffers & bufferProcessType change to BUFFER_SHARE");
        } else if ((portIndex == OUTPUT_PORT_INDEX) &&
            (pExynosPort->bStoreMetaData == OMX_FALSE && pExynosPort->bIsANBEnabled == OMX_FALSE) &&
            pExynosPort->bufferProcessType == BUFFER_SHARE) {
            pExynosPort->bufferProcessType = (EXYNOS_OMX_BUFFERPROCESS_TYPE)(BUFFER_COPY | BUFFER_ANBSHARE);
            pExynosPort->portDefinition.format.video.eColorFormat = OMX_COLOR_FormatYUV420Planar;
            Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "No OMX_IndexParamEnableAndroidBuffers => reset bufferProcessType");
        }
    }
        break;

    case OMX_IndexParamUseAndroidNativeBuffer:
    {
        EXYNOS_OMX_VIDEODEC_COMPONENT *pVideoDec = (EXYNOS_OMX_VIDEODEC_COMPONENT *)pExynosComponent->hComponentHandle;
        (void)pVideoDec;
        UseAndroidNativeBufferParams *pANBParams = (UseAndroidNativeBufferParams *) ComponentParameterStructure;
        OMX_U32 portIndex = pANBParams->nPortIndex;
        EXYNOS_OMX_BASEPORT *pExynosPort = NULL;
        android_native_buffer_t *pANB;
        OMX_U32 nSizeBytes;

        Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "%s: OMX_IndexParamUseAndroidNativeBuffer, portIndex: %d", __func__, portIndex);

        ret = Exynos_OMX_Check_SizeVersion(pANBParams, sizeof(UseAndroidNativeBufferParams));
        if (ret != OMX_ErrorNone) {
            Exynos_OSAL_Log(EXYNOS_LOG_ERROR, "%s: Exynos_OMX_Check_SizeVersion(UseAndroidNativeBufferParams) is failed", __func__);
            goto EXIT;
        }

        if (portIndex >= pExynosComponent->portParam.nPorts) {
            ret = OMX_ErrorBadPortIndex;
            goto EXIT;
        }

        pExynosPort = &pExynosComponent->pExynosPort[portIndex];
        if (CHECK_PORT_TUNNELED(pExynosPort) && CHECK_PORT_BUFFER_SUPPLIER(pExynosPort)) {
            ret = OMX_ErrorBadPortIndex;
            goto EXIT;
        }

        if (pExynosPort->portState != OMX_StateIdle) {
            Exynos_OSAL_Log(EXYNOS_LOG_ERROR, "%s: Port state should be IDLE", __func__);
            ret = OMX_ErrorIncorrectStateOperation;
            goto EXIT;
        }

        pANB = pANBParams->nativeBuffer.get();

        /* MALI alignment restriction */
        nSizeBytes = ALIGN(pANB->width, 16) * ALIGN(pANB->height, 16);
        nSizeBytes += ALIGN(pANB->width / 2, 16) * ALIGN(pANB->height / 2, 16) * 2;

        ret = useAndroidNativeBuffer(pExynosPort,
                                     pANBParams->bufferHeader,
                                     pANBParams->nPortIndex,
                                     pANBParams->pAppPrivate,
                                     nSizeBytes,
                                     (OMX_U8 *) pANB);
        if (ret != OMX_ErrorNone) {
            Exynos_OSAL_Log(EXYNOS_LOG_ERROR, "%s: useAndroidNativeBuffer is failed: err=0x%x", __func__,ret);
            goto EXIT;
        }
    }
        break;

    case OMX_IndexParamStoreMetaDataBuffer:
    {
        StoreMetaDataInBuffersParams *pANBParams = (StoreMetaDataInBuffersParams *) ComponentParameterStructure;
        OMX_U32 portIndex = pANBParams->nPortIndex;
        EXYNOS_OMX_BASEPORT *pExynosPort = NULL;

        Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "%s: OMX_IndexParamStoreMetaDataBuffer", __func__);

        ret = Exynos_OMX_Check_SizeVersion(pANBParams, sizeof(StoreMetaDataInBuffersParams));
        if (ret != OMX_ErrorNone) {
            Exynos_OSAL_Log(EXYNOS_LOG_ERROR, "%s: Exynos_OMX_Check_SizeVersion(StoreMetaDataInBuffersParams) is failed", __func__);
            goto EXIT;
        }

        if (portIndex >= pExynosComponent->portParam.nPorts) {
            ret = OMX_ErrorBadPortIndex;
            goto EXIT;
        }

        pExynosPort = &pExynosComponent->pExynosPort[portIndex];
        if (CHECK_PORT_TUNNELED(pExynosPort) && CHECK_PORT_BUFFER_SUPPLIER(pExynosPort)) {
            ret = OMX_ErrorBadPortIndex;
            goto EXIT;
        }

        pExynosPort->bStoreMetaData = pANBParams->bStoreMetaData;
        if (pExynosComponent->codecType == HW_VIDEO_ENC_CODEC) {
            EXYNOS_OMX_VIDEOENC_COMPONENT *pVideoEnc = (EXYNOS_OMX_VIDEOENC_COMPONENT *)pExynosComponent->hComponentHandle;;
            (void)pVideoEnc;
        } else if (pExynosComponent->codecType == HW_VIDEO_DEC_CODEC) {
            EXYNOS_OMX_VIDEODEC_COMPONENT *pVideoDec = (EXYNOS_OMX_VIDEODEC_COMPONENT *)pExynosComponent->hComponentHandle;;
            (void)pVideoDec;
            if ((portIndex == OUTPUT_PORT_INDEX) &&
                (pExynosPort->bStoreMetaData == OMX_TRUE) &&
                ((pExynosPort->bufferProcessType & BUFFER_ANBSHARE) == BUFFER_ANBSHARE)) {
                pExynosPort->bufferProcessType = BUFFER_SHARE;
                pExynosPort->portDefinition.format.video.eColorFormat = (OMX_COLOR_FORMATTYPE)OMX_SEC_COLOR_FormatNV12Tiled;
                Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "OMX_IndexParamStoreMetaDataBuffer & bufferProcessType change to BUFFER_SHARE");
            } else if ((portIndex == OUTPUT_PORT_INDEX) &&
                (pExynosPort->bStoreMetaData == OMX_FALSE && pExynosPort->bIsANBEnabled == OMX_FALSE) &&
                pExynosPort->bufferProcessType == BUFFER_SHARE) {
                pExynosPort->bufferProcessType = (EXYNOS_OMX_BUFFERPROCESS_TYPE)(BUFFER_COPY | BUFFER_ANBSHARE);
                pExynosPort->portDefinition.format.video.eColorFormat = OMX_COLOR_FormatYUV420Planar;
                Exynos_OSAL_Log(EXYNOS_LOG_TRACE, "No OMX_IndexParamStoreMetaDataBuffer => reset bufferProcessType");
            }

        }
    }
        break;

    default:
    {
        Exynos_OSAL_Log(EXYNOS_LOG_ERROR, "%s: Unsupported index (%d)", __func__, nIndex);
        ret = OMX_ErrorUnsupportedIndex;
        goto EXIT;
    }
        break;
    }

EXIT:
    FunctionOut();

    return ret;
}

OMX_ERRORTYPE Exynos_OSAL_GetInfoFromMetaData(OMX_IN OMX_BYTE pBuffer,
                                           OMX_OUT OMX_PTR *ppBuf)
{
    OMX_ERRORTYPE      ret = OMX_ErrorNone;
    MetadataBufferType type;

    FunctionIn();

/*
 * meta data contains the following data format.
 * payload depends on the MetadataBufferType
 * --------------------------------------------------------------
 * | MetadataBufferType                         |          payload                           |
 * --------------------------------------------------------------
 *
 * If MetadataBufferType is kMetadataBufferTypeCameraSource, then
 * --------------------------------------------------------------
 * | kMetadataBufferTypeCameraSource  | physical addr. of Y |physical addr. of CbCr |
 * --------------------------------------------------------------
 *
 * If MetadataBufferType is kMetadataBufferTypeGrallocSource, then
 * --------------------------------------------------------------
 * | kMetadataBufferTypeGrallocSource    | buffer_handle_t |
 * --------------------------------------------------------------
 */

    /* MetadataBufferType */
    Exynos_OSAL_Memcpy(&type, (MetadataBufferType *)pBuffer, sizeof(MetadataBufferType));

    if (type == kMetadataBufferTypeCameraSource) {
        void *pAddress = NULL;

        /* Address. of Y */
        Exynos_OSAL_Memcpy(&pAddress, pBuffer + sizeof(MetadataBufferType), sizeof(void *));
        ppBuf[0] = (void *)pAddress;

        /* Address. of CbCr */
        Exynos_OSAL_Memcpy(&pAddress, pBuffer + sizeof(MetadataBufferType) + sizeof(void *), sizeof(void *));
        ppBuf[1] = (void *)pAddress;

    } else if (type == kMetadataBufferTypeGrallocSource) {
        buffer_handle_t    pBufHandle;

        /* buffer_handle_t */
        Exynos_OSAL_Memcpy(&pBufHandle, pBuffer + sizeof(MetadataBufferType), sizeof(buffer_handle_t));
        ppBuf[0] = (OMX_PTR)pBufHandle;
    }

    FunctionOut();

    return ret;
}

OMX_ERRORTYPE Exynos_OSAL_SetPrependSPSPPSToIDR(
    OMX_PTR pComponentParameterStructure,
    OMX_PTR pbPrependSpsPpsToIdr)
{
    OMX_ERRORTYPE                    ret        = OMX_ErrorNone;
    PrependSPSPPSToIDRFramesParams  *pANBParams = (PrependSPSPPSToIDRFramesParams *)pComponentParameterStructure;

    ret = Exynos_OMX_Check_SizeVersion(pANBParams, sizeof(PrependSPSPPSToIDRFramesParams));
    if (ret != OMX_ErrorNone) {
        Exynos_OSAL_Log(EXYNOS_LOG_ERROR, "%s: Exynos_OMX_Check_SizeVersion(PrependSPSPPSToIDRFrames) is failed", __func__);
        goto EXIT;
    }

    (*((OMX_BOOL *)pbPrependSpsPpsToIdr)) = pANBParams->bEnable;

EXIT:
    return ret;
}

OMX_COLOR_FORMATTYPE Exynos_OSAL_Hal2OMXPixelFormat(
    unsigned int hal_format)
{
    OMX_COLOR_FORMATTYPE omx_format;
    switch (hal_format) {
    case HAL_PIXEL_FORMAT_YCbCr_422_I:
        omx_format = OMX_COLOR_FormatYCbYCr;
        break;
    case HAL_PIXEL_FORMAT_YV12:
        omx_format = OMX_COLOR_FormatYUV420Planar;
        break;
    case HAL_PIXEL_FORMAT_YCRCB_420_SP:
        omx_format = OMX_COLOR_FormatYUV420SemiPlanar;
        break;
    case HAL_PIXEL_FORMAT_EXYNOS_YCbCr_420_SP_M_TILED:
        omx_format = (OMX_COLOR_FORMATTYPE)OMX_SEC_COLOR_FormatNV12Tiled;
        break;
    case HAL_PIXEL_FORMAT_BGRA_8888:
    case HAL_PIXEL_FORMAT_EXYNOS_ARGB_8888:
        omx_format = OMX_COLOR_Format32bitARGB8888;
        break;
    default:
        omx_format = OMX_COLOR_FormatYUV420Planar;
        break;
    }
    return omx_format;
}

unsigned int Exynos_OSAL_OMX2HalPixelFormat(
    OMX_COLOR_FORMATTYPE omx_format)
{
    unsigned int hal_format;
    switch ((OMX_U32)omx_format) {
    case OMX_COLOR_FormatYCbYCr:
        hal_format = HAL_PIXEL_FORMAT_YCbCr_422_I;
        break;
    case OMX_COLOR_FormatYUV420Planar:
        hal_format = HAL_PIXEL_FORMAT_YV12;
        break;
    case OMX_COLOR_FormatYUV420SemiPlanar:
        hal_format = HAL_PIXEL_FORMAT_YCRCB_420_SP;
        break;
    case OMX_SEC_COLOR_FormatNV12TPhysicalAddress:
    case OMX_SEC_COLOR_FormatNV12Tiled:
        hal_format = HAL_PIXEL_FORMAT_EXYNOS_YCbCr_420_SP_M_TILED;
        break;
    case OMX_COLOR_Format32bitARGB8888:
        hal_format = HAL_PIXEL_FORMAT_EXYNOS_ARGB_8888;
        break;
    default:
        hal_format = HAL_PIXEL_FORMAT_YV12;
        break;
    }
    return hal_format;
}


#ifdef __cplusplus
}
#endif
