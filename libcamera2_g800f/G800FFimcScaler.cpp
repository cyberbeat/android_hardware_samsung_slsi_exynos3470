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

/*
 * G800FFimcScaler — FIMC M2M hardware scaler via libcsc (CSC_METHOD_HW).
 *
 * See G800FFimcScaler.h for architecture documentation.
 *
 * This implementation uses the libcsc API (csc_init, csc_set_*,
 * csc_convert_with_rotation).
 *
 * libcsc with -DENABLE_FIMC internally calls exynos_fimc_create_exclusive()
 * from libexynosfimc.so.  This uses the proven power management path
 * of the vendor library, and the V4L2 buffer setup logic
 * (m_exynos_fimc_set_addr) is correctly executed — including the internal
 * plane size calculation that was missing in the direct dlopen/dlsym
 * approach (QBUF failed because the v4l2_buffer structure was not
 * correctly initialized).
 */

#define LOG_TAG "G800FFimcScaler"
#include <cutils/log.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

#include "G800FFimcScaler.h"

/* libcsc API */
#include "csc.h"
/* V4L2↔HAL pixel format conversion */
#include "exynos_format.h"

namespace android {

G800FFimcScaler::G800FFimcScaler()
    : m_cscHandle(NULL), m_node(0), m_configured(false),
      m_srcW(0), m_srcH(0), m_srcFmt(0),
      m_dstW(0), m_dstH(0), m_dstFmt(0) {}

G800FFimcScaler::~G800FFimcScaler()
{
    close();
}

status_t G800FFimcScaler::open(int node)
{
    if (m_cscHandle) {
        ALOGE("%s: already open", __FUNCTION__);
        return BAD_VALUE;
    }
    if (node < 0 || node > 3) {
        ALOGE("%s: invalid node %d (0-3)", __FUNCTION__, node);
        return BAD_VALUE;
    }
    m_node = node;

    /* 1. csc_init(CSC_METHOD_HW).
     * CSC_METHOD_HW = 1 → libcsc uses FIMC (with ENABLE_FIMC) or GSC. */
    m_cscHandle = csc_init(CSC_METHOD_HW);
    if (!m_cscHandle) {
        ALOGE("%s: csc_init(CSC_METHOD_HW) failed", __FUNCTION__);
        return NO_INIT;
    }

    /* 2. csc_set_hw_property — fixed_node = FIMC node 0-3. */
    CSC_ERRORCODE ret = csc_set_hw_property(m_cscHandle,
                                            CSC_HW_PROPERTY_FIXED_NODE,
                                            m_node);
    if (ret != CSC_ErrorNone) {
        ALOGE("%s: csc_set_hw_property(FIXED_NODE, %d) failed: %d",
              __FUNCTION__, m_node, ret);
        csc_deinit(m_cscHandle);
        m_cscHandle = NULL;
        return NO_INIT;
    }

    ALOGI("%s: csc_init(CSC_METHOD_HW) ok, node=%d, handle=%p",
          __FUNCTION__, m_node, m_cscHandle);
    return NO_ERROR;
}

status_t G800FFimcScaler::configure(int srcW, int srcH, unsigned int srcFmt,
                                     int dstW, int dstH, unsigned int dstFmt)
{
    if (!m_cscHandle) {
        ALOGE("%s: not open", __FUNCTION__);
        return NO_INIT;
    }

    /* Align source width/height to 16-byte boundary — FIMC expects
     * 16-byte-aligned buffers: (w + 0xf) & ~0xf. */
    unsigned int srcWidthAligned  = (unsigned int)((srcW + 0xf) & ~0xf);
    unsigned int srcHeightAligned = (unsigned int)((srcH + 0xf) & ~0xf);

    /* Convert V4L2 → HAL pixel format (standard pattern).
     * csc_set_src_format expects HAL_PIXEL_FORMAT, not V4L2. */
    unsigned int srcHalFmt = (unsigned int)V4L2_PIX_2_HAL_PIXEL_FORMAT((int)srcFmt);
    unsigned int dstHalFmt = (unsigned int)V4L2_PIX_2_HAL_PIXEL_FORMAT((int)dstFmt);

    /* Aspect ratio adjustment: If source and destination have different
     * aspect ratios (e.g. front camera 1920x1080 = 16:9, but
     * preview 960x720 = 4:3), the source is center-cropped to the
     * target aspect ratio. Without this, the image would be stretched.
     * Rear camera: 1440x1080 (4:3) → 960x720 (4:3) → no crop.
     * Front camera: 1920x1080 (16:9) → 960x720 (4:3) → crop to 1440x1080. */
    unsigned int cropW = (unsigned int)srcW;
    unsigned int cropH = (unsigned int)srcH;
    unsigned int cropLeft = 0;
    unsigned int cropTop = 0;
    if (srcW > 0 && srcH > 0 && dstW > 0 && dstH > 0) {
        unsigned int srcRatio = (unsigned int)(srcW * 1000 / srcH);
        unsigned int dstRatio = (unsigned int)(dstW * 1000 / dstH);
        if (srcRatio != dstRatio) {
            if (srcRatio > dstRatio) {
                /* Source is wider → crop horizontally */
                cropW = (unsigned int)((long)srcH * dstW / dstH);
                cropLeft = (unsigned int)((srcW - cropW) / 2);
            } else {
                /* Source is narrower → crop vertically */
                cropH = (unsigned int)((long)srcW * dstH / dstW);
                cropTop = (unsigned int)((srcH - cropH) / 2);
            }
            ALOGI("%s: aspect ratio crop: %dx%d → crop %dx%d at (%d,%d) for dst %dx%d",
                  __FUNCTION__, srcW, srcH, cropW, cropH, cropLeft, cropTop, dstW, dstH);
        }
    }

    ALOGI("%s: src=%dx%d v4l2=0x%x hal=0x%x (aligned %dx%d, crop %dx%d at %d,%d) → dst=%dx%d v4l2=0x%x hal=0x%x",
          __FUNCTION__, srcW, srcH, srcFmt, srcHalFmt, srcWidthAligned, srcHeightAligned,
          cropW, cropH, cropLeft, cropTop, dstW, dstH, dstFmt, dstHalFmt);

    /* 3. csc_set_src_format — width/height aligned, crop = aspect-ratio adjusted.
     * csc_set_src_format(handle, alignedW, alignedH,
     *                          crop_left, crop_top, crop_w, crop_h,
     *                          halFmt, cacheable=0) */
    CSC_ERRORCODE ret = csc_set_src_format(m_cscHandle,
                                           srcWidthAligned, srcHeightAligned,
                                           cropLeft, cropTop,
                                           cropW, cropH,
                                           srcHalFmt,
                                           0);              /* cacheable */
    if (ret != CSC_ErrorNone) {
        ALOGE("%s: csc_set_src_format failed: %d", __FUNCTION__, ret);
        return NO_INIT;
    }

    /* 4. csc_set_dst_format — width/height NOT aligned (standard pattern).
     * crop = full image (crop_w = dstW, crop_h = dstH). */
    ret = csc_set_dst_format(m_cscHandle,
                             (unsigned int)dstW, (unsigned int)dstH,
                             0, 0,
                             (unsigned int)dstW, (unsigned int)dstH,
                             dstHalFmt,
                             0);
    if (ret != CSC_ErrorNone) {
        ALOGE("%s: csc_set_dst_format failed: %d", __FUNCTION__, ret);
        return NO_INIT;
    }

    m_srcW = srcW; m_srcH = srcH; m_srcFmt = srcFmt;
    m_dstW = dstW; m_dstH = dstH; m_dstFmt = dstFmt;
    m_configured = true;

    ALOGI("%s: configured ok (node=%d, handle=%p)",
          __FUNCTION__, m_node, m_cscHandle);
    return NO_ERROR;
}

status_t G800FFimcScaler::processFrame(int srcFdY, int srcFdVU, int dstFd)
{
    if (!m_cscHandle || !m_configured) {
        ALOGE("%s: not configured", __FUNCTION__);
        return NO_INIT;
    }
    if (srcFdY < 0 || dstFd < 0) {
        ALOGE("%s: invalid fd (srcY=%d dst=%d)", __FUNCTION__, srcFdY, dstFd);
        return BAD_VALUE;
    }

    //ALOGV("%s: node=%d srcY=%d srcVU=%d dst=%d (start)",
    //      __FUNCTION__, m_node, srcFdY, srcFdVU, dstFd);

    /* 5. csc_set_src_buffer — DMABUF fds as void*[] (standard pattern).
     * srcBuf[] = { fd[0], fd[1], fd[2] }, mem_type = CSC_MEMORY_DMABUF.
     * IMPORTANT: FDs are cast to void* (not dereferenced as pointers). */
    void *srcBuf[CSC_MAX_PLANES] = { NULL, NULL, NULL };
    srcBuf[0] = (void *)(intptr_t)srcFdY;
    srcBuf[1] = (srcFdVU >= 0) ? (void *)(intptr_t)srcFdVU : NULL;

    CSC_ERRORCODE ret = csc_set_src_buffer(m_cscHandle, srcBuf, CSC_MEMORY_DMABUF);
    if (ret != CSC_ErrorNone) {
        ALOGE("%s: csc_set_src_buffer failed: %d (srcY=%d srcVU=%d)",
              __FUNCTION__, ret, srcFdY, srcFdVU);
        return UNKNOWN_ERROR;
    }

    /* 6. csc_set_dst_buffer — gralloc NV21 single-plane: only fd[0]. */
    void *dstBuf[CSC_MAX_PLANES] = { NULL, NULL, NULL };
    dstBuf[0] = (void *)(intptr_t)dstFd;

    ret = csc_set_dst_buffer(m_cscHandle, dstBuf, CSC_MEMORY_DMABUF);
    if (ret != CSC_ErrorNone) {
        ALOGE("%s: csc_set_dst_buffer failed: %d (dst=%d)",
              __FUNCTION__, ret, dstFd);
        return UNKNOWN_ERROR;
    }

    //ALOGV("%s: node=%d buffers set, calling csc_convert_with_rotation",
    //      __FUNCTION__, m_node);

    /* 7. csc_convert_with_rotation — rotation=0, flipH=0, flipV=0.
     * csc_convert_with_rotation(handle, 0, flipH, flipV) is called. */
    ret = csc_convert_with_rotation(m_cscHandle, 0, 0, 0);
    if (ret != CSC_ErrorNone) {
        ALOGE("%s: csc_convert_with_rotation failed: %d (srcY=%d srcVU=%d dst=%d)",
              __FUNCTION__, ret, srcFdY, srcFdVU, dstFd);
        return UNKNOWN_ERROR;
    }

    //ALOGV("%s: node=%d done ok", __FUNCTION__, m_node);

    return NO_ERROR;
}

void G800FFimcScaler::close()
{
    if (m_cscHandle) {
        csc_deinit(m_cscHandle);
        m_cscHandle = NULL;
    }
    m_configured = false;
}

} // namespace android
