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

#ifndef G800F_FIMC_SCALER_H
#define G800F_FIMC_SCALER_H

#include <utils/Errors.h>
#include <linux/videodev2.h>

namespace android {

/*
 * G800FFimcScaler — FIMC M2M hardware scaler via libcsc (CSC_METHOD_HW).
 *
 * The Exynos 3470 has no GSC (G-Scaler) hardware block.  The only
 * independent hardware scalers are the FIMC M2M nodes:
 *   /dev/video23 = fimc.0.m2m  (FIMC node 0)
 *   /dev/video25 = fimc.1.m2m  (FIMC node 1)
 *   /dev/video27 = fimc.2.m2m  (FIMC node 2)
 *   /dev/video29 = fimc.3.m2m  (FIMC node 3)
 *
 * FIMC M2M is used via the libcsc API:
 *   csc_init(CSC_METHOD_HW)
 *   csc_set_hw_property(handle, CSC_HW_PROPERTY_FIXED_NODE, node)
 *   csc_set_src_format / csc_set_dst_format
 *   csc_set_src_buffer / csc_set_dst_buffer  (DMABUF fds as void*)
 *   csc_convert_with_rotation
 *
 * libcsc with ENABLE_FIMC internally calls exynos_fimc_create_exclusive()
 * from libexynosfimc.so.  This has its own power management path that
 * correctly handles FIMC M2M even while FIMC-IS is active.
 *
 * Usage (e.g. preview 1440×1080 → 960×720):
 *   G800FFimcScaler scaler;
 *   scaler.open(0);                   // csc_init + hw_property (node 0)
 *   scaler.configure(1440, 1080, V4L2_PIX_FMT_NV21M,
 *                    960, 720,  V4L2_PIX_FMT_NV21);
 *   // per frame:
 *   scaler.processFrame(srcFdY, srcFdVU, dstFd);
 *   scaler.close();                    // csc_deinit
 *
 * Zero-copy: FIMC reads directly from the SCP dmabuf and writes directly
 * to the gralloc dmabuf.  No CPU access to pixel data, no malloc, no memcpy.
 */

class G800FFimcScaler {
public:
    G800FFimcScaler();
    ~G800FFimcScaler();

    // Initializes libcsc (CSC_METHOD_HW) and sets the fixed_node property.
    // node = FIMC node 0-3 (for CSC_HW_PROPERTY_FIXED_NODE).
    status_t open(int node = 0);

    // Configures source (OUTPUT) and destination (CAPTURE) formats.
    // Must be called before processFrame().
    // srcFmt: V4L2_PIX_FMT_NV21M (2-plane) or V4L2_PIX_FMT_NV21 (1-plane)
    // dstFmt: V4L2_PIX_FMT_NV21 (1-plane, for gralloc)
    status_t configure(int srcW, int srcH, unsigned int srcFmt,
                       int dstW, int dstH, unsigned int dstFmt);

    // Scales a frame synchronously (blocks until FIMC is done).
    // srcFdY/srcFdVU = DMABUF fds of the source Y/VU planes (from SCP V4L2).
    // dstFd = DMABUF fd of the destination buffer (from gralloc, single-plane).
    // For single-plane source (NV21): srcFdVU = -1.
    status_t processFrame(int srcFdY, int srcFdVU, int dstFd);

    // Closes the CSC handle and releases all resources.
    void close();

    bool isOpen() const { return m_cscHandle != NULL; }
    bool isConfigured() const { return m_cscHandle != NULL && m_configured; }

    int getDstWidth() const { return m_dstW; }
    int getDstHeight() const { return m_dstH; }

private:
    void *m_cscHandle;     // csc_init() handle (CSC_HANDLE*)
    int   m_node;          // FIMC node number (0-3)
    bool  m_configured;    // configure() was successfully called

    // Source config
    int           m_srcW, m_srcH;
    unsigned int  m_srcFmt;

    // Destination config
    int           m_dstW, m_dstH;
    unsigned int  m_dstFmt;

    G800FFimcScaler(const G800FFimcScaler&);
    G800FFimcScaler& operator=(const G800FFimcScaler&);
};

} // namespace android

#endif // G800F_FIMC_SCALER_H
