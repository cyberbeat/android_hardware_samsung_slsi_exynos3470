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

#ifndef EXYNOS_FIMC_H
#define EXYNOS_FIMC_H

/*
 * exynos_fimc.h — API of the vendor libexynosfimc.so (FIMC M2M scaler).
 *
 * Declarations for the exported libexynosfimc.so entry points as used by
 * libcsc (CSC_METHOD_HW).  We call them directly via dlopen/dlsym to use
 * the library's power-management path, which handles FIMC M2M correctly
 * even while FIMC-IS is active (unlike the direct V4L2 path where
 * pm_runtime_get_sync fails).
 *
 * planes: void* array [CSC_MAX_PLANES=3]; for DMABUF the pointers
 *         contain the fd values (cast to void*).
 * mem_type: FIMC_MEMORY_DMABUF = 4 (matches CSC_MEMORY_DMABUF in csc.h)
 * color_format: V4L2 fourcc
 * FIMC_M2M_MODE: 0 (analogous to GSC_M2M_MODE = 0 from exynos_gscaler.h)
 */

#ifdef __cplusplus
extern "C" {
#endif

/* FIMC M2M mode — analogous to GSC_M2M_MODE = 0 from exynos_gscaler.h */
#define FIMC_M2M_MODE 0

/* Image alignment required by the FIMC */
#define FIMC_IMG_ALIGN_WIDTH  16
#define FIMC_IMG_ALIGN_HEIGHT 2

/* Memory types — from csc.h */
#define FIMC_MEMORY_DMABUF 4

/*
 * Create FIMC handle (auto-select node).
 * Return: handle pointer, or NULL on failure.
 */
void *exynos_fimc_create();

/*
 * Create FIMC handle with exclusive node.
 * @param node: FIMC node number (0-3)
 * @param mode: FIMC_M2M_MODE (0) or FIMC_OUTPUT_MODE (1)
 * @param allow_drm: DRM content support (0=disable, 1=enable)
 * @param need_security: secure mode (0=disable, 1=enable)
 * Return: handle pointer, or NULL on failure.
 */
void *exynos_fimc_create_exclusive(int node, int mode, int allow_drm, int need_security);

/*
 * Destroy FIMC handle and release resources.
 * @param handle: FIMC handle from create/create_exclusive
 */
void exynos_fimc_destroy(void *handle);

/*
 * Set source (OUTPUT) format.
 * @param handle: FIMC handle
 * @param width: source image width (ALIGN to FIMC_IMG_ALIGN_WIDTH)
 * @param height: source image height (ALIGN to FIMC_IMG_ALIGN_HEIGHT)
 * @param crop_left: source crop left offset
 * @param crop_top: source crop top offset
 * @param crop_width: source crop width
 * @param crop_height: source crop height
 * @param color_format: V4L2 fourcc (e.g. V4L2_PIX_FMT_NV21M)
 * @param cacheable: cache mode (0=non-cacheable, 1=cacheable)
 * @param drm: DRM mode (0=disable, 1=enable)
 */
void exynos_fimc_set_src_format(void *handle,
                                 int width, int height,
                                 int crop_left, int crop_top,
                                 int crop_width, int crop_height,
                                 int color_format, int cacheable, int drm);

/*
 * Set destination (CAPTURE) format.
 * @param handle: FIMC handle
 * @param width: destination image width
 * @param height: destination image height
 * @param crop_left: destination crop left offset
 * @param crop_top: destination crop top offset
 * @param crop_width: destination crop width
 * @param crop_height: destination crop height
 * @param color_format: V4L2 fourcc (e.g. V4L2_PIX_FMT_NV21)
 * @param cacheable: cache mode (0=non-cacheable, 1=cacheable)
 * @param drm: DRM mode (0=disable, 1=enable)
 * @param rotation: rotation angle (0, 90, 180, 270)
 */
void exynos_fimc_set_dst_format(void *handle,
                                 int width, int height,
                                 int crop_left, int crop_top,
                                 int crop_width, int crop_height,
                                 int color_format, int cacheable, int drm,
                                 int rotation);

/*
 * Set source buffer addresses.
 * @param handle: FIMC handle
 * @param planes: array of plane addresses/fds (void* [CSC_MAX_PLANES])
 *               For DMABUF: fd values cast to void*
 * @param mem_type: FIMC_MEMORY_DMABUF (4) or other memory type
 * @param index: buffer index (-1 for single-buffer mode)
 */
void exynos_fimc_set_src_addr(void *handle, void **planes, int mem_type, int index);

/*
 * Set destination buffer addresses.
 * @param handle: FIMC handle
 * @param planes: array of plane addresses/fds (void* [CSC_MAX_PLANES])
 *               For DMABUF: fd values cast to void*
 * @param mem_type: FIMC_MEMORY_DMABUF (4) or other memory type
 * @param index: buffer index (-1 for single-buffer mode)
 */
void exynos_fimc_set_dst_addr(void *handle, void **planes, int mem_type, int index);

/*
 * Set rotation.
 * @param handle: FIMC handle
 * @param rotation: rotation angle (0, 90, 180, 270)
 * @param flip_horizontal: horizontal flip (0=disable, 1=enable)
 * @param flip_vertical: vertical flip (0=disable, 1=enable)
 */
void exynos_fimc_set_rotation(void *handle, int rotation,
                              int flip_horizontal, int flip_vertical);

/*
 * Set CSC (Color Space Conversion) properties.
 * @param handle: FIMC handle
 * @param mode: CSC mode
 * @param range: color range
 * @param colorspace: colorspace
 */
void exynos_fimc_set_csc_property(void *handle, int mode, int range, int colorspace);

/*
 * Run FIMC conversion (synchronous — blocks until done).
 * @param handle: FIMC handle
 * Return: 0 on success, non-zero on failure.
 */
int exynos_fimc_convert(void *handle);

/*
 * Stop exclusive FIMC session.
 * @param handle: FIMC handle
 */
void exynos_fimc_stop_exclusive(void *handle);

/*
 * Run exclusive FIMC conversion.
 * @param handle: FIMC handle
 * Return: 0 on success, non-zero on failure.
 */
int exynos_fimc_run_exclusive(void *handle);

/*
 * Configure exclusive FIMC session.
 * @param handle: FIMC handle
 * Return: 0 on success, non-zero on failure.
 */
int exynos_fimc_config_exclusive(void *handle);

#ifdef __cplusplus
}
#endif

#endif /* EXYNOS_FIMC_H */
