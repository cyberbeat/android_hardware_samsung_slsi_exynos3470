#ifndef EXYNOS_FIMC_H
#define EXYNOS_FIMC_H

/*
 * exynos_fimc.h — Rekonstruierte API für vendor/samsung/kminilte/proprietary/lib/libexynosfimc.so
 *
 * Der Source-Code für libexynosfimc.so ist nicht verfügbar.  Diese Header-Datei
 * wurde rekonstruiert aus:
 *   1. Binary-Analyse der exportierten Symbole (readelf -sW)
 *   2. Verwendungs-Stellen in hardware/samsung_slsi-cm/exynos3470/libcsc/csc.c
 *      (dort #ifdef ENABLE_FIMC ausgeklammert, aber vollständiger Code vorhanden)
 *   3. ARM-Calling-Convention (r0=handle, r1..r3=args, stack für weitere)
 *
 * Die proprietäre HAL1 nutzt diese API über libcsc (CSC_METHOD_HW) für die
 * GSC_VIDEO-Pipe (Video-Scaling).  Wir verwenden sie direkt via dlopen/dlsym
 * um den Power-Management-Pfad von libexynosfimc.so zu nutzen, der FIMC M2M
 * auch während FIMC-IS aktiv ist korrekt handhabt (im Gegensatz zum direkten
 * V4L2-Pfad wo pm_runtime_get_sync fehlschlägt).
 *
 * Verifizierte Export-Symbole (readelf -sW libexynosfimc.so):
 *   exynos_fimc_create             (252 bytes)
 *   exynos_fimc_create_exclusive   (388 bytes)
 *   exynos_fimc_destroy            (64 bytes)
 *   exynos_fimc_set_src_format     (88 bytes)
 *   exynos_fimc_set_dst_format     (120 bytes)
 *   exynos_fimc_set_src_addr       (not exported — internal? use set_addr)
 *   exynos_fimc_set_dst_addr       (76 bytes)
 *   exynos_fimc_set_rotation       (140 bytes)
 *   exynos_fimc_set_csc_property   (64 bytes)
 *   exynos_fimc_convert            (240 bytes)
 *   exynos_fimc_stop_exclusive     (56 bytes)
 *   exynos_fimc_run_exclusive      (64 bytes)
 *   exynos_fimc_config_exclusive   (64 bytes)
 *
 * Rekonstruiert aus csc.c Verwendungs-Stellen:
 *   csc.c:525: exynos_fimc_create_exclusive(node, FIMC_M2M_MODE, 0, 0)
 *   csc.c:527: exynos_fimc_create()
 *   csc.c:581: exynos_fimc_set_src_format(handle, width, height,
 *             crop_left, crop_top, crop_width, crop_height,
 *             color_format, cacheable, drm)
 *   csc.c:593: exynos_fimc_set_dst_format(handle, width, height,
 *             crop_left, crop_top, crop_width, crop_height,
 *             color_format, cacheable, drm, rotation)
 *   csc.c:693: exynos_fimc_set_src_addr(handle, planes, mem_type, -1)
 *   csc.c:694: exynos_fimc_set_dst_addr(handle, planes, mem_type, -1)
 *   csc.c:475: exynos_fimc_convert(handle)
 *   csc.c:749: exynos_fimc_destroy(handle)
 *
 * planes: void* array [CSC_MAX_PLANES=3], bei DMABUF enthalten die Pointer
 *         die fd-Werte (als void* gecastet).
 * mem_type: CSC_MEMORY_DMABUF = 4 (aus csc.h)
 * color_format: V4L2 fourcc (HAL_PIXEL_FORMAT_2_V4L2_PIX macro)
 * FIMC_M2M_MODE: 0 (analog zu GSC_M2M_MODE = 0 aus exynos_gscaler.h)
 */

#ifdef __cplusplus
extern "C" {
#endif

/* FIMC M2M mode — analog zu GSC_M2M_MODE = 0 aus exynos_gscaler.h */
#define FIMC_M2M_MODE 0

/* Image alignment — aus csc.c Zeile 55-56 */
#define FIMC_IMG_ALIGN_WIDTH  16
#define FIMC_IMG_ALIGN_HEIGHT 2

/* Memory types — aus csc.h */
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
 * Return: 0 on success, non-zero on failure.
 */
int exynos_fimc_stop_exclusive(void *handle);

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

/*
 * Wait for frame completion in exclusive mode.
 * @param handle: FIMC handle
 * Return: 0 on success, non-zero on failure.
 */
int exynos_fimc_wait_frame_done_exclusive(void *handle);

#ifdef __cplusplus
}
#endif

#endif /* EXYNOS_FIMC_H */
