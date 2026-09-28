/*
 * Compatibility definitions for building legacy Exynos OMX source
 * against Android 9 / LineageOS 16 headers.
 *
 * After the Android 9 port the HAL pixel format names and V4L2 CIDs are
 * resolved directly in the source / via the kernel header copy.
 * OMX_SEC_COLOR_FormatNV12Tiled is provided by Exynos_OMX_Def.h when
 * USE_LOCAL_SEC_NV12TILED is defined (set in all relevant Android.mk).
 *
 * The only remaining fallback is OMX_COLOR_FormatAndroidOpaque, which
 * is only visible in Exynos_OMX_Def.h under USE_KHRONOS_OMX_HEADER (not
 * set for this build) or in frameworks/native OMX_IVCommon.h.
 */
#ifndef EXYNOS_OMX_COMPAT_H
#define EXYNOS_OMX_COMPAT_H

#include <OMX_Index.h>

/* OMX_COLOR_FormatAndroidOpaque — vendor extension enum value from
 * Exynos_OMX_Def.h (only visible when USE_KHRONOS_OMX_HEADER is defined)
 * and from frameworks/native OMX_IVCommon.h.  Provided here as a fallback
 * #define so that translation units which do not have either of those
 * available still resolve the symbol. */
#ifndef OMX_COLOR_FormatAndroidOpaque
#define OMX_COLOR_FormatAndroidOpaque         ((OMX_COLOR_FORMATTYPE)0x7F000789)
#endif

#endif /* EXYNOS_OMX_COMPAT_H */
