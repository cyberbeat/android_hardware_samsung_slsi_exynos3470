LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_CLANG_CFLAGS += -Wno-int-conversion

LOCAL_SRC_FILES := \
	Exynos_OMX_VencControl.c \
	Exynos_OMX_Venc.c

LOCAL_MODULE := libExynosOMX_Venc
LOCAL_ARM_MODE := arm
LOCAL_MODULE_TAGS := optional

LOCAL_CFLAGS := -DUSE_CSC_GSCALER -DUSE_LOCAL_SEC_NV12TILED
LOCAL_CFLAGS += -include $(EXYNOS_OMX_INC)/exynos/Exynos_OMX_Compat.h

LOCAL_C_INCLUDES := \
	$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include \
	$(EXYNOS_OMX_INC)/khronos \
	$(EXYNOS_OMX_INC)/exynos \
	$(EXYNOS_OMX_TOP)/osal \
	$(EXYNOS_OMX_TOP)/core \
	$(EXYNOS_OMX_COMPONENT)/common \
	$(EXYNOS_OMX_COMPONENT)/video/enc \
	hardware/samsung_slsi/exynos3470/include \
	hardware/samsung_slsi/exynos3470/libcsc \
	hardware/samsung_slsi/exynos3470/exynos_omx/codecs/exynos_codecs/video/exynos3/mfc_v4l2/include \
	frameworks/native/libs/nativewindow/include \
	frameworks/native/libs/nativebase/include \
	frameworks/native/libs/arect/include

LOCAL_ADDITIONAL_DEPENDENCIES := \
	$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

ifeq ($(BOARD_USE_METADATABUFFERTYPE), true)
LOCAL_CFLAGS += -DUSE_METADATABUFFERTYPE
endif

ifeq ($(BOARD_USE_STOREMETADATA), true)
LOCAL_CFLAGS += -DUSE_STOREMETADATA
endif

LOCAL_SHARED_LIBRARIES := libcsc

include $(BUILD_STATIC_LIBRARY)
