LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_CLANG_CFLAGS += -Wno-int-conversion

LOCAL_MODULE_TAGS := optional

LOCAL_SRC_FILES := \
	Exynos_OMX_H264dec.c \
	library_register.c

LOCAL_MODULE := libOMX.Exynos.AVC.Decoder
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/omx

LOCAL_CFLAGS := -Wno-unused-variable -Wno-unused-label -Wno-unused-parameter -Wno-implicit-function-declaration -Wno-sign-compare -Wno-enum-conversion -Wno-switch -Wno-incompatible-pointer-types -DUSE_LOCAL_SEC_NV12TILED

ifeq ($(BOARD_NONBLOCK_MODE_PROCESS), true)
LOCAL_CFLAGS += -DNONBLOCK_MODE_PROCESS
endif

ifeq ($(BOARD_USE_ANB), true)
LOCAL_CFLAGS += -DUSE_ANB
endif

ifeq ($(BOARD_USE_S3D_SUPPORT), true)
LOCAL_CFLAGS += -DS3D_SUPPORT
endif

LOCAL_ARM_MODE := arm

LOCAL_STATIC_LIBRARIES := libExynosOMX_Vdec libExynosOMX_OSAL libExynosOMX_Basecomponent \
	libswconverter libExynosVideoApi
LOCAL_SHARED_LIBRARIES := libc libdl libcutils libutils liblog libui \
	libExynosOMX_Resourcemanager libcsc libexynosv4l2 libion_exynos \
	libhardware

ifeq ($(BOARD_USES_MFC_FPS),true)
LOCAL_CFLAGS += -DCONFIG_MFC_FPS
endif

LOCAL_C_INCLUDES := \
	$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include \
	$(EXYNOS_OMX_INC)/khronos \
	$(EXYNOS_OMX_INC)/exynos \
	$(EXYNOS_OMX_TOP)/osal \
	$(EXYNOS_OMX_TOP)/core \
	$(EXYNOS_OMX_COMPONENT)/common \
	$(EXYNOS_OMX_COMPONENT)/video/dec \
	hardware/samsung_slsi/exynos3470/include \
	hardware/samsung_slsi/exynos3470/libcsc \
	hardware/samsung_slsi/exynos3470/exynos_omx/codecs/exynos_codecs/video/exynos3/mfc_v4l2/include

LOCAL_ADDITIONAL_DEPENDENCIES := \
	$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

include $(BUILD_SHARED_LIBRARY)
