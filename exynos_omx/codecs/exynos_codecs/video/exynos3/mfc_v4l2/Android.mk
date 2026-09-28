LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_CLANG_CFLAGS += \
	-Wno-int-conversion \
	-Wno-incompatible-pointer-types

LOCAL_CFLAGS += -Wno-unused-variable -Wno-format -Wno-sign-compare -Wno-unused-function

LOCAL_SRC_FILES := \
	enc/src/ExynosVideoEncoder.c \
	dec/src/ExynosVideoDecoder.c

LOCAL_C_INCLUDES := \
	$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include \
	$(LOCAL_PATH)/include \
	hardware/samsung_slsi/exynos3470/include \
	hardware/samsung_slsi/exynos3470/exynos_omx/openmax/exynos_omx/include/khronos

LOCAL_ADDITIONAL_DEPENDENCIES := \
	$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_MODULE := libExynosVideoApi
LOCAL_MODULE_TAGS := optional
LOCAL_ARM_MODE := arm

include $(BUILD_STATIC_LIBRARY)
