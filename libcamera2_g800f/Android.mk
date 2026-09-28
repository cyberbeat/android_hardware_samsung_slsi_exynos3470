# Copyright (C) 2026 The LineageOS Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# Camera2 HAL3 for the Samsung G800F (kminilte, Exynos 3470).
# Native HAL3 implementation that drives the FIMC-IS pipeline directly
# via V4L2.

LOCAL_CLANG_CFLAGS += -Wno-c++11-narrowing -Wno-unused-parameter

LOCAL_C_INCLUDES += \
    hardware/samsung_slsi-cm/exynos3470/include \
    frameworks/native/include \
    frameworks/native/libs/arect/include \
    frameworks/native/libs/nativebase/include \
    frameworks/native/libs/ui/include \
    system/libhidl/transport/token/1.0/utils/include \
    system/media/camera/include

LOCAL_SRC_FILES := \
    G800FCamera2.cpp \
    G800FCamera2Device.cpp \
    G800FExynosCameraNode.cpp \
    G800FExynosCameraBuffer.cpp \
    G800FFimcScaler.cpp \
    G800FFrameQueue.cpp \
    G800FFrameSelector.cpp \
    G800FPipe.cpp \
    G800FPipeEngine.cpp

# The kminilte kernel fimc-is-metadata.h is the only version matching
# the firmware's shot_ext layout (camera2_node_group right after
# setfile, bypass flags at the expected offsets, etc.).
LOCAL_C_INCLUDES += \
    kernel/samsung/kminilte/drivers/media/video/exynos/fimc-is

LOCAL_SHARED_LIBRARIES := \
    libutils \
    libcutils \
    libbinder \
    liblog \
    libcamera_client \
    libhardware \
    libexynosv4l2 \
    libexynosutils \
    libion_exynos \
    libui \
    libcamera_metadata \
    libjpeg \
    libhwjpeg \
    libcsc \
    libdl

LOCAL_C_INCLUDES += external/libjpeg-turbo

LOCAL_HEADER_LIBRARIES := generated_kernel_headers

LOCAL_CFLAGS += -DLOG_NDEBUG=0

LOCAL_MODULE := camera.universal3470
LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_MODULE_TAGS      := optional
LOCAL_PROPRIETARY_MODULE := true

include $(BUILD_SHARED_LIBRARY)
