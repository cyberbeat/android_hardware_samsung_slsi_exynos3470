# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


LOCAL_PATH := $(call my-dir)

# HAL module implemenation stored in
# hw/<OVERLAY_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)

LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_SHARED_LIBRARIES := liblog libcutils libion libutils

LOCAL_C_INCLUDES := \
        $(LOCAL_PATH)/../include \
        $(TOP)/hardware/samsung_slsi-cm/exynos3470/include

LOCAL_SRC_FILES := 	\
	gralloc.cpp 	\
	mapper.cpp	\
	framebuffer.cpp

LOCAL_HEADER_LIBRARIES := generated_kernel_headers

LOCAL_MODULE := gralloc.exynos3
LOCAL_CFLAGS := -DLOG_TAG=\"gralloc\"
LOCAL_CFLAGS += -DLOG_NDEBUG=0
LOCAL_CFLAGS += -Wno-unused-parameter
LOCAL_CFLAGS += -Wno-unused-variable
# Use GNU libstdc++ instead of LLVM libc++ to match the original Samsung blob ABI
LOCAL_CFLAGS += -D__STDC_LIMIT_MACROS
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := samsung_arm
# Install to /system/lib/hw/ (not /system/vendor/lib/hw/) to override
# the proprietary blob that kminilte-vendor-blobs.mk copies there.
LOCAL_PROPRIETARY_MODULE := false

include $(BUILD_SHARED_LIBRARY)
