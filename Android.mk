#
#
# Copyright (C) 2009 The Android Open Source Project
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
#

ifeq ($(TARGET_BOARD_PLATFORM),exynos3)

exynos3_dirs := \
        libion_exynos \
        libmemtrack \
        libsecurepath \
        libstagefrighthw \
        libswconverter \
        libexynosutils \
        libv4l2 \
        gralloc \
        libhwjpeg \
        libcsc \
        libexynosfimc_patch \
        libcamera2_g800f \
        exynos_omx
#        libcamera2
##        mobicore \
##        libgscaler \
##        libhwc \
##        libcsc
# libgscaler auskommentiert: Exynos 3470 hat kein GSC, und libMcClient fehlt.
# FIMC M2M (über libcsc mit ENABLE_FIMC) wird stattdessen verwendet.

BOARD_USE_V4L2 := true
BOARD_USE_V4L2_ION := true

BOARD_USE_SAMSUNG_COLORFORMAT := true
BOARD_FIX_NATIVE_COLOR_FORMAT := true
BOARD_NONBLOCK_MODE_PROCESS := true
BOARD_USE_STOREMETADATA := true
BOARD_USE_METADATABUFFERTYPE := true
BOARD_USES_MFC_FPS := true
BOARD_USE_S3D_SUPPORT := true
BOARD_USE_EXYNOS_OMX := true

# HWC
USE_HWC_CSC_THREAD := true


include $(call all-named-subdir-makefiles,$(exynos3_dirs))

endif
