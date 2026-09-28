# libexynosfimc — Samsung proprietary FIMC M2M library, binary-patched.
#
# The original Samsung libexynosfimc.so (from vendor/samsung/kminilte)
# has a bug: exynos_fimc_convert() closes two internal fd fields (at
# object offsets 0x50 and 0x288) after each conversion. These fields
# are zero-initialized via memset, so the binary calls close(0) —
# closing fd 0. In the camera.provider process, fd 0 is often a V4L2
# device (e.g. FLITE), so close(0) invalidates the camera pipeline.
#
# Patch: in exynos_fimc_convert() (file offset 0x385c, Thumb mode), two
#   `blt.n` (branch-if-less-than) instructions guard the close() calls:
#     0x38c8: blt.n 0x38d4  (guard for close at 0x38ca)
#     0x38da: blt.n 0x38e8  (guard for close at 0x38dc)
#   Changing `blt.n` to `b.n` (unconditional branch) skips the close()
#   calls entirely. Only 4 bytes changed, binary size unchanged.
#
# The original binary stays in the vendor tree; this module copies it,
# applies the patch, and installs the patched version as libexynosfimc.so.
# The vendor tree Android.mk for libexynosfimc is disabled to avoid
# a duplicate module definition.

LOCAL_PATH := $(call my-dir)

# Original binary from vendor tree (absolute path via TOP)
FIMC_ORIG := $(TOP)/vendor/samsung/kminilte/proprietary/lib/libexynosfimc.so

# Patched copy in a separate intermediates directory (avoid dependency cycle)
FIMC_PATCH_DIR := $(call intermediates-dir-for,EXECUTABLES,libexynosfimc_patch,,)
FIMC_PATCHED := $(FIMC_PATCH_DIR)/libexynosfimc.so

# Rule: copy original, then apply 4-byte patch with dd
# Patch offsets are file offsets (0x38c8 = 14536, 0x38da = 14554)
$(FIMC_PATCHED): $(FIMC_ORIG)
	@echo "Patching libexynosfimc.so (fd0-close bug fix)"
	$(hide) mkdir -p $(dir $@)
	$(hide) cp $(FIMC_ORIG) $@
	$(hide) printf '\x04\xe0' | dd of=$@ bs=1 seek=14536 conv=notrunc 2>/dev/null
	$(hide) printf '\x05\xe0' | dd of=$@ bs=1 seek=14554 conv=notrunc 2>/dev/null

include $(CLEAR_VARS)
LOCAL_PATH := $(FIMC_PATCH_DIR)
LOCAL_MODULE := libexynosfimc
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_SUFFIX := .so
LOCAL_MODULE_TAGS := optional
LOCAL_STRIP_MODULE := false
LOCAL_SRC_FILES := libexynosfimc.so
include $(BUILD_PREBUILT)
