LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
    worker.cpp

LOCAL_C_INCLUDES := \
    hardware/rockchip/hwcomposer/drmhwc2/include

LOCAL_CPPFLAGS := \
    -Wall \
    -Werror

LOCAL_MODULE := libdrmhwcutils

include $(BUILD_STATIC_LIBRARY)
