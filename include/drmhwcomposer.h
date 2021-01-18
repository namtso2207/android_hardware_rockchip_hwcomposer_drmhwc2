/*
 * Copyright (C) 2015 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ANDROID_DRM_HWCOMPOSER_H_
#define ANDROID_DRM_HWCOMPOSER_H_

#include <stdbool.h>
#include <stdint.h>

#include <vector>
#include <utils/String8.h>

#include <hardware/hardware.h>
#include <hardware/hwcomposer2.h>

#include "autofd.h"
#include "drmhwcgralloc.h"
#include "rockchip/drmtype.h"

struct hwc_import_context;

int hwc_import_init(struct hwc_import_context **ctx);
int hwc_import_destroy(struct hwc_import_context *ctx);

int hwc_import_bo_create(int fd, struct hwc_import_context *ctx,
                         buffer_handle_t buf, struct hwc_drm_bo *bo);
bool hwc_import_bo_release(int fd, struct hwc_import_context *ctx,
                           struct hwc_drm_bo *bo);

namespace android {

#define UN_USED(arg)     (arg=arg)

int hwc_get_int_property(const char* pcProperty,const char* default_value);
bool hwc_get_bool_property(const char* pcProperty,const char* default_value);
int hwc_get_string_property(const char* pcProperty,const char* default_value,char* retult);


class Importer;

class DrmHwcBuffer {
 public:
  DrmHwcBuffer() = default;
  DrmHwcBuffer(const hwc_drm_bo &bo, Importer *importer)
      : bo_(bo), importer_(importer) {
  }
  DrmHwcBuffer(DrmHwcBuffer &&rhs) : bo_(rhs.bo_), importer_(rhs.importer_) {
    rhs.importer_ = NULL;
  }

  ~DrmHwcBuffer() {
    Clear();
  }

  DrmHwcBuffer &operator=(DrmHwcBuffer &&rhs) {
    Clear();
    importer_ = rhs.importer_;
    rhs.importer_ = NULL;
    bo_ = rhs.bo_;
    return *this;
  }

  operator bool() const {
    return importer_ != NULL;
  }

  const hwc_drm_bo *operator->() const;

  void Clear();

  int ImportBuffer(buffer_handle_t handle, Importer *importer);

 private:
  hwc_drm_bo bo_;
  Importer *importer_ = NULL;
};

class DrmHwcNativeHandle {
 public:
  DrmHwcNativeHandle() = default;

  DrmHwcNativeHandle(native_handle_t *handle) : handle_(handle) {
  }

  DrmHwcNativeHandle(DrmHwcNativeHandle &&rhs) {
    handle_ = rhs.handle_;
    rhs.handle_ = NULL;
  }

  ~DrmHwcNativeHandle();

  DrmHwcNativeHandle &operator=(DrmHwcNativeHandle &&rhs) {
    Clear();
    handle_ = rhs.handle_;
    rhs.handle_ = NULL;
    return *this;
  }

  int CopyBufferHandle(buffer_handle_t handle, int width, int height,
                       int layerCount, int format, int usage, int stride);

  void Clear();

  buffer_handle_t get() const {
    return handle_;
  }

 private:
  native_handle_t *handle_ = NULL;
};

//Drm driver version is 2.0.0 use these.
enum DrmHwcTransform {
    kIdentity = 0,
    kRotate0 = 1 << 0,
    kRotate90 = 1 << 1,
    kRotate180 = 1 << 2,
    kRotate270 = 1 << 3,
    kFlipH = 1 << 4,
    kFlipV = 1 << 5,
};


enum class DrmHwcBlending : int32_t {
  kNone = HWC_BLENDING_NONE,
  kPreMult = HWC_BLENDING_PREMULT,
  kCoverage = HWC_BLENDING_COVERAGE,
};

struct DrmHwcLayer {
  buffer_handle_t sf_handle = NULL;
  int gralloc_buffer_usage = 0;
  DrmHwcBuffer buffer;
  DrmHwcNativeHandle handle;
  uint32_t transform;
  DrmHwcBlending blending = DrmHwcBlending::kNone;
  HWC2::Composition sf_composition;
  uint16_t alpha = 0xff;
  hwc_frect_t source_crop;
  hwc_rect_t display_frame;

  UniqueFd acquire_fence;
  OutputFd release_fence;

  // Frame info
  uint32_t uId_;
  uint32_t uFrameNo_;
  int  iZpos_;
  int  iDrmZpos_;
  bool bFbTarget_=false;
  bool bAfbcd_=false;
  bool bYuv_;
  bool bScale_;
  bool bHdr_;
  bool bSkipLayer_;
  float fHScaleMul_;
  float fVScaleMul_;

  // Buffer info
  int iFd_;
  int iFormat_;
  int iWidth_;
  int iHeight_;
  int iStride_;
  int iUsage;
  uint32_t uFourccFormat_;
  uint64_t uModifier_;
  std::string sLayerName_;

  bool bMatch_;
  bool bUse_;
  bool bMix_;

  bool bGlesCompose_=false;

  int iBestPlaneType=0;

  int iGroupId_;
  int iShareId_;
  int iSkipLine_;

  android_dataspace_t eDataSpace_;
  v4l2_colorspace uColorSpace = V4L2_COLORSPACE_DEFAULT;
  uint16_t uEOTF=0;


  int ImportBuffer(Importer *importer);
  int Init();
  int InitFromDrmHwcLayer(DrmHwcLayer *layer, Importer *importer);
  void SetTransform(int32_t sf_transform);
  void SetSourceCrop(hwc_frect_t const &crop);
  void SetDisplayFrame(hwc_rect_t const &frame);

  buffer_handle_t get_usable_handle() const {
    return handle.get() != NULL ? handle.get() : sf_handle;
  }

  bool protected_usage() const {
    return (gralloc_buffer_usage & GRALLOC_USAGE_PROTECTED) ==
           GRALLOC_USAGE_PROTECTED;
  }
  bool IsYuvFormat(int format);
  bool IsScale(hwc_frect_t &source_crop, hwc_rect_t &display_frame, int transform);
  bool IsAfbcModifier(uint64_t modifier);
  bool IsSkipLayer();
  bool IsGlesCompose();

  bool IsHdr(int usage);
  int GetSkipLine();
  v4l2_colorspace GetColorSpace(android_dataspace_t dataspace);
  supported_eotf_type GetEOTF(android_dataspace_t dataspace);
  std::string TransformToString(uint32_t transform) const;
  std::string BlendingToString(DrmHwcBlending blending) const;
  int DumpInfo(String8 &out);

};

struct DrmHwcDisplayContents {
  OutputFd retire_fence;
  std::vector<DrmHwcLayer> layers;
};
}  // namespace android

#endif
