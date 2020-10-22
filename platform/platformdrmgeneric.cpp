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

#define LOG_TAG "hwc-platform-drm-generic"

#include "platformdrmgeneric.h"
#include "drmdevice.h"
#include "platform.h"
#if USE_GRALLOC_4
#else
#include "gralloc_drm_handle.h"
#endif
#include "rockchip/drmgralloc.h"
#include "rockchip/platform/drmvop.h"
#include "rockchip/platform/drmvop2.h"


#include <drm_fourcc.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include <cutils/properties.h>
#include <hardware/gralloc.h>
#include <log/log.h>


#define ALIGN_DOWN( value, base)	(value & (~(base-1)) )

namespace android {

#ifdef USE_DRM_GENERIC_IMPORTER
// static
Importer *Importer::CreateInstance(DrmDevice *drm) {
  DrmGenericImporter *importer = new DrmGenericImporter(drm);
  if (!importer)
    return NULL;

  int ret = importer->Init();
  if (ret) {
    ALOGE("Failed to initialize the nv importer %d", ret);
    delete importer;
    return NULL;
  }
  return importer;
}
#endif

DrmGenericImporter::DrmGenericImporter(DrmDevice *drm)
    : drm_(drm),
    exclude_non_hwfb_(false){
    drmGralloc_ = DrmGralloc::getInstance();
}

DrmGenericImporter::~DrmGenericImporter() {
}

int DrmGenericImporter::Init() {

  char exclude_non_hwfb_prop[PROPERTY_VALUE_MAX];
  property_get("hwc.drm.exclude_non_hwfb_imports", exclude_non_hwfb_prop, "0");
  exclude_non_hwfb_ = static_cast<bool>(strncmp(exclude_non_hwfb_prop, "0", 1));

  return 0;
}

uint32_t DrmGenericImporter::ConvertHalFormatToDrm(uint32_t hal_format) {
  switch (hal_format) {
    case HAL_PIXEL_FORMAT_RGB_888:
      return DRM_FORMAT_BGR888;
    case HAL_PIXEL_FORMAT_BGRA_8888:
      return DRM_FORMAT_ARGB8888;
    case HAL_PIXEL_FORMAT_RGBX_8888:
      return DRM_FORMAT_XBGR8888;
    case HAL_PIXEL_FORMAT_RGBA_8888:
      return DRM_FORMAT_ABGR8888;
    case HAL_PIXEL_FORMAT_RGB_565:
      return DRM_FORMAT_BGR565;
    case HAL_PIXEL_FORMAT_YV12:
      return DRM_FORMAT_YVU420;
    case HAL_PIXEL_FORMAT_YCrCb_NV12:
      return DRM_FORMAT_NV12;
    case HAL_PIXEL_FORMAT_YCrCb_NV12_10:
      return DRM_FORMAT_NV12_10;
    default:
      ALOGE("Cannot convert hal format to drm format %u", hal_format);
      return -EINVAL;
  }
}

uint32_t DrmGenericImporter::DrmFormatToBitsPerPixel(uint32_t drm_format) {
  switch (drm_format) {
    case DRM_FORMAT_ARGB8888:
    case DRM_FORMAT_XBGR8888:
    case DRM_FORMAT_ABGR8888:
      return 32;
    case DRM_FORMAT_BGR888:
      return 24;
    case DRM_FORMAT_BGR565:
      return 16;
    case DRM_FORMAT_YVU420:
      return 12;
    case DRM_FORMAT_NV12:
      return 12;
    case DRM_FORMAT_NV12_10:
      return 15;
    default:
      ALOGE("Cannot convert hal format %u to bpp (returning 32)", drm_format);
      return 32;
  }
}

int DrmGenericImporter::ImportBuffer(buffer_handle_t handle, hwc_drm_bo_t *bo) {

  int fd,width,height,byte_stride,format,usage;
  fd     = drmGralloc_->hwc_get_handle_primefd(handle);
  width  = drmGralloc_->hwc_get_handle_attibute(handle,ATT_WIDTH);
  height = drmGralloc_->hwc_get_handle_attibute(handle,ATT_HEIGHT);
  format = drmGralloc_->hwc_get_handle_attibute(handle,ATT_FORMAT);
  usage  = drmGralloc_->hwc_get_handle_usage(handle);
  byte_stride = drmGralloc_->hwc_get_handle_attibute(handle,ATT_BYTE_STRIDE);

  uint32_t gem_handle;
  int ret = drmPrimeFDToHandle(drm_->fd(), fd, &gem_handle);
  if (ret) {
    ALOGE("failed to import prime fd %d ret=%d", fd, ret);
    return ret;
  }

  memset(bo, 0, sizeof(hwc_drm_bo_t));
  if(format == HAL_PIXEL_FORMAT_YCrCb_NV12_10){
      bo->width = width/1.25;
      bo->width = ALIGN_DOWN(bo->width,2);
  }else{
      bo->width = width;
  }

  bo->height = height;
  bo->hal_format = format;
  bo->format = ConvertHalFormatToDrm(format);
  bo->usage = usage;
  bo->pixel_stride = (byte_stride) /
                     DrmFormatToBitsPerPixel(bo->format);
  bo->pitches[0] = byte_stride;
  bo->gem_handles[0] = gem_handle;
  bo->offsets[0] = 0;

  if(format == HAL_PIXEL_FORMAT_YCrCb_NV12 || format == HAL_PIXEL_FORMAT_YCrCb_NV12_10){
    bo->pitches[1] = bo->pitches[0];
    bo->gem_handles[1] = gem_handle;
    bo->offsets[1] = bo->pitches[1] * bo->height;
  }

  __u64 modifier[4];
  uint64_t internal_format;
  memset(modifier, 0, sizeof(modifier));

  internal_format = drmGralloc_->hwc_get_handle_internal_format(handle);
  if (internal_format & GRALLOC_ARM_INTFMT_AFBC){
      ALOGV("KP : to set DRM_FORMAT_MOD_ARM_AFBC.");
#ifdef ANDROID_R
      modifier[0] = DRM_FORMAT_MOD_ARM_AFBC(1);
#else
      modifier[0] = DRM_FORMAT_MOD_ARM_AFBC;
#endif
  }

  ret = drmModeAddFB2_ext(drm_->fd(), bo->width, bo->height, bo->format,
                      bo->gem_handles, bo->pitches, bo->offsets, modifier,
		                  &bo->fb_id, DRM_MODE_FB_MODIFIERS);


  ALOGD_IF(LogLevel(DBG_DEBUG),"ImportBuffer fd=%d,w=%d,h=%d,format=0x%x,bo->format=0x%x,gem_handle=%d,bo->pitches[0]=%d,fb_id=%d",
      drm_->fd(), bo->width, bo->height, format,bo->format,
      gem_handle, bo->pitches[0], bo->fb_id);

  if (ret) {
    ALOGE("could not create drm fb %d", ret);
    ALOGE("ImportBuffer fail fd=%d,w=%d,h=%d,format=0x%x,bo->format=0x%x,gem_handle=%d,bo->pitches[0]=%d,fb_id=%d",
    drm_->fd(), bo->width, bo->height, format,bo->format,
    gem_handle, bo->pitches[0], bo->fb_id);
    return ret;
  }


  // CopyBufferHandle need layer_cnt.
  unsigned int layer_count;
  for (layer_count = 0; layer_count < HWC_DRM_BO_MAX_PLANES; ++layer_count)
    if (bo->gem_handles[layer_count] == 0)
      break;
  bo->layer_cnt = layer_count;

  // Fix "Failed to close gem handle" bug which lead by no reference counting.
#if 1
  struct drm_gem_close gem_close;
  memset(&gem_close, 0, sizeof(gem_close));

  for (int i = 0; i < HWC_DRM_BO_MAX_PLANES; i++) {
    if (!bo->gem_handles[i])
      continue;

    gem_close.handle = bo->gem_handles[i];
    int ret = drmIoctl(drm_->fd(), DRM_IOCTL_GEM_CLOSE, &gem_close);
    if (ret) {
      ALOGE("Failed to close gem handle %d %d", i, ret);
    } else {
      for (int j = i + 1; j < HWC_DRM_BO_MAX_PLANES; j++)
        if (bo->gem_handles[j] == bo->gem_handles[i])
          bo->gem_handles[j] = 0;
      bo->gem_handles[i] = 0;
    }
  }
#endif
  return ret;
}

int DrmGenericImporter::ReleaseBuffer(hwc_drm_bo_t *bo) {
  if (bo->fb_id)
    if (drmModeRmFB(drm_->fd(), bo->fb_id))
      ALOGE("Failed to rm fb");
#if 0
  struct drm_gem_close gem_close;
  memset(&gem_close, 0, sizeof(gem_close));

  for (int i = 0; i < HWC_DRM_BO_MAX_PLANES; i++) {
    if (!bo->gem_handles[i])
      continue;

    gem_close.handle = bo->gem_handles[i];
    int ret = drmIoctl(drm_->fd(), DRM_IOCTL_GEM_CLOSE, &gem_close);
    if (ret) {
      ALOGE("Failed to close gem handle %d %d", i, ret);
    } else {
      for (int j = i + 1; j < HWC_DRM_BO_MAX_PLANES; j++)
        if (bo->gem_handles[j] == bo->gem_handles[i])
          bo->gem_handles[j] = 0;
      bo->gem_handles[i] = 0;
    }
  }
#endif
  return 0;
}

bool DrmGenericImporter::CanImportBuffer(buffer_handle_t handle) {
  if (handle == NULL)
    return false;

//  if (exclude_non_hwfb_) {
//    gralloc_drm_handle_t *hnd = gralloc_drm_handle(handle);
//    return hnd->usage & GRALLOC_USAGE_HW_FB;
//  }

  return true;
}

#ifdef USE_DRM_GENERIC_IMPORTER
std::unique_ptr<Planner> Planner::CreateInstance(DrmDevice *) {
  std::unique_ptr<Planner> planner(new Planner);
  planner->AddStage<PlanStageVop2>();
  return planner;
}
#endif
}
