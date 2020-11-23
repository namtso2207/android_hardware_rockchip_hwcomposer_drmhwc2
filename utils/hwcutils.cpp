/*
 * Copyright (C) 2016 The Android Open Source Project
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

#define ATRACE_TAG ATRACE_TAG_GRAPHICS
#define LOG_TAG "hwc-drm-utils"

#include "drmhwcomposer.h"
#include "platform.h"

#include <log/log.h>
#include <ui/GraphicBufferMapper.h>
#include <cutils/properties.h>


#define hwcMIN(x, y)			(((x) <= (y)) ?  (x) :  (y))
#define hwcMAX(x, y)			(((x) >= (y)) ?  (x) :  (y))
#define IS_ALIGN(val,align)    (((val)&(align-1))==0)
#ifndef ALIGN
#define ALIGN( value, base ) (((value) + ((base) - 1)) & ~((base) - 1))
#endif
#define ALIGN_DOWN( value, base)	(value & (~(base-1)) )


namespace android {

int hwc_get_int_property(const char* pcProperty,const char* default_value)
{
    char value[PROPERTY_VALUE_MAX];
    int new_value = 0;

    if(pcProperty == NULL || default_value == NULL)
    {
        ALOGE("hwc_get_int_property: invalid param");
        return -1;
    }

    property_get(pcProperty, value, default_value);
    new_value = atoi(value);

    return new_value;
}

bool hwc_get_bool_property(const char* pcProperty,const char* default_value)
{
    char value[PROPERTY_VALUE_MAX];
    bool result = false;

    if(pcProperty == NULL || default_value == NULL)
    {
        ALOGE("hwc_get_int_property: invalid param");
        return -1;
    }

    property_get(pcProperty, value, default_value);
    if(!strcmp(value,"true"))
        result = true;
    else
        result = false;

    return result;
}


int hwc_get_string_property(const char* pcProperty,const char* default_value,char* retult)
{
    if(pcProperty == NULL || default_value == NULL || retult == NULL)
    {
        ALOGE("hwc_get_string_property: invalid param");
        return -1;
    }
    property_get(pcProperty, retult, default_value);

    return 0;
}

const hwc_drm_bo *DrmHwcBuffer::operator->() const {
  if (importer_ == NULL) {
    ALOGE("Access of non-existent BO");
    exit(1);
    return NULL;
  }
  return &bo_;
}

void DrmHwcBuffer::Clear() {
  if (importer_ != NULL) {
    importer_->ReleaseBuffer(&bo_);
    importer_ = NULL;
  }
}

int DrmHwcBuffer::ImportBuffer(buffer_handle_t handle, Importer *importer) {
  hwc_drm_bo tmp_bo;

  int ret = importer->ImportBuffer(handle, &tmp_bo);
  if (ret)
    return ret;

  if (importer_ != NULL) {
    importer_->ReleaseBuffer(&bo_);
  }

  importer_ = importer;

  bo_ = tmp_bo;

  return 0;
}

int DrmHwcLayer::ImportBuffer(Importer *importer) {
  int ret = buffer.ImportBuffer(sf_handle, importer);
  if (ret)
    return ret;

  const hwc_drm_bo *bo = buffer.operator->();

  gralloc_buffer_usage = bo->usage;

  return 0;
}
int DrmHwcLayer::Init() {
  bYuv_ = IsYuvFormat(iFormat_);
  bScale_  = IsScale(source_crop, display_frame, transform);
  iSkipLine_  = GetSkipLine();
  bAfbcd_ = IsAfbcInternalFormat(uInternalFormat_);
  bSkipLayer_ = IsSkipLayer();

  // HDR
  bHdr_ = IsHdr(iUsage);
  uColorSpace = GetColorSpace(eDataSpace_);
  uEOTF = GetEOTF(eDataSpace_);
  return 0;
}

int DrmHwcLayer::InitFromDrmHwcLayer(DrmHwcLayer *src_layer,
                                     Importer *importer) {
  blending = src_layer->blending;
  sf_handle = src_layer->sf_handle;
  acquire_fence = -1;
  display_frame = src_layer->display_frame;
  alpha = src_layer->alpha;
  source_crop = src_layer->source_crop;
  transform = src_layer->transform;
  return ImportBuffer(importer);
}
void DrmHwcLayer::SetSourceCrop(hwc_frect_t const &crop) {
  source_crop = crop;
}

void DrmHwcLayer::SetDisplayFrame(hwc_rect_t const &frame) {
  display_frame = frame;
}

void DrmHwcLayer::SetTransform(int32_t sf_transform) {
  transform = DrmHwcTransform::kRotate0;
  // 270* and 180* cannot be combined with flips. More specifically, they
  // already contain both horizontal and vertical flips, so those fields are
  // redundant in this case. 90* rotation can be combined with either horizontal
  // flip or vertical flip, so treat it differently
  if (sf_transform == HWC_TRANSFORM_ROT_270) {
    transform = DrmHwcTransform::kRotate270;
  } else if (sf_transform == HWC_TRANSFORM_ROT_180) {
    transform = DrmHwcTransform::kRotate180;
  } else {
    if (sf_transform & HWC_TRANSFORM_FLIP_H)
      transform |= DrmHwcTransform::kFlipH;
    if (sf_transform & HWC_TRANSFORM_FLIP_V)
      transform |= DrmHwcTransform::kFlipV;
    if (sf_transform & HWC_TRANSFORM_ROT_90)
      transform |= DrmHwcTransform::kRotate90;
  }
}
bool DrmHwcLayer::IsYuvFormat(int format){
  switch(format){
    case HAL_PIXEL_FORMAT_YCrCb_NV12:
    case HAL_PIXEL_FORMAT_YCrCb_NV12_10:
    case HAL_PIXEL_FORMAT_YCrCb_NV12_VIDEO:
      return true;
    default:
      return false;
  }
}
bool DrmHwcLayer::IsScale(hwc_frect_t &source_crop, hwc_rect_t &display_frame, int transform){
  int src_w, src_h, dst_w, dst_h;
  src_w = (int)(source_crop.right - source_crop.left);
  src_h = (int)(source_crop.bottom - source_crop.top);
  dst_w = (int)(display_frame.right - display_frame.left);
  dst_h = (int)(display_frame.bottom - display_frame.top);

  if((transform == DrmHwcTransform::kRotate90) || (transform == DrmHwcTransform::kRotate270)){
    if(bYuv_){
        //rga need this alignment.
        src_h = ALIGN_DOWN(src_h, 8);
        src_w = ALIGN_DOWN(src_w, 2);
    }
    fHScaleMul_ = (float) (src_h)/(dst_w);
    fVScaleMul_ = (float) (src_w)/(dst_h);
  } else {
    fHScaleMul_ = (float) (src_w)/(dst_w);
    fVScaleMul_ = (float) (src_h)/(dst_h);
  }
  return (fHScaleMul_ != 1.0 ) || ( fVScaleMul_ != 1.0);
}
bool DrmHwcLayer::IsHdr(int usage){
  return ((usage & 0x0F000000) == HDR_ST2084_USAGE || (usage & 0x0F000000) == HDR_HLG_USAGE);
}
bool DrmHwcLayer::IsAfbcInternalFormat(uint64_t internal_format){
  return (internal_format & GRALLOC_ARM_INTFMT_AFBC);             // for Midgard gralloc r14
}

bool DrmHwcLayer::IsSkipLayer(){
  return (!sf_handle ? true:false) ||
         (iFormat_ == HAL_PIXEL_FORMAT_RGBA_1010102);
}

int DrmHwcLayer::GetSkipLine(){
    int skip_line = 0;
    if(bYuv_){
      if(iWidth_ >= 3840){
        if(fHScaleMul_ > 1.0 || fVScaleMul_ > 1.0){
            skip_line = 2;
        }
        if(iFormat_ == HAL_PIXEL_FORMAT_YCrCb_NV12_10 && fHScaleMul_ >= (3840 / 1600)){
            skip_line = 3;
        }
      }
      int video_skipline = property_get_int32("vendor.video.skipline", 0);
      if (video_skipline == 2){
        skip_line = 2;
      }else if(video_skipline == 3){
        skip_line = 3;
      }
    }
    return (skip_line >= 0 ? skip_line : 0);
}

#define CONTAIN_VALUE(value,mask) ((dataspace & mask) == value)
v4l2_colorspace DrmHwcLayer::GetColorSpace(android_dataspace_t dataspace){
  if (CONTAIN_VALUE(HAL_DATASPACE_STANDARD_BT2020, HAL_DATASPACE_STANDARD_MASK)){
      return V4L2_COLORSPACE_BT2020;
  }
  else if (CONTAIN_VALUE(HAL_DATASPACE_STANDARD_BT601_625, HAL_DATASPACE_STANDARD_MASK) &&
          CONTAIN_VALUE(HAL_DATASPACE_TRANSFER_SMPTE_170M, HAL_DATASPACE_TRANSFER_MASK)){
      if (CONTAIN_VALUE(HAL_DATASPACE_RANGE_FULL, HAL_DATASPACE_RANGE_MASK))
          return V4L2_COLORSPACE_JPEG;
      else if (CONTAIN_VALUE(HAL_DATASPACE_RANGE_LIMITED, HAL_DATASPACE_RANGE_MASK))
          return V4L2_COLORSPACE_SMPTE170M;
  }
  else if (CONTAIN_VALUE(HAL_DATASPACE_STANDARD_BT601_525, HAL_DATASPACE_STANDARD_MASK) &&
          CONTAIN_VALUE(HAL_DATASPACE_TRANSFER_SMPTE_170M, HAL_DATASPACE_TRANSFER_MASK) &&
          CONTAIN_VALUE(HAL_DATASPACE_RANGE_LIMITED, HAL_DATASPACE_RANGE_MASK)){
      return V4L2_COLORSPACE_SMPTE170M;
  }
  else if (CONTAIN_VALUE(HAL_DATASPACE_STANDARD_BT709, HAL_DATASPACE_STANDARD_MASK) &&
      CONTAIN_VALUE(HAL_DATASPACE_TRANSFER_SMPTE_170M, HAL_DATASPACE_TRANSFER_MASK) &&
      CONTAIN_VALUE(HAL_DATASPACE_RANGE_LIMITED, HAL_DATASPACE_RANGE_MASK)){
      return V4L2_COLORSPACE_REC709;
  }
  else if (CONTAIN_VALUE(HAL_DATASPACE_TRANSFER_SRGB, HAL_DATASPACE_TRANSFER_MASK)){
      return V4L2_COLORSPACE_SRGB;
  }
  //ALOGE("Unknow colorspace 0x%x",colorspace);
  return V4L2_COLORSPACE_DEFAULT;

}

supported_eotf_type DrmHwcLayer::GetEOTF(android_dataspace_t dataspace){
  if(bYuv_){
    if((dataspace & HAL_DATASPACE_TRANSFER_MASK) == HAL_DATASPACE_TRANSFER_ST2084){
        ALOGD_IF(LogLevel(DBG_VERBOSE),"%s:line=%d has st2084",__FUNCTION__,__LINE__);
        return SMPTE_ST2084;
    }else{
        //ALOGE("Unknow etof %d",eotf);
        return TRADITIONAL_GAMMA_SDR;
    }
  }

  return TRADITIONAL_GAMMA_SDR;
}

std::string DrmHwcLayer::TransformToString(uint32_t transform) const{
  switch (transform) {
    case DrmHwcTransform::kIdentity:
      return "IDENTITY";
    case DrmHwcTransform::kFlipH:
      return "FLIPH";
    case DrmHwcTransform::kFlipV:
      return "FLIPV";
    case DrmHwcTransform::kRotate90:
      return "ROTATE90";
    case DrmHwcTransform::kRotate180:
      return "ROTATE180";
    case DrmHwcTransform::kRotate270:
      return "ROTATE270";
    default:
      return "<invalid>";
  }
}
std::string DrmHwcLayer::BlendingToString(DrmHwcBlending blending) const{
  switch (blending) {
    case DrmHwcBlending::kNone:
      return "NONE";
    case DrmHwcBlending::kPreMult:
      return "PREMULT";
    case DrmHwcBlending::kCoverage:
      return "COVERAGE";
    default:
      return "<invalid>";
  }
}

int DrmHwcLayer::DumpInfo(String8 &out){
    if(bFbTarget_)
      out.appendFormat( "DrmHwcFBtar[%4u] Buffer[w/h/s/format]=[%4d,%4d,%4d,%4d] Transform=%-8.8s Blend[a=%d]=%-8.8s "
                    "source_crop[l,t,r,b]=[%4.2f,%4.2f,%4.2f,%4.2f] display_frame[l,t,r,b]=[%4d,%4d,%4d,%4d]\n",
                   uId_,iWidth_,iHeight_,iStride_,iFormat_,TransformToString(transform).c_str(),
                   alpha,BlendingToString(blending).c_str(),
                   source_crop.left,source_crop.top,source_crop.right,source_crop.bottom,
                   display_frame.left,display_frame.top,display_frame.right,display_frame.bottom);
    else
      out.appendFormat( "DrmHwcLayer[%4u] Buffer[w/h/s/format]=[%4d,%4d,%4d,%4d] Transform=%-8.8s Blend[a=%d]=%-8.8s "
                        "source_crop[l,t,r,b]=[%4.2f,%4.2f,%4.2f,%4.2f] display_frame[l,t,r,b]=[%4d,%4d,%4d,%4d],skip=%d,afbcd=%d\n",
                       uId_,iWidth_,iHeight_,iStride_,iFormat_,TransformToString(transform).c_str(),
                       alpha,BlendingToString(blending).c_str(),
                       source_crop.left,source_crop.top,source_crop.right,source_crop.bottom,
                       display_frame.left,display_frame.top,display_frame.right,display_frame.bottom,bSkipLayer_,bAfbcd_);
    return 0;
}

}  // namespace android
