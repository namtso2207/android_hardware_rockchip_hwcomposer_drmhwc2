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

#define LOG_TAG "hwc-drm-crtc"

#include "drmcrtc.h"
#include "drmdevice.h"

#include <stdint.h>
#include <xf86drmMode.h>

#include <log/log.h>

namespace android {

DrmCrtc::DrmCrtc(DrmDevice *drm, drmModeCrtcPtr c, unsigned pipe)
    : drm_(drm), id_(c->crtc_id), pipe_(pipe), display_(-1), mode_(&c->mode) {
}

int DrmCrtc::Init() {
  int ret = drm_->GetCrtcProperty(*this, "ACTIVE", &active_property_);
  if (ret) {
    ALOGE("Failed to get ACTIVE property");
    return ret;
  }

  ret = drm_->GetCrtcProperty(*this, "MODE_ID", &mode_property_);
  if (ret) {
    ALOGE("Failed to get MODE_ID property");
    return ret;
  }

  ret = drm_->GetCrtcProperty(*this, "FEATURE", &feature_property_);
  if (ret)
    ALOGE("Could not get FEATURE property");


  uint64_t feature=0;
  feature_property_.set_feature("afbdc");
  std::tie(ret,feature) = feature_property_.value();
  b_afbc_ = (feature ==1)?true:false;


  can_overscan_ = true;
  ret = drm_->GetCrtcProperty(*this, "left margin", &left_margin_property_);
  if (ret) {
    ALOGE("Failed to get left margin property");
    can_overscan_ = false;
  }
  ret = drm_->GetCrtcProperty(*this, "right margin", &right_margin_property_);
  if (ret) {
    ALOGE("Failed to get right margin property");
    can_overscan_ = false;
  }
  ret = drm_->GetCrtcProperty(*this, "top margin", &top_margin_property_);
  if (ret) {
    ALOGE("Failed to get top margin property");
    can_overscan_ = false;
  }
  ret = drm_->GetCrtcProperty(*this, "bottom margin", &bottom_margin_property_);
  if (ret) {
    ALOGE("Failed to get bottom margin property");
    can_overscan_ = false;
  }

  uint64_t alpha_scale = 0;
  can_alpha_scale_ = true;
  ret = drm_->GetCrtcProperty(*this, "ALPHA_SCALE", &alpha_scale_property_);
  if (ret) {
    ALOGE("Failed to get alpha_scale_property property");
  }
  std::tie(alpha_scale, ret) = alpha_scale_property_.value();
  if(alpha_scale == 0)
    can_alpha_scale_ = false;


  ret = drm_->GetCrtcProperty(*this, "OUT_FENCE_PTR", &out_fence_ptr_property_);
  if (ret) {
    ALOGE("Failed to get OUT_FENCE_PTR property");
  }

  ret = drm_->GetCrtcProperty(*this, "SOC_ID", &soc_type_property_);
  if (ret) {
    ALOGE("Failed to get SOC_ID property");
  }
  std::tie(ret, soc_id_) = soc_type_property_.value();
  if(ret)
    ALOGE("Failed to get SOC_ID value");

  ret = drm_->GetCrtcProperty(*this, "PORT_ID", &port_id_property_);
  if (ret) {
    ALOGE("Failed to get PORT_ID property");
  }
  std::tie(ret, port_id_) = port_id_property_.value();
  if(ret)
    ALOGE("Failed to get PORT_ID value");

  ret = drm_->GetCrtcProperty(*this, "ACLK", &aclk_property_);
  if (ret) {
    ALOGE("Failed to get PORT_ID property");
  }
  std::tie(ret, aclk_) = aclk_property_.value();
  if(ret)
    ALOGE("Failed to get ACLK value");

  ALOGD("rk-debug aclk_ = %d ",aclk_);



  return 0;
}
bool DrmCrtc::get_afbc() const {
    return b_afbc_;
}

bool DrmCrtc::get_alpha_scale() const {
    return can_alpha_scale_;
}

uint32_t DrmCrtc::id() const {
  return id_;
}

unsigned DrmCrtc::pipe() const {
  return pipe_;
}

int DrmCrtc::display() const {
  return display_;
}

void DrmCrtc::set_display(int display) {
  display_ = display;
}

bool DrmCrtc::can_bind(int display) const {
  return display_ == -1 || display_ == display;
}

const DrmProperty &DrmCrtc::active_property() const {
  return active_property_;
}

const DrmProperty &DrmCrtc::mode_property() const {
  return mode_property_;
}
const DrmProperty &DrmCrtc::out_fence_ptr_property() const {
  return out_fence_ptr_property_;
}

bool DrmCrtc::can_overscan() const {
  return can_overscan_;
}

const DrmProperty &DrmCrtc::left_margin_property() const {
  return left_margin_property_;
}

const DrmProperty &DrmCrtc::right_margin_property() const {
  return right_margin_property_;
}

const DrmProperty &DrmCrtc::top_margin_property() const {
  return top_margin_property_;
}

const DrmProperty &DrmCrtc::bottom_margin_property() const {
  return bottom_margin_property_;
}

const DrmProperty &DrmCrtc::alpha_scale_property() const {
  return alpha_scale_property_;
}

}  // namespace android
