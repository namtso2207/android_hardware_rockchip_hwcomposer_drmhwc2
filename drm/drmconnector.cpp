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

#define LOG_TAG "hwc-drm-connector"

#include "drmconnector.h"
#include "drmdevice.h"

#include <errno.h>
#include <stdint.h>

#include <log/log.h>
#include <xf86drmMode.h>

namespace android {

DrmConnector::DrmConnector(DrmDevice *drm, drmModeConnectorPtr c,
                           DrmEncoder *current_encoder,
                           std::vector<DrmEncoder *> &possible_encoders)
    : drm_(drm),
      id_(c->connector_id),
      encoder_(current_encoder),
      display_(-1),
      type_(c->connector_type),
      type_id_(c->connector_type_id),
      priority_(-1),
      state_(c->connection),
      force_disconnect_(false),
      mm_width_(c->mmWidth),
      mm_height_(c->mmHeight),
      possible_encoders_(possible_encoders),
      connector_(c),
      possible_displays_(0) {
}

int DrmConnector::Init() {
  int ret = drm_->GetConnectorProperty(*this, "DPMS", &dpms_property_);
  if (ret) {
    ALOGE("Could not get DPMS property\n");
    return ret;
  }
  ret = drm_->GetConnectorProperty(*this, "CRTC_ID", &crtc_id_property_);
  if (ret) {
    ALOGE("Could not get CRTC_ID property\n");
    return ret;
  }
  if (writeback()) {
    ret = drm_->GetConnectorProperty(*this, "WRITEBACK_PIXEL_FORMATS",
                                     &writeback_pixel_formats_);
    if (ret) {
      ALOGE("Could not get WRITEBACK_PIXEL_FORMATS connector_id = %d\n", id_);
      return ret;
    }
    ret = drm_->GetConnectorProperty(*this, "WRITEBACK_FB_ID",
                                     &writeback_fb_id_);
    if (ret) {
      ALOGE("Could not get WRITEBACK_FB_ID connector_id = %d\n", id_);
      return ret;
    }
    ret = drm_->GetConnectorProperty(*this, "WRITEBACK_OUT_FENCE_PTR",
                                     &writeback_out_fence_);
    if (ret) {
      ALOGE("Could not get WRITEBACK_OUT_FENCE_PTR connector_id = %d\n", id_);
      return ret;
    }
  }

  ret = drm_->GetConnectorProperty(*this, "brightness", &brightness_id_property_);
  if (ret)
    ALOGW("Could not get brightness property\n");

  ret = drm_->GetConnectorProperty(*this, "contrast", &contrast_id_property_);
  if (ret)
    ALOGW("Could not get contrast property\n");

  ret = drm_->GetConnectorProperty(*this, "saturation", &saturation_id_property_);
  if (ret)
    ALOGW("Could not get saturation property\n");

  ret = drm_->GetConnectorProperty(*this, "hue", &hue_id_property_);
  if (ret)
    ALOGW("Could not get hue property\n");

  ret = drm_->GetConnectorProperty(*this, "HDR_OUTPUT_METADATA", &hdr_metadata_property_);
  if (ret)
    ALOGW("Could not get hdr output metadata property\n");

  ret = drm_->GetConnectorProperty(*this, "HDR_PANEL_METADATA", &hdr_panel_property_);
  if (ret)
    ALOGW("Could not get hdr panel metadata property\n");

  ret = drm_->GetConnectorProperty(*this, "hdmi_output_colorimetry", &hdmi_output_colorimetry_);
  if (ret)
    ALOGW("Could not get hdmi_output_colorimetry property\n");

  ret = drm_->GetConnectorProperty(*this, "hdmi_output_format", &hdmi_output_format_);
  if (ret) {
    ALOGW("Could not get hdmi_output_format property\n");
  }

  ret = drm_->GetConnectorProperty(*this, "hdmi_output_depth", &hdmi_output_depth_);
  if (ret) {
   ALOGW("Could not get hdmi_output_depth property\n");
  }

  drm_->GetHdrPanelMetadata(this,&hdr_metadata_);
  bSupportSt2084_ = drm_->is_hdr_panel_support_st2084(this);
  bSupportHLG_    = drm_->is_hdr_panel_support_HLG(this);
  drmHdr_.clear();
  if(bSupportSt2084_){
      drmHdr_.push_back(DrmHdr(DRM_HWC_HDR10,
                        hdr_metadata_.max_mastering_display_luminance,
                        (hdr_metadata_.max_mastering_display_luminance + hdr_metadata_.min_mastering_display_luminance) / 2,
                        hdr_metadata_.min_mastering_display_luminance));
  }
  if(bSupportHLG_){
      drmHdr_.push_back(DrmHdr(DRM_HWC_HLG,
                        hdr_metadata_.max_mastering_display_luminance,
                        (hdr_metadata_.max_mastering_display_luminance + hdr_metadata_.min_mastering_display_luminance) / 2,
                        hdr_metadata_.min_mastering_display_luminance));
  }

  ALOGD("rk-debug connector %u init bSupportSt2084_ = %d, bSupportHLG_ = %d",id_,bSupportSt2084_,bSupportHLG_);
  return 0;
}

uint32_t DrmConnector::id() const {
  return id_;
}

int DrmConnector::display() const {
  return display_;
}

void DrmConnector::set_display(int display) {
  display_ = display;
}
int DrmConnector::priority() const{
  return priority_;
}
void DrmConnector::set_priority(uint32_t priority){
  priority_ = priority;
}

uint32_t DrmConnector::possible_displays() const {
  return possible_displays_;
}

void DrmConnector::set_possible_displays(uint32_t possible_displays) {
  possible_displays_ = possible_displays;
}

bool DrmConnector::internal() const {

  if(!possible_displays_){
    return type_ == DRM_MODE_CONNECTOR_LVDS || type_ == DRM_MODE_CONNECTOR_eDP ||
           type_ == DRM_MODE_CONNECTOR_DSI ||
           type_ == DRM_MODE_CONNECTOR_VIRTUAL || type_ == DRM_MODE_CONNECTOR_DPI;
  }else{
    return (possible_displays_ & HWC_DISPLAY_PRIMARY_BIT) > 0;
  }
}

bool DrmConnector::external() const {
  if(!possible_displays_){
    return type_ == DRM_MODE_CONNECTOR_HDMIA ||
           type_ == DRM_MODE_CONNECTOR_DisplayPort ||
           type_ == DRM_MODE_CONNECTOR_DVID || type_ == DRM_MODE_CONNECTOR_DVII ||
           type_ == DRM_MODE_CONNECTOR_VGA;
  }else{
    return (possible_displays_ & HWC_DISPLAY_EXTERNAL_BIT) > 0;
  }
}

bool DrmConnector::writeback() const {
#ifdef DRM_MODE_CONNECTOR_WRITEBACK
  return type_ == DRM_MODE_CONNECTOR_WRITEBACK;
#else
  return false;
#endif
}

bool DrmConnector::valid_type() const {
  return internal() || external() || writeback();
}

int DrmConnector::UpdateModes() {
  int fd = drm_->fd();

  drmModeConnectorPtr c = drmModeGetConnector(fd, id_);
  if (!c) {
    ALOGE("Failed to get connector %d", id_);
    return -ENODEV;
  }

  //When Plug-in/Plug-out TV panel,some Property of the connector will need be updated.
  bSupportSt2084_ = drm_->is_hdr_panel_support_st2084(this);
  bSupportHLG_    = drm_->is_hdr_panel_support_HLG(this);

  state_ = c->connection;

  if (!c->count_modes)
    state_ = DRM_MODE_DISCONNECTED;

  bool preferred_mode_found = false;
  std::vector<DrmMode> new_modes;
  for (int i = 0; i < c->count_modes; ++i) {
    bool exists = false;
    for (const DrmMode &mode : modes_) {
      if (mode == c->modes[i]) {
        new_modes.push_back(mode);
        exists = true;
        break;
      }
    }
    if (!exists) {
      DrmMode m(&c->modes[i]);
      m.set_id(drm_->next_mode_id());
      new_modes.push_back(m);
    }

    //Get original mode from connector
    std::vector<DrmMode> new_raw_modes;
    for (int i = 0; i < c->count_modes; ++i) {
      bool exists = false;
      for (const DrmMode &mode : modes_) {
        if (mode == c->modes[i]) {
          new_raw_modes.push_back(mode);
          exists = true;
          break;
        }
      }
      if (exists)
        continue;

      DrmMode m(&c->modes[i]);
      m.set_id(drm_->next_mode_id());
      new_raw_modes.push_back(m);
    }
    raw_modes_.swap(new_raw_modes);

    // Use only the first DRM_MODE_TYPE_PREFERRED mode found
    if (!preferred_mode_found &&
        (new_modes.back().type() & DRM_MODE_TYPE_PREFERRED)) {
      preferred_mode_id_ = new_modes.back().id();
      preferred_mode_found = true;
    }
  }
  modes_.swap(new_modes);
  if (!preferred_mode_found && modes_.size() != 0) {
    preferred_mode_id_ = modes_[0].id();
  }

  bModeReady_ = true;
  return 0;
}

const DrmMode &DrmConnector::active_mode() const {
  return active_mode_;
}

const DrmMode &DrmConnector::best_mode() const {
  return best_mode_;
}

const DrmMode &DrmConnector::current_mode() const {
  return current_mode_;
}

void DrmConnector::set_best_mode(const DrmMode &mode) {
  best_mode_ = mode;
}

void DrmConnector::set_active_mode(const DrmMode &mode) {
  active_mode_ = mode;
}

void DrmConnector::set_current_mode(const DrmMode &mode) {
  current_mode_ = mode;
}

void DrmConnector::SetDpmsMode(uint32_t dpms_mode) {
  int ret = drmModeConnectorSetProperty(drm_->fd(), id_, dpms_property_.id(), dpms_mode);
  if (ret) {
    ALOGE("Failed to set dpms mode %d %d", ret, dpms_mode);
    return;
  }
}

const DrmProperty &DrmConnector::dpms_property() const {
  return dpms_property_;
}

const DrmProperty &DrmConnector::crtc_id_property() const {
  return crtc_id_property_;
}

const DrmProperty &DrmConnector::writeback_pixel_formats() const {
  return writeback_pixel_formats_;
}

const DrmProperty &DrmConnector::writeback_fb_id() const {
  return writeback_fb_id_;
}

const DrmProperty &DrmConnector::writeback_out_fence() const {
  return writeback_out_fence_;
}

DrmEncoder *DrmConnector::encoder() const {
  return encoder_;
}

void DrmConnector::set_encoder(DrmEncoder *encoder) {
  encoder_ = encoder;
}

drmModeConnection DrmConnector::state() const {
  if (force_disconnect_)
    return DRM_MODE_DISCONNECTED;
  return state_;
}

drmModeConnection DrmConnector::raw_state() const {
  return state_;
}

uint32_t DrmConnector::mm_width() const {
  return mm_width_;
}

uint32_t DrmConnector::mm_height() const {
  return mm_height_;
}

bool DrmConnector::is_hdmi_support_hdr() const
{
    return (hdr_metadata_property_.id() && bSupportSt2084_) || (hdr_metadata_property_.id() && bSupportHLG_);
}

const DrmProperty &DrmConnector::brightness_id_property() const {
  return brightness_id_property_;
}
const DrmProperty &DrmConnector::contrast_id_property() const {
  return contrast_id_property_;
}
const DrmProperty &DrmConnector::saturation_id_property() const {
  return saturation_id_property_;
}
const DrmProperty &DrmConnector::hue_id_property() const {
  return hue_id_property_;
}

const DrmProperty &DrmConnector::hdr_metadata_property() const {
  return hdr_metadata_property_;
}

const DrmProperty &DrmConnector::hdr_panel_property() const {
  return hdr_panel_property_;
}

const DrmProperty &DrmConnector::hdmi_output_colorimetry_property() const {
  return hdmi_output_colorimetry_;
}

const DrmProperty &DrmConnector::hdmi_output_format_property() const {
  return hdmi_output_format_;
}

const DrmProperty &DrmConnector::hdmi_output_depth_property() const {
  return hdmi_output_depth_;
}
void DrmConnector::force_disconnect(bool force) {
    force_disconnect_ = force;
}

}  // namespace android
