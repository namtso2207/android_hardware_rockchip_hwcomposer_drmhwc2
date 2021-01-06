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
        if(type_ == DRM_MODE_CONNECTOR_HDMIA || type_ == DRM_MODE_CONNECTOR_DisplayPort){
            //filter mode by /system/usr/share/resolution_white.xml.
            if(drm_->mode_verify(mode)){
                new_modes.push_back(mode);
                exists = true;
                break;
            }
        }else{
            new_modes.push_back(mode);
            exists = true;
            break;
        }
      }
    }
    if (exists)
      continue;

    DrmMode m(&c->modes[i]);
    if ((type_ == DRM_MODE_CONNECTOR_HDMIA || type_ == DRM_MODE_CONNECTOR_DisplayPort) && !drm_->mode_verify(m))
      continue;

    m.set_id(drm_->next_mode_id());
    new_modes.push_back(m);

    // Use only the first DRM_MODE_TYPE_PREFERRED mode found
    if (!preferred_mode_found &&
        (new_modes.back().type() & DRM_MODE_TYPE_PREFERRED)) {
      preferred_mode_id_ = new_modes.back().id();
      preferred_mode_found = true;
    }
  }
  modes_.swap(new_modes);

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

int DrmConnector::switch_hdmi_hdr_mode(android_dataspace_t colorspace){
  ALOGD_IF(LogLevel(DBG_DEBUG),"%s:line=%d, connector-id=%d, isSupportSt2084 = %d, isSupportHLG = %d , colorspace = %x",
            __FUNCTION__,__LINE__,id(),isSupportSt2084(),isSupportHLG(),colorspace);
  struct hdr_output_metadata hdr_metadata;
  memset(&hdr_metadata, 0, sizeof(struct hdr_output_metadata));

  if((colorspace & HAL_DATASPACE_TRANSFER_MASK) == HAL_DATASPACE_TRANSFER_ST2084
      && isSupportSt2084()){
      ALOGD_IF(LogLevel(DBG_DEBUG),"%s:line=%d has st2084",__FUNCTION__,__LINE__);
      hdr_metadata.hdmi_metadata_type.eotf = SMPTE_ST2084;
  }else if((colorspace & HAL_DATASPACE_TRANSFER_MASK) == HAL_DATASPACE_TRANSFER_HLG
      && isSupportHLG()){
      ALOGD_IF(LogLevel(DBG_DEBUG),"%s:line=%d has HLG",__FUNCTION__,__LINE__);
      hdr_metadata.hdmi_metadata_type.eotf = HLG;
  }else{
      //ALOGE("Unknow etof %d",eotf);
      hdr_metadata.hdmi_metadata_type.eotf = TRADITIONAL_GAMMA_SDR;
  }

  uint32_t blob_id = 0;
  int colorimetry = 0;
  int ret = -1;
  bool hdr_state_update = false;
  if(hdr_metadata_property().id())
  {
      ALOGD_IF(LogLevel(DBG_DEBUG),"%s: android_colorspace = 0x%x", __FUNCTION__, colorspace);
      drmModeAtomicReqPtr pset = drmModeAtomicAlloc();
      if (!pset) {
          ALOGE("%s:line=%d Failed to allocate property set", __FUNCTION__, __LINE__);
          return -1;
      }
      if(!memcmp(&last_hdr_metadata_, &hdr_metadata, sizeof(struct hdr_output_metadata))){
          ALOGD_IF(LogLevel(DBG_DEBUG),"%s: no need to update metadata", __FUNCTION__);
      }else{
        hdr_state_update = true;
        ALOGD_IF(LogLevel(DBG_DEBUG),"%s: hdr_metadata eotf=0x%x", __FUNCTION__,hdr_metadata.hdmi_metadata_type.eotf);
        drm_->CreatePropertyBlob(&hdr_metadata, sizeof(struct hdr_output_metadata), &blob_id);
        ret = drmModeAtomicAddProperty(pset, id(), hdr_metadata_property().id(), blob_id);
        if (ret < 0) {
          ALOGE("%s:line=%d Failed to add prop[%d] to [%d]", __FUNCTION__, __LINE__, hdr_metadata_property().id(), id());
        }
      }

      if(hdmi_output_colorimetry_property().id()){
          if((colorspace & HAL_DATASPACE_STANDARD_BT2020) == HAL_DATASPACE_STANDARD_BT2020){
              colorimetry = COLOR_METRY_ITU_2020;
          }

          if(colorimetry_ != colorimetry){
              hdr_state_update = true;
              ALOGD_IF(LogLevel(DBG_DEBUG),"%s: change bt2020 colorimetry=%d", __FUNCTION__, colorimetry);
              ret = drmModeAtomicAddProperty(pset, id(), hdmi_output_colorimetry_property().id(), colorimetry);
              if (ret < 0) {
                ALOGE("%s:line=%d Failed to add prop[%d] to [%d]", __FUNCTION__, __LINE__,hdmi_output_colorimetry_property().id(), id());
              }
          }else{
              ALOGD_IF(LogLevel(DBG_DEBUG),"%s: no need to update colorimetry", __FUNCTION__);
          }
      }
      if(hdr_state_update){
        ret = drmModeAtomicCommit(drm_->fd(), pset, DRM_MODE_ATOMIC_ALLOW_MODESET, drm_);
        if (ret < 0) {
            ALOGE("%s:line=%d Failed to commit pset ret=%d\n", __FUNCTION__, __LINE__, ret);
            drmModeAtomicFree(pset);
            return ret;
        }else{
            memcpy(&last_hdr_metadata_, &hdr_metadata, sizeof(struct hdr_output_metadata));
            colorimetry_ = colorimetry;
        }
      }
      if (blob_id)
          drm_->DestroyPropertyBlob(blob_id);

      drmModeAtomicFree(pset);
      return 0;
  }
  else
  {
      ALOGD_IF(LogLevel(DBG_DEBUG),"%s: hdmi don't support hdr metadata", __FUNCTION__);
      return -1;
  }
  return -1;
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
