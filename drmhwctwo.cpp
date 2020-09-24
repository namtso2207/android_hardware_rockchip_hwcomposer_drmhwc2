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
#define LOG_TAG "hwc-drm-two"

#include "drmhwctwo.h"
#include "drmdisplaycomposition.h"
#include "drmhwcomposer.h"
#include "platform.h"
#include "vsyncworker.h"
#include "rockchip/utils/drmdebug.h"
#include "rockchip/drmgralloc.h"

#include <inttypes.h>
#include <string>

#include <cutils/properties.h>
#include <hardware/hardware.h>
#include <hardware/hwcomposer2.h>
#include <log/log.h>
#include <utils/Trace.h>


#define ALOGD_HWC2_INFO(log_level) \
    ALOGD_IF(LogLevel(log_level),"%s,line=%d",__FUNCTION__,__LINE__)

#define ALOGD_HWC2_DISPLAY_INFO(log_level, display_id) \
    ALOGD_IF(LogLevel(log_level),"%s, Display-id=%" PRIu64 ",line=%d",__FUNCTION__,\
                        display_id, __LINE__)

#define ALOGD_HWC2_LAYER_INFO(log_level, layer_id) \
    ALOGD_IF(LogLevel(log_level),"%s, Layer-id=%" PRIu32 ",line=%d",__FUNCTION__,\
                        layer_id, __LINE__)

#define ALOGD_HWC2_DRM_LAYER_INFO(log_level, drmHwcLayers) \
    if(LogLevel(log_level)){ \
      String8 output; \
      for(auto &drmHwcLayer : drmHwcLayers) \
        drmHwcLayer.DumpInfo(output); \
      ALOGD_IF(LogLevel(log_level),"%s",output.string()); \
    }

namespace android {

class DrmVsyncCallback : public VsyncCallback {
 public:
  DrmVsyncCallback(hwc2_callback_data_t data, hwc2_function_pointer_t hook)
      : data_(data), hook_(hook) {
  }

  void Callback(int display, int64_t timestamp) {
    auto hook = reinterpret_cast<HWC2_PFN_VSYNC>(hook_);
    hook(data_, display, timestamp);
  }

 private:
  hwc2_callback_data_t data_;
  hwc2_function_pointer_t hook_;
};

DrmHwcTwo::DrmHwcTwo() {
  common.tag = HARDWARE_DEVICE_TAG;
  common.version = HWC_DEVICE_API_VERSION_2_0;
  common.close = HookDevClose;
  getCapabilities = HookDevGetCapabilities;
  getFunction = HookDevGetFunction;
}

HWC2::Error DrmHwcTwo::CreateDisplay(hwc2_display_t displ,
                                     HWC2::DisplayType type) {
  ALOGD_HWC2_INFO(DBG_VERBOSE);
  DrmDevice *drm = resource_manager_.GetDrmDevice(displ);
  std::shared_ptr<Importer> importer = resource_manager_.GetImporter(displ);
  if (!drm || !importer) {
    ALOGE("Failed to get a valid drmresource and importer");
    return HWC2::Error::NoResources;
  }
  displays_.emplace(std::piecewise_construct, std::forward_as_tuple(displ),
                    std::forward_as_tuple(&resource_manager_, drm, importer,
                                          displ, type));
  displays_.at(displ).Init();
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::Init() {
  ALOGD_HWC2_INFO(DBG_VERBOSE);
  int rv = resource_manager_.Init();
  if (rv) {
    ALOGE("Can't initialize the resource manager %d", rv);
    return HWC2::Error::NoResources;
  }

  HWC2::Error ret = HWC2::Error::None;
  for (int i = 0; i < resource_manager_.getDisplayCount(); i++) {
    ret = CreateDisplay(i, HWC2::DisplayType::Physical);
    if (ret != HWC2::Error::None) {
      ALOGE("Failed to create display %d with error %d", i, ret);
      return ret;
    }
  }

  auto &drmDevices = resource_manager_.getDrmDevices();
  for (auto &device : drmDevices) {
    device->RegisterHotplugHandler(new DrmHotplugHandler(this, device.get()));
  }
  return ret;
}

template <typename... Args>
static inline HWC2::Error unsupported(char const *func, Args... /*args*/) {
  ALOGV("Unsupported function: %s", func);
  return HWC2::Error::Unsupported;
}

static inline void supported(char const *func) {
  ALOGV("Supported function: %s", func);
}

HWC2::Error DrmHwcTwo::CreateVirtualDisplay(uint32_t width, uint32_t height,
                                            int32_t *format,
                                            hwc2_display_t *display) {
  ALOGD_HWC2_INFO(DBG_VERBOSE);
  // TODO: Implement virtual display
  return unsupported(__func__, width, height, format, display);
}

HWC2::Error DrmHwcTwo::DestroyVirtualDisplay(hwc2_display_t display) {

  ALOGD_HWC2_INFO(DBG_VERBOSE);
  // TODO: Implement virtual display
  return unsupported(__func__, display);
}

void DrmHwcTwo::Dump(uint32_t *size, char *buffer) {
  if (buffer != nullptr) {
      auto copiedBytes = mDumpString.copy(buffer, *size);
      *size = static_cast<uint32_t>(copiedBytes);
      return;
  }
  String8 output;

  output.appendFormat("-- HWC2 Version 2.0 by Bing --\n");
  for(auto &map_disp: displays_){
    output.append("\n");
    if((map_disp.second.DumpDisplayInfo(output)) < 0)
      continue;
  }
  mDumpString = output.string();
  *size = static_cast<uint32_t>(mDumpString.size());
  return;
}

uint32_t DrmHwcTwo::GetMaxVirtualDisplayCount() {
  ALOGD_HWC2_INFO(DBG_VERBOSE);
  // TODO: Implement virtual display
  unsupported(__func__);
  return 0;
}

HWC2::Error DrmHwcTwo::RegisterCallback(int32_t descriptor,
                                        hwc2_callback_data_t data,
                                        hwc2_function_pointer_t function) {
  ALOGD_HWC2_INFO(DBG_VERBOSE);
  auto callback = static_cast<HWC2::Callback>(descriptor);

  if (!function) {
    callbacks_.erase(callback);
    return HWC2::Error::None;
  }

  callbacks_.emplace(callback, HwcCallback(data, function));

  switch (callback) {
    case HWC2::Callback::Hotplug: {
      auto hotplug = reinterpret_cast<HWC2_PFN_HOTPLUG>(function);
      hotplug(data, HWC_DISPLAY_PRIMARY,
              static_cast<int32_t>(HWC2::Connection::Connected));
      auto &drmDevices = resource_manager_.getDrmDevices();
      for (auto &device : drmDevices)
        HandleInitialHotplugState(device.get());
      break;
    }
    case HWC2::Callback::Vsync: {
      for (std::pair<const hwc2_display_t, DrmHwcTwo::HwcDisplay> &d :
           displays_)
        d.second.RegisterVsyncCallback(data, function);
      break;
    }
    case HWC2::Callback::Refresh: {
      for (std::pair<const hwc2_display_t, DrmHwcTwo::HwcDisplay> &d :
           displays_)
        d.second.RegisterVsyncCallback(data, function);
        break;
    }
    default:
      break;
  }
  return HWC2::Error::None;
}

DrmHwcTwo::HwcDisplay::HwcDisplay(ResourceManager *resource_manager,
                                  DrmDevice *drm,
                                  std::shared_ptr<Importer> importer,
                                  hwc2_display_t handle, HWC2::DisplayType type)
    : resource_manager_(resource_manager),
      drm_(drm),
      importer_(importer),
      handle_(handle),
      type_(type),
      client_layer_(UINT32_MAX, resource_manager->gralloc()),
      init_success_(false){
}

void DrmHwcTwo::HwcDisplay::ClearDisplay() {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  compositor_.ClearDisplay();
}

HWC2::Error DrmHwcTwo::HwcDisplay::Init() {

  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);

  int display = static_cast<int>(handle_);

  connector_ = drm_->GetConnectorForDisplay(display);
  if (!connector_) {
    ALOGE("Failed to get connector for display %d", display);
    return HWC2::Error::BadDisplay;
  }

  if(connector_->raw_state() != DRM_MODE_CONNECTED){
    ALOGI("Connector %u type=%s, type_id=%d, state is DRM_MODE_DISCONNECTED, skip init.\n",
          connector_->id(),drm_->connector_type_str(connector_->type()),connector_->type_id());
    return HWC2::Error::NoResources;
  }

  UpdateDisplayMode();
  drm_->DisplayChanged();
  drm_->UpdateDisplayRoute();

  DrmCrtc *crtc = drm_->GetCrtcForDisplay(static_cast<int>(display));
  if (!crtc) {
    ALOGE("Failed to get crtc for display %d", static_cast<int>(display));
    return HWC2::Error::BadDisplay;
  }
  std::vector<DrmPlane *> display_planes;
  for (auto &plane : drm_->planes()) {
    if (plane->GetCrtcSupported(*crtc))
      display_planes.push_back(plane.get());
  }

  // Split up the given display planes into primary and overlay to properly
  // interface with the composition
  char use_overlay_planes_prop[PROPERTY_VALUE_MAX];
  property_get("hwc.drm.use_overlay_planes", use_overlay_planes_prop, "0");
  bool use_overlay_planes = atoi(use_overlay_planes_prop);
  for (auto &plane : display_planes) {
    if (plane->type() == DRM_PLANE_TYPE_PRIMARY)
      primary_planes_.push_back(plane);
    else if (use_overlay_planes && (((plane)->type() == DRM_PLANE_TYPE_OVERLAY) || (plane)->type() == DRM_PLANE_TYPE_CURSOR))
      overlay_planes_.push_back(plane);
  }

  crtc_ = drm_->GetCrtcForDisplay(display);
  if (!crtc_) {
    ALOGE("Failed to get crtc for display %d", display);
    return HWC2::Error::BadDisplay;
  }

  if(!init_success_){
    planner_ = Planner::CreateInstance(drm_);
    if (!planner_) {
      ALOGE("Failed to create planner instance for composition");
      return HWC2::Error::NoResources;
    }

    int ret = compositor_.Init(resource_manager_, display);
    if (ret) {
      ALOGE("Failed display compositor init for display %d (%d)", display, ret);
      return HWC2::Error::NoResources;
    }

    ret = vsync_worker_.Init(drm_, display);
    if (ret) {
      ALOGE("Failed to create event worker for d=%d %d\n", display, ret);
      return HWC2::Error::BadDisplay;
    }
  }

  init_success_ = true;

  return ChosePreferredConfig();
}

HWC2::Error DrmHwcTwo::HwcDisplay::ChosePreferredConfig() {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  // Fetch the number of modes from the display
  uint32_t num_configs;
  HWC2::Error err = GetDisplayConfigs(&num_configs, NULL);
  if (err != HWC2::Error::None || !num_configs)
    return err;

  return SetActiveConfig(connector_->get_preferred_mode_id());
}

HWC2::Error DrmHwcTwo::HwcDisplay::RegisterVsyncCallback(
    hwc2_callback_data_t data, hwc2_function_pointer_t func) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  auto callback = std::make_shared<DrmVsyncCallback>(data, func);
  vsync_worker_.RegisterCallback(std::move(callback));
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::AcceptDisplayChanges() {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_)
    l.second.accept_type_change();
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::CreateLayer(hwc2_layer_t *layer) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  layers_.emplace(static_cast<hwc2_layer_t>(layer_idx_), HwcLayer(layer_idx_,resource_manager_->gralloc()));
  *layer = static_cast<hwc2_layer_t>(layer_idx_);
  ++layer_idx_;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::DestroyLayer(hwc2_layer_t layer) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  layers_.erase(layer);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetActiveConfig(hwc2_config_t *config) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  DrmMode const &mode = connector_->active_mode();
  if (mode.id() == 0)
    return HWC2::Error::BadConfig;

  ctx_.framebuffer_width = mode.h_display();
  ctx_.framebuffer_height = mode.v_display();

  *config = mode.id();
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetChangedCompositionTypes(
    uint32_t *num_elements, hwc2_layer_t *layers, int32_t *types) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  uint32_t num_changes = 0;
  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_) {
    if (l.second.type_changed()) {
      if (layers && num_changes < *num_elements)
        layers[num_changes] = l.first;
      if (types && num_changes < *num_elements)
        types[num_changes] = static_cast<int32_t>(l.second.validated_type());
      ++num_changes;
    }
  }
  if (!layers && !types)
    *num_elements = num_changes;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetClientTargetSupport(uint32_t width,
                                                          uint32_t height,
                                                          int32_t /*format*/,
                                                          int32_t dataspace) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  std::pair<uint32_t, uint32_t> min = drm_->min_resolution();
  std::pair<uint32_t, uint32_t> max = drm_->max_resolution();

  if (width < min.first || height < min.second)
    return HWC2::Error::Unsupported;

  if (width > max.first || height > max.second)
    return HWC2::Error::Unsupported;

  if (dataspace != HAL_DATASPACE_UNKNOWN &&
      dataspace != HAL_DATASPACE_STANDARD_UNSPECIFIED)
    return HWC2::Error::Unsupported;

  // TODO: Validate format can be handled by either GL or planes
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetColorModes(uint32_t *num_modes,
                                                 int32_t *modes) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  if (!modes)
    *num_modes = 1;

  if (modes)
    *modes = HAL_COLOR_MODE_NATIVE;

  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetDisplayAttribute(hwc2_config_t config,
                                                       int32_t attribute_in,
                                                       int32_t *value) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  auto mode = std::find_if(connector_->modes().begin(),
                           connector_->modes().end(),
                           [config](DrmMode const &m) {
                             return m.id() == config;
                           });
  if (mode == connector_->modes().end()) {
    ALOGE("Could not find active mode for %d", config);
    return HWC2::Error::BadConfig;
  }

  static const int32_t kUmPerInch = 25400;
  uint32_t mm_width = connector_->mm_width();
  uint32_t mm_height = connector_->mm_height();
  auto attribute = static_cast<HWC2::Attribute>(attribute_in);
  switch (attribute) {
    case HWC2::Attribute::Width:
      *value = mode->h_display();
      break;
    case HWC2::Attribute::Height:
      *value = mode->v_display();
      break;
    case HWC2::Attribute::VsyncPeriod:
      // in nanoseconds
      *value = 1000 * 1000 * 1000 / mode->v_refresh();
      break;
    case HWC2::Attribute::DpiX:
      // Dots per 1000 inches
      *value = mm_width ? (mode->h_display() * kUmPerInch) / mm_width : -1;
      break;
    case HWC2::Attribute::DpiY:
      // Dots per 1000 inches
      *value = mm_height ? (mode->v_display() * kUmPerInch) / mm_height : -1;
      break;
    default:
      *value = -1;
      return HWC2::Error::BadConfig;
  }
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetDisplayConfigs(uint32_t *num_configs,
                                                     hwc2_config_t *configs) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  // Since this callback is normally invoked twice (once to get the count, and
  // once to populate configs), we don't really want to read the edid
  // redundantly. Instead, only update the modes on the first invocation. While
  // it's possible this will result in stale modes, it'll all come out in the
  // wash when we try to set the active config later.
  if (!configs) {
    int ret = connector_->UpdateModes();
    if (ret) {
      ALOGE("Failed to update display modes %d", ret);
      return HWC2::Error::BadDisplay;
    }
  }

  // Since the upper layers only look at vactive/hactive/refresh, height and
  // width, it doesn't differentiate interlaced from progressive and other
  // similar modes. Depending on the order of modes we return to SF, it could
  // end up choosing a suboptimal configuration and dropping the preferred
  // mode. To workaround this, don't offer interlaced modes to SF if there is
  // at least one non-interlaced alternative and only offer a single WxH@R
  // mode with at least the prefered mode from in DrmConnector::UpdateModes()

  // TODO: Remove the following block of code until AOSP handles all modes
  std::vector<DrmMode> sel_modes;

  // Add the preferred mode first to be sure it's not dropped
  auto mode = std::find_if(connector_->modes().begin(),
                           connector_->modes().end(), [&](DrmMode const &m) {
                             return m.id() ==
                                    connector_->get_preferred_mode_id();
                           });
  if (mode != connector_->modes().end())
    sel_modes.push_back(*mode);

  // Add the active mode if different from preferred mode
  if (connector_->active_mode().id() != connector_->get_preferred_mode_id())
    sel_modes.push_back(connector_->active_mode());

  // Cycle over the modes and filter out "similar" modes, keeping only the
  // first ones in the order given by DRM (from CEA ids and timings order)
  for (const DrmMode &mode : connector_->modes()) {
    // TODO: Remove this when 3D Attributes are in AOSP
    if (mode.flags() & DRM_MODE_FLAG_3D_MASK)
      continue;

    // TODO: Remove this when the Interlaced attribute is in AOSP
    if (mode.flags() & DRM_MODE_FLAG_INTERLACE) {
      auto m = std::find_if(connector_->modes().begin(),
                            connector_->modes().end(),
                            [&mode](DrmMode const &m) {
                              return !(m.flags() & DRM_MODE_FLAG_INTERLACE) &&
                                     m.h_display() == mode.h_display() &&
                                     m.v_display() == mode.v_display();
                            });
      if (m == connector_->modes().end())
        sel_modes.push_back(mode);

      continue;
    }

    // Search for a similar WxH@R mode in the filtered list and drop it if
    // another mode with the same WxH@R has already been selected
    // TODO: Remove this when AOSP handles duplicates modes
    auto m = std::find_if(sel_modes.begin(), sel_modes.end(),
                          [&mode](DrmMode const &m) {
                            return m.h_display() == mode.h_display() &&
                                   m.v_display() == mode.v_display() &&
                                   m.v_refresh() == mode.v_refresh();
                          });
    if (m == sel_modes.end())
      sel_modes.push_back(mode);
  }

  auto num_modes = static_cast<uint32_t>(sel_modes.size());
  if (!configs) {
    *num_configs = num_modes;
    return HWC2::Error::None;
  }



  uint32_t idx = 0;
  for (const DrmMode &mode : sel_modes) {
    if (idx >= *num_configs)
      break;
    configs[idx++] = mode.id();
  }

  sf_modes_.swap(sel_modes);

  *num_configs = idx;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetDisplayName(uint32_t *size, char *name) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  std::ostringstream stream;
  stream << "display-" << connector_->id();
  std::string string = stream.str();
  size_t length = string.length();
  if (!name) {
    *size = length;
    return HWC2::Error::None;
  }

  *size = std::min<uint32_t>(static_cast<uint32_t>(length - 1), *size);
  strncpy(name, string.c_str(), *size);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetDisplayRequests(int32_t *display_requests,
                                                      uint32_t *num_elements,
                                                      hwc2_layer_t *layers,
                                                      int32_t *layer_requests) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  // TODO: I think virtual display should request
  //      HWC2_DISPLAY_REQUEST_WRITE_CLIENT_TARGET_TO_OUTPUT here
  unsupported(__func__, display_requests, num_elements, layers, layer_requests);
  *num_elements = 0;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetDisplayType(int32_t *type) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  *type = static_cast<int32_t>(type_);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetDozeSupport(int32_t *support) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  *support = 0;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetHdrCapabilities(
    uint32_t *num_types, int32_t *types, float * max_luminance,
    float *max_average_luminance, float * min_luminance) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);

  int display = static_cast<int>(handle_);
  int HdrIndex = 0;

    if (!connector_) {
    ALOGE("%s:Failed to get connector for display %d line=%d", __FUNCTION__,display,__LINE__);
    return HWC2::Error::None;
  }
  int ret = connector_->UpdateModes();
  if (ret) {
    ALOGE("Failed to update display modes %d", ret);
    return HWC2::Error::None;
  }
  const std::vector<DrmHdr> hdr_support_list = connector_->get_hdr_support_list();

  if(types == NULL){
      *num_types = hdr_support_list.size();
      return HWC2::Error::None;
  }

  for(const DrmHdr &hdr_mode : hdr_support_list){
      types[HdrIndex] = hdr_mode.drmHdrType;
      *max_luminance = hdr_mode.outMaxLuminance;
      *max_average_luminance = hdr_mode.outMaxAverageLuminance;
      *min_luminance = hdr_mode.outMinLuminance;
      HdrIndex++;
  }
  *num_types = hdr_support_list.size();

  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::GetReleaseFences(uint32_t *num_elements,
                                                    hwc2_layer_t *layers,
                                                    int32_t *fences) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);

  uint32_t num_layers = 0;

  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_) {
    ++num_layers;
    if (layers == NULL || fences == NULL) {
      continue;
    } else if (num_layers > *num_elements) {
      ALOGW("Overflow num_elements %d/%d", num_layers, *num_elements);
      return HWC2::Error::None;
    }

    layers[num_layers - 1] = l.first;
    fences[num_layers - 1] = l.second.take_release_fence();
    ALOGV("rk-debug GetReleaseFences [%" PRIu64 "][%d]",layers[num_layers - 1],fences[num_layers - 1]);
  }
  *num_elements = num_layers;
  return HWC2::Error::None;
}

void DrmHwcTwo::HwcDisplay::AddFenceToRetireFence(int fd) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);

  if (fd < 0){
    for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_) {
      l.second.manage_release_fence();

      int releaseFenceFd = l.second.release_fence();

      if (releaseFenceFd < 0)
        continue;

      if (retire_fence_.get() >= 0) {
        int old_retire_fence = retire_fence_.get();
        retire_fence_.Set(sync_merge("dc_retire", old_retire_fence, releaseFenceFd));
      } else {
        retire_fence_.Set(dup(releaseFenceFd));
      }
    }
    client_layer_.manage_release_fence();
    if(client_layer_.release_fence() > 0){
      int releaseFenceFd = client_layer_.take_release_fence();
      if(retire_fence_.get() >= 0){
          int old_retire_fence = retire_fence_.get();
          retire_fence_.Set(sync_merge("dc_retire", old_retire_fence, releaseFenceFd));
      }else{
          retire_fence_.Set(dup(releaseFenceFd));
      }
      close(releaseFenceFd);
    }
    ALOGV("rk-debug AddFenceToRetireFence [%d]",retire_fence_.get());
    return;
  }else{
    if (retire_fence_.get() >= 0) {
      int old_fence = retire_fence_.get();
      retire_fence_.Set(sync_merge("dc_retire", old_fence, fd));
    } else {
      retire_fence_.Set(dup(fd));
    }
  }
}

bool SortByZpos(const DrmHwcLayer &drmHwcLayer1, const DrmHwcLayer &drmHwcLayer2){
    return drmHwcLayer1.iZpos_ <= drmHwcLayer2.iZpos_;
}
HWC2::Error DrmHwcTwo::HwcDisplay::InitDrmHwcLayer() {
  drm_hwc_layers_.clear();

  // now that they're ordered by z, add them to the composition
  for (auto &hwc2layer : layers_) {
    DrmHwcLayer drmHwclayer;
    hwc2layer.second.PopulateDrmLayer(hwc2layer.first, &drmHwclayer, &ctx_, frame_no_, false);
    drm_hwc_layers_.emplace_back(std::move(drmHwclayer));
  }

  std::sort(drm_hwc_layers_.begin(),drm_hwc_layers_.end(),SortByZpos);

  uint32_t client_id = UINT32_MAX;
  DrmHwcLayer client_target_layer;
  client_layer_.PopulateDrmLayer(client_id, &client_target_layer, &ctx_, frame_no_, true);
  drm_hwc_layers_.emplace_back(std::move(client_target_layer));

  ALOGD_HWC2_DRM_LAYER_INFO((DBG_DEBUG),drm_hwc_layers_);

  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::ValidatePlanes() {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  int ret;

  InitDrmHwcLayer();

  // First to try GLES all layer
  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_)
    l.second.set_validated_type(HWC2::Composition::Client);

  std::map<size_t, DrmHwcLayer *> to_composite;
  for(size_t i = 0; i < drm_hwc_layers_.size(); ++i){
    if(drm_hwc_layers_[i].bFbTarget_)
      to_composite.emplace(std::make_pair(0, &drm_hwc_layers_[i]));
  }

  std::vector<DrmPlane *> primary_planes(primary_planes_);
  std::vector<DrmPlane *> overlay_planes(overlay_planes_);
  std::tie(ret,
           composition_planes_) = planner_->MatchPlanes(to_composite, crtc_,
                                                            &primary_planes,
                                                            &overlay_planes);
  if (ret){
    ALOGE("First, GLES policy fail ret=%d", ret);
    return HWC2::Error::BadConfig;
  }

  for (auto &drm_hwc_layer : drm_hwc_layers_) {
    auto l = layers_.find(drm_hwc_layer.uId_);
    HWC2::Composition comp_type;
    comp_type = l->second.validated_type();
    switch (comp_type) {
      case HWC2::Composition::Device:
        ALOGV("rk-debug ValidatePlanes layer id = %" PRIu32 ",comp_type = Device,line = %d",drm_hwc_layer.uId_,__LINE__);
        break;
      case HWC2::Composition::Client:
        ALOGV("rk-debug ValidatePlanes layer id = %" PRIu32 ",comp_type = Client,line = %d",drm_hwc_layer.uId_,__LINE__);
        break;
      default:
        continue;
    }
  }

  return HWC2::Error::None;
}


HWC2::Error DrmHwcTwo::HwcDisplay::CreateComposition() {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  int ret;
  std::vector<DrmCompositionDisplayLayersMap> layers_map;
  layers_map.emplace_back();
  DrmCompositionDisplayLayersMap &map = layers_map.back();

  map.display = static_cast<int>(handle_);
  map.geometry_changed = true;  // TODO: Fix this

  bool use_client_layer = false;
  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_)
    if(l.second.sf_type() == HWC2::Composition::Client)
      use_client_layer = true;
  if(use_client_layer){
    for (auto &drm_hwc_layer : drm_hwc_layers_) {
      if(drm_hwc_layer.bFbTarget_){
        uint32_t client_id = UINT32_MAX;
        client_layer_.PopulateDrmLayer(client_id, &drm_hwc_layer, &ctx_, frame_no_, true);
      }
    }
  }

  for (auto &drm_hwc_layer : drm_hwc_layers_) {
    auto l = layers_.find(drm_hwc_layer.uId_);
    HWC2::Composition comp_type;
    comp_type = l->second.validated_type();
    switch (comp_type) {
      case HWC2::Composition::Device:
        ret = drm_hwc_layer.ImportBuffer(importer_.get());
        if (ret) {
          ALOGE("Failed to import layer, ret=%d", ret);
          return HWC2::Error::NoResources;
        }
        map.layers.emplace_back(std::move(drm_hwc_layer));
        ALOGV("rk-debug ValidatePlanes layer id = %" PRIu32 ", comp_type = Device,line = %d",drm_hwc_layer.uId_,__LINE__);
        break;
      case HWC2::Composition::Client:
        ALOGV("rk-debug ValidatePlanes layer id = %" PRIu32 ", comp_type = Client,line = %d",drm_hwc_layer.uId_,__LINE__);
        break;
      default:
        ret = drm_hwc_layer.ImportBuffer(importer_.get());
        if (ret) {
          ALOGE("Failed to import layer, ret=%d", ret);
          return HWC2::Error::NoResources;
        }
        map.layers.emplace_back(std::move(drm_hwc_layer));
        continue;
    }
  }

  std::unique_ptr<DrmDisplayComposition> composition = compositor_
                                                         .CreateComposition();
  composition->Init(drm_, crtc_, importer_.get(), planner_.get(), frame_no_);

  // TODO: Don't always assume geometry changed
  ret = composition->SetLayers(map.layers.data(), map.layers.size(), true);
  if (ret) {
    ALOGE("Failed to set layers in the composition ret=%d", ret);
    return HWC2::Error::BadLayer;
  }
  for(auto &composition_plane :composition_planes_)
    ret = composition->AddPlaneComposition(std::move(composition_plane));

  std::vector<DrmPlane *> primary_planes(primary_planes_);
  std::vector<DrmPlane *> overlay_planes(overlay_planes_);
  ret = composition->Plan(&primary_planes, &overlay_planes);
  if (ret) {
    ALOGE("Failed to plan the composition ret=%d", ret);
    return HWC2::Error::BadConfig;
  }

  // Disable the planes we're not using
  for (auto i = primary_planes.begin(); i != primary_planes.end();) {
    composition->AddPlaneDisable(*i);
    i = primary_planes.erase(i);
  }
  for (auto i = overlay_planes.begin(); i != overlay_planes.end();) {
    composition->AddPlaneDisable(*i);
    i = overlay_planes.erase(i);
  }

  ret = composition->CreateAndAssignReleaseFences();
  AddFenceToRetireFence(composition->take_out_fence());
  ret = compositor_.QueueComposition(std::move(composition));

  return HWC2::Error::None;
}



HWC2::Error DrmHwcTwo::HwcDisplay::PresentDisplay(int32_t *retire_fence) {
  ATRACE_CALL();

  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);

  HWC2::Error ret;

  ret = CreateComposition();
  if (ret == HWC2::Error::BadLayer) {
    // Can we really have no client or device layers?
    *retire_fence = -1;
    return HWC2::Error::None;
  }
  if (ret != HWC2::Error::None)
    return ret;

  *retire_fence = retire_fence_.Release();

  ++frame_no_;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetActiveConfig(hwc2_config_t config) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  auto mode = std::find_if(connector_->modes().begin(),
                           connector_->modes().end(),
                           [config](DrmMode const &m) {
                             return m.id() == config;
                           });
  if (mode == connector_->modes().end()) {
    ALOGE("Could not find active mode for %d", config);
    return HWC2::Error::BadConfig;
  }

//  std::unique_ptr<DrmDisplayComposition> composition = compositor_
//                                                           .CreateComposition();
//  composition->Init(drm_, crtc_, importer_.get(), planner_.get(), frame_no_);
//  int ret = composition->SetDisplayMode(*mode);
//  ret = compositor_.QueueComposition(std::move(composition));
//  if (ret) {
//    ALOGE("Failed to queue dpms composition on %d", ret);
//    return HWC2::Error::BadConfig;
//  }

  connector_->set_active_mode(*mode);

  // Setup the client layer's dimensions
  hwc_rect_t display_frame = {.left = 0,
                              .top = 0,
                              .right = static_cast<int>(mode->h_display()),
                              .bottom = static_cast<int>(mode->v_display())};
  client_layer_.SetLayerDisplayFrame(display_frame);
  hwc_frect_t source_crop = {.left = 0.0f,
                             .top = 0.0f,
                             .right = mode->h_display() + 0.0f,
                             .bottom = mode->v_display() + 0.0f};
  client_layer_.SetLayerSourceCrop(source_crop);

  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetClientTarget(buffer_handle_t target,
                                                   int32_t acquire_fence,
                                                   int32_t dataspace,
                                                   hwc_region_t /*damage*/) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  UniqueFd uf(acquire_fence);

  client_layer_.set_buffer(target);
  client_layer_.set_acquire_fence(uf.get());
  client_layer_.SetLayerDataspace(dataspace);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetColorMode(int32_t mode) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);

  if (mode != HAL_COLOR_MODE_NATIVE)
    return HWC2::Error::Unsupported;

  color_mode_ = mode;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetColorTransform(const float *matrix,
                                                     int32_t hint) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  // TODO: Force client composition if we get this
  return unsupported(__func__, matrix, hint);
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetOutputBuffer(buffer_handle_t buffer,
                                                   int32_t release_fence) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  // TODO: Need virtual display support
  return unsupported(__func__, buffer, release_fence);
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetPowerMode(int32_t mode_in) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);

  uint64_t dpms_value = 0;
  auto mode = static_cast<HWC2::PowerMode>(mode_in);
  switch (mode) {
    case HWC2::PowerMode::Off:
      dpms_value = DRM_MODE_DPMS_OFF;
      break;
    case HWC2::PowerMode::On:
      dpms_value = DRM_MODE_DPMS_ON;
      break;
    default:
      ALOGI("Power mode %d is unsupported\n", mode);
      return HWC2::Error::Unsupported;
  };

  std::unique_ptr<DrmDisplayComposition> composition = compositor_
                                                           .CreateComposition();
  composition->Init(drm_, crtc_, importer_.get(), planner_.get(), frame_no_);
  composition->SetDpmsMode(dpms_value);
  int ret = compositor_.QueueComposition(std::move(composition));
  if (ret) {
    ALOGE("Failed to apply the dpms composition ret=%d", ret);
    return HWC2::Error::BadParameter;
  }
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::SetVsyncEnabled(int32_t enabled) {
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  vsync_worker_.VSyncControl(HWC2_VSYNC_ENABLE == enabled);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcDisplay::ValidateDisplay(uint32_t *num_types,
                                                   uint32_t *num_requests) {
  ATRACE_CALL();
  ALOGD_HWC2_DISPLAY_INFO(DBG_VERBOSE,handle_);
  // Enable/disable debug log
  UpdateLogLevel();
  UpdateDisplayMode();
  drm_->UpdateDisplayRoute();

  *num_types = 0;
  *num_requests = 0;

  HWC2::Error ret;

  if(LogLevel(DBG_DEBUG)){
    String8 out;
    DumpDisplayLayersInfo(out);
    ALOGD("%s",out.string());

  }

  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_)
    l.second.set_validated_type(HWC2::Composition::Invalid);

  ret = ValidatePlanes();
  if (ret != HWC2::Error::None){
    ALOGE("%s fail , ret = %d,line = %d",__FUNCTION__,ret,__LINE__);
    return HWC2::Error::BadConfig;
  }

  for (std::pair<const hwc2_layer_t, DrmHwcTwo::HwcLayer> &l : layers_) {
    DrmHwcTwo::HwcLayer &layer = l.second;
    // We can only handle layers of Device type, send everything else to SF
    if (layer.sf_type() != HWC2::Composition::Device ||
        layer.validated_type() != HWC2::Composition::Device) {
      layer.set_validated_type(HWC2::Composition::Client);
      ++*num_types;
    }
  }
  return *num_types ? HWC2::Error::HasChanges : HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetCursorPosition(int32_t x, int32_t y) {
  cursor_x_ = x;
  cursor_y_ = y;
  return HWC2::Error::None;
}

int DrmHwcTwo::HwcDisplay::DumpDisplayInfo(String8 &output){

  output.appendFormat(" DisplayId=%" PRIu64 ", Connector %u, Type = %s-%u, Connector state = %s\n",handle_,
                        connector_->id(),drm_->connector_type_str(connector_->type()),connector_->type_id(),
                        connector_->raw_state() == DRM_MODE_CONNECTED ? "DRM_MODE_CONNECTED" : "DRM_MODE_DISCONNECTED");

  if(connector_->raw_state() != DRM_MODE_CONNECTED)
    return -1;

  DrmMode const &active_mode = connector_->active_mode();
  if (active_mode.id() == 0){
    return -1;
  }

  output.appendFormat("  NumHwLayers=%zu, activeModeId=%u, %s%c%.2f, colorMode = %d\n",
                        get_layers().size(),
                        active_mode.id(), active_mode.name().c_str(),'p' ,active_mode.v_refresh(),
                        color_mode_);
  uint32_t idx = 0;
  if(sf_modes_.size() > 0){
    for (const DrmMode &mode : sf_modes_) {
      if(active_mode.id() == mode.id())
        output.appendFormat("    Config[%2u] = %s%c%.2f (active)\n",idx, mode.name().c_str(), 'p' , mode.v_refresh());
      else
        output.appendFormat("    Config[%2u] = %s%c%.2f\n",idx, mode.name().c_str(), 'p' , mode.v_refresh());
      idx++;
    }
  }

  output.append(
              "------+-----------+--------------+-------------+------------+--------------------------------+------------------------+------\n"
              "  id  |    type   |    handle    |  transform  |    blnd    |     source crop (l,t,r,b)      |          frame         | dataspace \n"
              "------+-----------+--------------+-------------+------------+--------------------------------+------------------------+------\n");
  for (auto &map_layer : layers_) {
      HwcLayer &layer = map_layer.second;
      layer.DumpLayerInfo(output);
  }
  output.append("------+-----------+--------------+-------------+------------+--------------------------------+------------------------+------\n");
  output.append("DrmHwcLayer Dump:\n");

  for(auto &drmHwcLayer : drm_hwc_layers_)
      drmHwcLayer.DumpInfo(output);

  return 0;
}

int DrmHwcTwo::HwcDisplay::DumpDisplayLayersInfo(String8 &output){

  output.appendFormat(" DisplayId=%" PRIu64 ", Connector %u, Type = %s-%u, Connector state = %s , frame_no = %d\n",handle_,
                        connector_->id(),drm_->connector_type_str(connector_->type()),connector_->type_id(),
                        connector_->raw_state() == DRM_MODE_CONNECTED ? "DRM_MODE_CONNECTED" : "DRM_MODE_DISCONNECTED",
                        frame_no_);

  output.append(
              "------+-----------+--------------+-------------+------------+--------------------------------+------------------------+------\n"
              "  id  |    type   |    handle    |  transform  |    blnd    |     source crop (l,t,r,b)      |          frame         | dataspace \n"
              "------+-----------+--------------+-------------+------------+--------------------------------+------------------------+------\n");
  for (auto &map_layer : layers_) {
      HwcLayer &layer = map_layer.second;
      layer.DumpLayerInfo(output);
  }
  output.append("------+-----------+--------------+-------------+------------+--------------------------------+------------------------+------\n");
  return 0;
}

int DrmHwcTwo::HwcDisplay::GetBestDisplayMode(){
  char resolution[PROPERTY_VALUE_MAX];
  uint32_t width, height, flags;
  uint32_t hsync_start, hsync_end, htotal;
  uint32_t vsync_start, vsync_end, vtotal;
  bool interlaced;
  float vrefresh;
  char val;
  uint32_t MaxResolution = 0,temp;

  int display = static_cast<int>(handle_);

  if(display == HWC_DISPLAY_PRIMARY)
    property_get("persist.vendor.resolution.main", resolution, "Auto");
  else
    property_get("persist.vendor.resolution.aux", resolution, "Auto");

  if(strcmp(resolution,"Auto") != 0){
    int len = sscanf(resolution, "%dx%d@%f-%d-%d-%d-%d-%d-%d-%x",
                     &width, &height, &vrefresh, &hsync_start,
                     &hsync_end, &htotal, &vsync_start,&vsync_end,
                     &vtotal, &flags);
    if (len == 10 && width != 0 && height != 0) {
      for (const DrmMode &conn_mode : connector_->modes()) {
        if (conn_mode.equal(width, height, vrefresh, hsync_start, hsync_end,
                            htotal, vsync_start, vsync_end, vtotal, flags)) {
          connector_->set_best_mode(conn_mode);
          return 0;
        }
      }
    }

    uint32_t ivrefresh;
    len = sscanf(resolution, "%dx%d%c%d", &width, &height, &val, &ivrefresh);

    if (val == 'i')
      interlaced = true;
    else
      interlaced = false;
    if (len == 4 && width != 0 && height != 0) {
      for (const DrmMode &conn_mode : connector_->modes()) {
        if (conn_mode.equal(width, height, ivrefresh, interlaced)) {
          connector_->set_best_mode(conn_mode);
          return 0;
        }
      }
    }
  }

  for (const DrmMode &conn_mode : connector_->modes()) {
    if (conn_mode.type() & DRM_MODE_TYPE_PREFERRED) {
      connector_->set_best_mode(conn_mode);
      return 0;
    }
    else {
      temp = conn_mode.h_display()*conn_mode.v_display();
      if(MaxResolution <= temp)
        MaxResolution = temp;
    }
  }
  for (const DrmMode &conn_mode : connector_->modes()) {
    if(MaxResolution == conn_mode.h_display()*conn_mode.v_display()) {
      connector_->set_best_mode(conn_mode);
      return 0;
    }
  }

  //use raw modes to get mode.
  for (const DrmMode &conn_mode : connector_->raw_modes()) {
    if (conn_mode.type() & DRM_MODE_TYPE_PREFERRED) {
      connector_->set_best_mode(conn_mode);
      return 0;
    }
    else {
      temp = conn_mode.h_display()*conn_mode.v_display();
      if(MaxResolution <= temp)
        MaxResolution = temp;
    }
  }
  for (const DrmMode &conn_mode : connector_->raw_modes()) {
    if(MaxResolution == conn_mode.h_display()*conn_mode.v_display()) {
      connector_->set_best_mode(conn_mode);
      return 0;
    }
  }

  ALOGE("Error: Should not get here display=%d %s %d\n", display, __FUNCTION__, __LINE__);
  DrmMode mode;
  connector_->set_best_mode(mode);

  return -ENOENT;
}

int DrmHwcTwo::HwcDisplay::UpdateDisplayMode(){
  int timeline;

  timeline = property_get_int32("vendor.display.timeline", -1);
  if(timeline && timeline == ctx_.display_timeline && ctx_.hotplug_timeline == drm_->timeline())
    return 0;
  ctx_.display_timeline = timeline;
  ctx_.hotplug_timeline = drm_->timeline();

  int ret = GetBestDisplayMode();
  if(!ret){
    const DrmMode best_mode = connector_->best_mode();
    connector_->set_current_mode(best_mode);
    ctx_.rel_xres = best_mode.h_display();
    ctx_.rel_yres = best_mode.v_display();
  }
  return 0;
}


HWC2::Error DrmHwcTwo::HwcLayer::SetLayerBlendMode(int32_t mode) {
  ALOGD_HWC2_LAYER_INFO(DBG_VERBOSE, id_);
  blending_ = static_cast<HWC2::BlendMode>(mode);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerBuffer(buffer_handle_t buffer,
                                                int32_t acquire_fence) {
  ALOGD_HWC2_LAYER_INFO(DBG_VERBOSE, id_);
  UniqueFd uf(acquire_fence);

  // The buffer and acquire_fence are handled elsewhere
  if (sf_type_ == HWC2::Composition::Client ||
      sf_type_ == HWC2::Composition::Sideband ||
      sf_type_ == HWC2::Composition::SolidColor)
    return HWC2::Error::None;

  set_buffer(buffer);
  set_acquire_fence(uf.get());
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerColor(hwc_color_t color) {
  ALOGD_HWC2_LAYER_INFO(DBG_VERBOSE, id_);
  // TODO: Punt to client composition here?
  return unsupported(__func__, color);
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerCompositionType(int32_t type) {
  ALOGD_HWC2_LAYER_INFO(DBG_VERBOSE, id_);
  sf_type_ = static_cast<HWC2::Composition>(type);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerDataspace(int32_t dataspace) {
  ALOGD_HWC2_LAYER_INFO(DBG_VERBOSE, id_);
  dataspace_ = static_cast<android_dataspace_t>(dataspace);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerDisplayFrame(hwc_rect_t frame) {
  ALOGD_HWC2_LAYER_INFO(DBG_VERBOSE, id_);
  display_frame_ = frame;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerPlaneAlpha(float alpha) {
  ALOGD_HWC2_LAYER_INFO(DBG_VERBOSE, id_);
  alpha_ = alpha;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerSidebandStream(
    const native_handle_t *stream) {
  ALOGD_HWC2_LAYER_INFO(DBG_VERBOSE, id_);
  // TODO: We don't support sideband
  return unsupported(__func__, stream);
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerSourceCrop(hwc_frect_t crop) {
  ALOGD_HWC2_LAYER_INFO(DBG_VERBOSE, id_);
  source_crop_ = crop;
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerSurfaceDamage(hwc_region_t damage) {
  ALOGD_HWC2_LAYER_INFO(DBG_VERBOSE, id_);
  // TODO: We don't use surface damage, marking as unsupported
  unsupported(__func__, damage);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerTransform(int32_t transform) {
  ALOGD_HWC2_LAYER_INFO(DBG_VERBOSE, id_);
  transform_ = static_cast<HWC2::Transform>(transform);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerVisibleRegion(hwc_region_t visible) {
  ALOGD_HWC2_LAYER_INFO(DBG_VERBOSE, id_);
  // TODO: We don't use this information, marking as unsupported
  unsupported(__func__, visible);
  return HWC2::Error::None;
}

HWC2::Error DrmHwcTwo::HwcLayer::SetLayerZOrder(uint32_t order) {
  ALOGD_HWC2_LAYER_INFO(DBG_VERBOSE, id_);
  z_order_ = order;
  return HWC2::Error::None;
}

void DrmHwcTwo::HwcLayer::PopulateDrmLayer(hwc2_layer_t layer_id, DrmHwcLayer *drmHwcLayer,
                                                 hwc_drm_display_t* ctx, uint32_t frame_no, bool client_layer) {
  drmHwcLayer->uId_ = layer_id;
  drmHwcLayer->iZpos_ = z_order_;
  drmHwcLayer->uFrameNo_ = frame_no;
  drmHwcLayer->bFbTarget_ = client_layer;

  switch (blending_) {
    case HWC2::BlendMode::None:
      drmHwcLayer->blending = DrmHwcBlending::kNone;
      break;
    case HWC2::BlendMode::Premultiplied:
      drmHwcLayer->blending = DrmHwcBlending::kPreMult;
      break;
    case HWC2::BlendMode::Coverage:
      drmHwcLayer->blending = DrmHwcBlending::kCoverage;
      break;
    default:
      ALOGE("Unknown blending mode b=%d", blending_);
      drmHwcLayer->blending = DrmHwcBlending::kNone;
      break;
  }

  OutputFd release_fence = release_fence_output();
  drmHwcLayer->sf_handle = buffer_;
  drmHwcLayer->acquire_fence = acquire_fence_.Release();
  drmHwcLayer->release_fence = std::move(release_fence);

  float w_scale = ctx->rel_xres / (float)ctx->framebuffer_width;
  float h_scale = ctx->rel_yres / (float)ctx->framebuffer_height;

  hwc_rect_t display_frame;
  display_frame.left = (int)(display_frame_.left * w_scale);
  display_frame.right = (int)(display_frame_.right * w_scale);
  display_frame.top = (int)(display_frame_.top * h_scale);
  display_frame.bottom = (int)(display_frame_.bottom * h_scale);

  drmHwcLayer->SetDisplayFrame(display_frame);
  drmHwcLayer->alpha = static_cast<uint16_t>(255.0f * alpha_ + 0.5f);
  drmHwcLayer->SetSourceCrop(source_crop_);
  drmHwcLayer->SetTransform(static_cast<int32_t>(transform_));

  if(buffer_){
    drmHwcLayer->iFd_ = hwc_get_handle_primefd(gralloc_, buffer_);
    drmHwcLayer->iWidth_ = hwc_get_handle_attibute(gralloc_,buffer_,ATT_WIDTH);
    drmHwcLayer->iHeight_ = hwc_get_handle_attibute(gralloc_,buffer_,ATT_HEIGHT);
    drmHwcLayer->iStride_ = hwc_get_handle_attibute(gralloc_,buffer_,ATT_STRIDE);
    drmHwcLayer->iFormat_ = hwc_get_handle_attibute(gralloc_,buffer_,ATT_FORMAT);
    drmHwcLayer->iBpp_ = android::bytesPerPixel(drmHwcLayer->iFormat_);
  }else{
    drmHwcLayer->iFd_ = -1;
    drmHwcLayer->iWidth_ = -1;
    drmHwcLayer->iHeight_ = -1;
    drmHwcLayer->iStride_ = -1;
    drmHwcLayer->iFormat_ = -1;
    drmHwcLayer->iBpp_ = -1;
  }
  drmHwcLayer->Init();

}

void DrmHwcTwo::HwcLayer::DumpLayerInfo(String8 &output) {

  output.appendFormat( " %04" PRIu32 " | %9s | %012" PRIxPTR " | %-11.11s | %-10.10s |%7.1f,%7.1f,%7.1f,%7.1f |%5d,%5d,%5d,%5d | %x\n",
                    id_,to_string(validated_type_).c_str(),
                    intptr_t(buffer_), to_string(transform_).c_str(), to_string(blending_).c_str(),
                    source_crop_.left, source_crop_.top, source_crop_.right, source_crop_.bottom,
                    display_frame_.left, display_frame_.top, display_frame_.right, display_frame_.bottom,
                    dataspace_);
  return;
}

void DrmHwcTwo::HandleDisplayHotplug(hwc2_display_t displayid, int state) {
  auto cb = callbacks_.find(HWC2::Callback::Hotplug);
  if (cb == callbacks_.end())
    return;

  auto hotplug = reinterpret_cast<HWC2_PFN_HOTPLUG>(cb->second.func);
  hotplug(cb->second.data, displayid,
          (state == DRM_MODE_CONNECTED ? HWC2_CONNECTION_CONNECTED
                                       : HWC2_CONNECTION_DISCONNECTED));
}

void DrmHwcTwo::HandleInitialHotplugState(DrmDevice *drmDevice) {
    for (auto &conn : drmDevice->connectors()) {
      if (conn->state() != DRM_MODE_CONNECTED)
        continue;
      for (auto &crtc : drmDevice->crtc()) {
        if(conn->display() != crtc->display())
          continue;
        ALOGI("HWC2 Init: SF register connector %u type=%s, type_id=%d \n",
          conn->id(),drmDevice->connector_type_str(conn->type()),conn->type_id());
        HandleDisplayHotplug(conn->display(), conn->state());
      }
    }
}

void DrmHwcTwo::DrmHotplugHandler::HandleEvent(uint64_t timestamp_us) {

  DrmConnector *extend = NULL;
  DrmConnector *primary = NULL;

  for (auto &conn : drm_->connectors()) {
    drmModeConnection old_state = conn->raw_state();
    drmModeConnection cur_state = conn->UpdateModes()
                                      ? DRM_MODE_UNKNOWNCONNECTION
                                      : conn->raw_state();

    if (cur_state == old_state)
      continue;

    ALOGI("hwc_hotplug: %s event @%" PRIu64 " for connector %u type=%s, type_id=%d\n",
          cur_state == DRM_MODE_CONNECTED ? "Plug" : "Unplug", timestamp_us,
          conn->id(),drm_->connector_type_str(conn->type()),conn->type_id());

    if (cur_state == DRM_MODE_CONNECTED) {
      if (conn->possible_displays() & HWC_DISPLAY_EXTERNAL_BIT){
        ALOGD("hwc_hotplug: find the first connect external type=%s(%d)",
          drm_->connector_type_str(conn->type()), conn->type_id());
        extend = conn.get();
      }
      else if (conn->possible_displays() & HWC_DISPLAY_PRIMARY_BIT){
        ALOGD("hwc_hotplug: find the first connect primary type=%s(%d)",
          drm_->connector_type_str(conn->type()), conn->type_id());
        primary = conn.get();
      }
    }
  }

  drm_->DisplayChanged();

  DrmConnector *old_primary = drm_->GetConnectorFromType(HWC_DISPLAY_PRIMARY);
  primary = primary ? primary : old_primary;
  if (!primary || primary->raw_state() != DRM_MODE_CONNECTED) {
    primary = NULL;
    for (auto &conn : drm_->connectors()) {
      if (!(conn->possible_displays() & HWC_DISPLAY_PRIMARY_BIT))
        continue;
      if (conn->raw_state() == DRM_MODE_CONNECTED) {
        primary = conn.get();
        ALOGD("hwc_hotplug: find the second connect primary type=%s(%d)",
          drm_->connector_type_str(conn->type()), conn->type_id());
        break;
      }
    }
  }

  if (!primary) {
    for (auto &conn : drm_->connectors()) {
      if (!(conn->possible_displays() & HWC_DISPLAY_PRIMARY_BIT))
        continue;
      ALOGD("hwc_hotplug: find the third primary type=%s(%d)",
          drm_->connector_type_str(conn->type()), conn->type_id());
      primary = conn.get();
    }
  }

  if (!primary) {
    ALOGE("hwc_hotplug: %s %d Failed to find primary display\n", __FUNCTION__, __LINE__);
    return;
  }

  if (primary != old_primary) {
    drm_->SetPrimaryDisplay(primary);
    int display_id = primary->display();
    if (primary->raw_state() == DRM_MODE_CONNECTED) {
      auto &display = hwc2_->displays_.at(display_id);
      display.ChosePreferredConfig();
    }else{
      auto &display = hwc2_->displays_.at(display_id);
      display.ClearDisplay();
    }
    return;

  }

  DrmConnector *old_extend = drm_->GetConnectorFromType(HWC_DISPLAY_EXTERNAL);
  extend = extend ? extend : old_extend;
  if (!extend || extend->raw_state() != DRM_MODE_CONNECTED) {
    extend = NULL;
    for (auto &conn : drm_->connectors()) {
      if (!(conn->possible_displays() & HWC_DISPLAY_EXTERNAL_BIT))
        continue;
      if (conn->id() == primary->id())
        continue;
      if (conn->raw_state() == DRM_MODE_CONNECTED) {
        extend = conn.get();
        ALOGD("hwc_hotplug: find the second connect external type=%s(%d)",
          drm_->connector_type_str(conn->type()), conn->type_id());
        break;
      }
    }
  }
  drm_->SetExtendDisplay(extend);
  if (!extend) {
    if(old_extend){
      int display_id = old_extend->display();
      auto &display = hwc2_->displays_.at(display_id);
      drm_->UpdateDisplayRoute();
      display.ClearDisplay();
      while(display.PresentFinish()){usleep(2*1000);}
      hwc2_->HandleDisplayHotplug(display_id, DRM_MODE_DISCONNECTED);
    }else{
      return;
    }
  }else{
    if(!old_extend){
      int display_id = extend->display();
      auto &display = hwc2_->displays_.at(display_id);
      drm_->UpdateDisplayRoute();
      display.Init();
      while(display.PresentFinish()){usleep(2*1000);}
      hwc2_->HandleDisplayHotplug(display_id, DRM_MODE_CONNECTED);
    }else{
      int display_id = old_extend->display();
      auto &display_old = hwc2_->displays_.at(display_id);
      while(display_old.PresentFinish()){usleep(2*1000);}
      hwc2_->HandleDisplayHotplug(display_id, DRM_MODE_DISCONNECTED);
      usleep(200 * 1000);
      display_id = extend->display();
      auto &display = hwc2_->displays_.at(display_id);
      drm_->UpdateDisplayRoute();
      display.Init();
      while(display.PresentFinish()){usleep(2*1000);}
      hwc2_->HandleDisplayHotplug(display_id, DRM_MODE_CONNECTED);
    }
  }
  return;
}

// static
int DrmHwcTwo::HookDevClose(hw_device_t * /*dev*/) {
  unsupported(__func__);
  return 0;
}

// static
void DrmHwcTwo::HookDevGetCapabilities(hwc2_device_t * /*dev*/,
                                       uint32_t *out_count,
                                       int32_t * /*out_capabilities*/) {
  supported(__func__);
  *out_count = 0;
}

// static
hwc2_function_pointer_t DrmHwcTwo::HookDevGetFunction(
    struct hwc2_device * /*dev*/, int32_t descriptor) {
  supported(__func__);
  auto func = static_cast<HWC2::FunctionDescriptor>(descriptor);
  switch (func) {
    // Device functions
    case HWC2::FunctionDescriptor::CreateVirtualDisplay:
      return ToHook<HWC2_PFN_CREATE_VIRTUAL_DISPLAY>(
          DeviceHook<int32_t, decltype(&DrmHwcTwo::CreateVirtualDisplay),
                     &DrmHwcTwo::CreateVirtualDisplay, uint32_t, uint32_t,
                     int32_t *, hwc2_display_t *>);
    case HWC2::FunctionDescriptor::DestroyVirtualDisplay:
      return ToHook<HWC2_PFN_DESTROY_VIRTUAL_DISPLAY>(
          DeviceHook<int32_t, decltype(&DrmHwcTwo::DestroyVirtualDisplay),
                     &DrmHwcTwo::DestroyVirtualDisplay, hwc2_display_t>);
    case HWC2::FunctionDescriptor::Dump:
      return ToHook<HWC2_PFN_DUMP>(
          DeviceHook<void, decltype(&DrmHwcTwo::Dump), &DrmHwcTwo::Dump,
                     uint32_t *, char *>);
    case HWC2::FunctionDescriptor::GetMaxVirtualDisplayCount:
      return ToHook<HWC2_PFN_GET_MAX_VIRTUAL_DISPLAY_COUNT>(
          DeviceHook<uint32_t, decltype(&DrmHwcTwo::GetMaxVirtualDisplayCount),
                     &DrmHwcTwo::GetMaxVirtualDisplayCount>);
    case HWC2::FunctionDescriptor::RegisterCallback:
      return ToHook<HWC2_PFN_REGISTER_CALLBACK>(
          DeviceHook<int32_t, decltype(&DrmHwcTwo::RegisterCallback),
                     &DrmHwcTwo::RegisterCallback, int32_t,
                     hwc2_callback_data_t, hwc2_function_pointer_t>);

    // Display functions
    case HWC2::FunctionDescriptor::AcceptDisplayChanges:
      return ToHook<HWC2_PFN_ACCEPT_DISPLAY_CHANGES>(
          DisplayHook<decltype(&HwcDisplay::AcceptDisplayChanges),
                      &HwcDisplay::AcceptDisplayChanges>);
    case HWC2::FunctionDescriptor::CreateLayer:
      return ToHook<HWC2_PFN_CREATE_LAYER>(
          DisplayHook<decltype(&HwcDisplay::CreateLayer),
                      &HwcDisplay::CreateLayer, hwc2_layer_t *>);
    case HWC2::FunctionDescriptor::DestroyLayer:
      return ToHook<HWC2_PFN_DESTROY_LAYER>(
          DisplayHook<decltype(&HwcDisplay::DestroyLayer),
                      &HwcDisplay::DestroyLayer, hwc2_layer_t>);
    case HWC2::FunctionDescriptor::GetActiveConfig:
      return ToHook<HWC2_PFN_GET_ACTIVE_CONFIG>(
          DisplayHook<decltype(&HwcDisplay::GetActiveConfig),
                      &HwcDisplay::GetActiveConfig, hwc2_config_t *>);
    case HWC2::FunctionDescriptor::GetChangedCompositionTypes:
      return ToHook<HWC2_PFN_GET_CHANGED_COMPOSITION_TYPES>(
          DisplayHook<decltype(&HwcDisplay::GetChangedCompositionTypes),
                      &HwcDisplay::GetChangedCompositionTypes, uint32_t *,
                      hwc2_layer_t *, int32_t *>);
    case HWC2::FunctionDescriptor::GetClientTargetSupport:
      return ToHook<HWC2_PFN_GET_CLIENT_TARGET_SUPPORT>(
          DisplayHook<decltype(&HwcDisplay::GetClientTargetSupport),
                      &HwcDisplay::GetClientTargetSupport, uint32_t, uint32_t,
                      int32_t, int32_t>);
    case HWC2::FunctionDescriptor::GetColorModes:
      return ToHook<HWC2_PFN_GET_COLOR_MODES>(
          DisplayHook<decltype(&HwcDisplay::GetColorModes),
                      &HwcDisplay::GetColorModes, uint32_t *, int32_t *>);
    case HWC2::FunctionDescriptor::GetDisplayAttribute:
      return ToHook<HWC2_PFN_GET_DISPLAY_ATTRIBUTE>(
          DisplayHook<decltype(&HwcDisplay::GetDisplayAttribute),
                      &HwcDisplay::GetDisplayAttribute, hwc2_config_t, int32_t,
                      int32_t *>);
    case HWC2::FunctionDescriptor::GetDisplayConfigs:
      return ToHook<HWC2_PFN_GET_DISPLAY_CONFIGS>(
          DisplayHook<decltype(&HwcDisplay::GetDisplayConfigs),
                      &HwcDisplay::GetDisplayConfigs, uint32_t *,
                      hwc2_config_t *>);
    case HWC2::FunctionDescriptor::GetDisplayName:
      return ToHook<HWC2_PFN_GET_DISPLAY_NAME>(
          DisplayHook<decltype(&HwcDisplay::GetDisplayName),
                      &HwcDisplay::GetDisplayName, uint32_t *, char *>);
    case HWC2::FunctionDescriptor::GetDisplayRequests:
      return ToHook<HWC2_PFN_GET_DISPLAY_REQUESTS>(
          DisplayHook<decltype(&HwcDisplay::GetDisplayRequests),
                      &HwcDisplay::GetDisplayRequests, int32_t *, uint32_t *,
                      hwc2_layer_t *, int32_t *>);
    case HWC2::FunctionDescriptor::GetDisplayType:
      return ToHook<HWC2_PFN_GET_DISPLAY_TYPE>(
          DisplayHook<decltype(&HwcDisplay::GetDisplayType),
                      &HwcDisplay::GetDisplayType, int32_t *>);
    case HWC2::FunctionDescriptor::GetDozeSupport:
      return ToHook<HWC2_PFN_GET_DOZE_SUPPORT>(
          DisplayHook<decltype(&HwcDisplay::GetDozeSupport),
                      &HwcDisplay::GetDozeSupport, int32_t *>);
    case HWC2::FunctionDescriptor::GetHdrCapabilities:
      return ToHook<HWC2_PFN_GET_HDR_CAPABILITIES>(
          DisplayHook<decltype(&HwcDisplay::GetHdrCapabilities),
                      &HwcDisplay::GetHdrCapabilities, uint32_t *, int32_t *,
                      float *, float *, float *>);
    case HWC2::FunctionDescriptor::GetReleaseFences:
      return ToHook<HWC2_PFN_GET_RELEASE_FENCES>(
          DisplayHook<decltype(&HwcDisplay::GetReleaseFences),
                      &HwcDisplay::GetReleaseFences, uint32_t *, hwc2_layer_t *,
                      int32_t *>);
    case HWC2::FunctionDescriptor::PresentDisplay:
      return ToHook<HWC2_PFN_PRESENT_DISPLAY>(
          DisplayHook<decltype(&HwcDisplay::PresentDisplay),
                      &HwcDisplay::PresentDisplay, int32_t *>);
    case HWC2::FunctionDescriptor::SetActiveConfig:
      return ToHook<HWC2_PFN_SET_ACTIVE_CONFIG>(
          DisplayHook<decltype(&HwcDisplay::SetActiveConfig),
                      &HwcDisplay::SetActiveConfig, hwc2_config_t>);
    case HWC2::FunctionDescriptor::SetClientTarget:
      return ToHook<HWC2_PFN_SET_CLIENT_TARGET>(
          DisplayHook<decltype(&HwcDisplay::SetClientTarget),
                      &HwcDisplay::SetClientTarget, buffer_handle_t, int32_t,
                      int32_t, hwc_region_t>);
    case HWC2::FunctionDescriptor::SetColorMode:
      return ToHook<HWC2_PFN_SET_COLOR_MODE>(
          DisplayHook<decltype(&HwcDisplay::SetColorMode),
                      &HwcDisplay::SetColorMode, int32_t>);
    case HWC2::FunctionDescriptor::SetColorTransform:
      return ToHook<HWC2_PFN_SET_COLOR_TRANSFORM>(
          DisplayHook<decltype(&HwcDisplay::SetColorTransform),
                      &HwcDisplay::SetColorTransform, const float *, int32_t>);
    case HWC2::FunctionDescriptor::SetOutputBuffer:
      return ToHook<HWC2_PFN_SET_OUTPUT_BUFFER>(
          DisplayHook<decltype(&HwcDisplay::SetOutputBuffer),
                      &HwcDisplay::SetOutputBuffer, buffer_handle_t, int32_t>);
    case HWC2::FunctionDescriptor::SetPowerMode:
      return ToHook<HWC2_PFN_SET_POWER_MODE>(
          DisplayHook<decltype(&HwcDisplay::SetPowerMode),
                      &HwcDisplay::SetPowerMode, int32_t>);
    case HWC2::FunctionDescriptor::SetVsyncEnabled:
      return ToHook<HWC2_PFN_SET_VSYNC_ENABLED>(
          DisplayHook<decltype(&HwcDisplay::SetVsyncEnabled),
                      &HwcDisplay::SetVsyncEnabled, int32_t>);
    case HWC2::FunctionDescriptor::ValidateDisplay:
      return ToHook<HWC2_PFN_VALIDATE_DISPLAY>(
          DisplayHook<decltype(&HwcDisplay::ValidateDisplay),
                      &HwcDisplay::ValidateDisplay, uint32_t *, uint32_t *>);

    // Layer functions
    case HWC2::FunctionDescriptor::SetCursorPosition:
      return ToHook<HWC2_PFN_SET_CURSOR_POSITION>(
          LayerHook<decltype(&HwcLayer::SetCursorPosition),
                    &HwcLayer::SetCursorPosition, int32_t, int32_t>);
    case HWC2::FunctionDescriptor::SetLayerBlendMode:
      return ToHook<HWC2_PFN_SET_LAYER_BLEND_MODE>(
          LayerHook<decltype(&HwcLayer::SetLayerBlendMode),
                    &HwcLayer::SetLayerBlendMode, int32_t>);
    case HWC2::FunctionDescriptor::SetLayerBuffer:
      return ToHook<HWC2_PFN_SET_LAYER_BUFFER>(
          LayerHook<decltype(&HwcLayer::SetLayerBuffer),
                    &HwcLayer::SetLayerBuffer, buffer_handle_t, int32_t>);
    case HWC2::FunctionDescriptor::SetLayerColor:
      return ToHook<HWC2_PFN_SET_LAYER_COLOR>(
          LayerHook<decltype(&HwcLayer::SetLayerColor),
                    &HwcLayer::SetLayerColor, hwc_color_t>);
    case HWC2::FunctionDescriptor::SetLayerCompositionType:
      return ToHook<HWC2_PFN_SET_LAYER_COMPOSITION_TYPE>(
          LayerHook<decltype(&HwcLayer::SetLayerCompositionType),
                    &HwcLayer::SetLayerCompositionType, int32_t>);
    case HWC2::FunctionDescriptor::SetLayerDataspace:
      return ToHook<HWC2_PFN_SET_LAYER_DATASPACE>(
          LayerHook<decltype(&HwcLayer::SetLayerDataspace),
                    &HwcLayer::SetLayerDataspace, int32_t>);
    case HWC2::FunctionDescriptor::SetLayerDisplayFrame:
      return ToHook<HWC2_PFN_SET_LAYER_DISPLAY_FRAME>(
          LayerHook<decltype(&HwcLayer::SetLayerDisplayFrame),
                    &HwcLayer::SetLayerDisplayFrame, hwc_rect_t>);
    case HWC2::FunctionDescriptor::SetLayerPlaneAlpha:
      return ToHook<HWC2_PFN_SET_LAYER_PLANE_ALPHA>(
          LayerHook<decltype(&HwcLayer::SetLayerPlaneAlpha),
                    &HwcLayer::SetLayerPlaneAlpha, float>);
    case HWC2::FunctionDescriptor::SetLayerSidebandStream:
      return ToHook<HWC2_PFN_SET_LAYER_SIDEBAND_STREAM>(
          LayerHook<decltype(&HwcLayer::SetLayerSidebandStream),
                    &HwcLayer::SetLayerSidebandStream,
                    const native_handle_t *>);
    case HWC2::FunctionDescriptor::SetLayerSourceCrop:
      return ToHook<HWC2_PFN_SET_LAYER_SOURCE_CROP>(
          LayerHook<decltype(&HwcLayer::SetLayerSourceCrop),
                    &HwcLayer::SetLayerSourceCrop, hwc_frect_t>);
    case HWC2::FunctionDescriptor::SetLayerSurfaceDamage:
      return ToHook<HWC2_PFN_SET_LAYER_SURFACE_DAMAGE>(
          LayerHook<decltype(&HwcLayer::SetLayerSurfaceDamage),
                    &HwcLayer::SetLayerSurfaceDamage, hwc_region_t>);
    case HWC2::FunctionDescriptor::SetLayerTransform:
      return ToHook<HWC2_PFN_SET_LAYER_TRANSFORM>(
          LayerHook<decltype(&HwcLayer::SetLayerTransform),
                    &HwcLayer::SetLayerTransform, int32_t>);
    case HWC2::FunctionDescriptor::SetLayerVisibleRegion:
      return ToHook<HWC2_PFN_SET_LAYER_VISIBLE_REGION>(
          LayerHook<decltype(&HwcLayer::SetLayerVisibleRegion),
                    &HwcLayer::SetLayerVisibleRegion, hwc_region_t>);
    case HWC2::FunctionDescriptor::SetLayerZOrder:
      return ToHook<HWC2_PFN_SET_LAYER_Z_ORDER>(
          LayerHook<decltype(&HwcLayer::SetLayerZOrder),
                    &HwcLayer::SetLayerZOrder, uint32_t>);
    case HWC2::FunctionDescriptor::Invalid:
    default:
      return NULL;
  }
}

// static
int DrmHwcTwo::HookDevOpen(const struct hw_module_t *module, const char *name,
                           struct hw_device_t **dev) {
  if (strcmp(name, HWC_HARDWARE_COMPOSER)) {
    ALOGE("Invalid module name- %s", name);
    return -EINVAL;
  }
  InitDebugModule();

  std::unique_ptr<DrmHwcTwo> ctx(new DrmHwcTwo());
  if (!ctx) {
    ALOGE("Failed to allocate DrmHwcTwo");
    return -ENOMEM;
  }

  HWC2::Error err = ctx->Init();
  if (err != HWC2::Error::None) {
    ALOGE("Failed to initialize DrmHwcTwo err=%d\n", err);
    return -EINVAL;
  }

  ctx->common.module = const_cast<hw_module_t *>(module);
  *dev = &ctx->common;
  ctx.release();
  return 0;
}
}  // namespace android

static struct hw_module_methods_t hwc2_module_methods = {
    .open = android::DrmHwcTwo::HookDevOpen,
};

hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .module_api_version = HARDWARE_MODULE_API_VERSION(2, 0),
    .id = HWC_HARDWARE_MODULE_ID,
    .name = "DrmHwcTwo module",
    .author = "The Android Open Source Project",
    .methods = &hwc2_module_methods,
    .dso = NULL,
    .reserved = {0},
};
