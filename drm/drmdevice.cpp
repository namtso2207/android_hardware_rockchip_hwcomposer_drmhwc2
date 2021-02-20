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

#define LOG_TAG "hwc-drm-device"

#include "drmdevice.h"
#include "drmconnector.h"
#include "drmcrtc.h"
#include "drmencoder.h"
#include "drmeventlistener.h"
#include "drmplane.h"
#include "rockchip/utils/drmdebug.h"
#include "rockchip/drmtype.h"


#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <cinttypes>

#include <cutils/properties.h>
#include <log/log.h>

//XML prase
#include <tinyxml2.h>


#define DEFAULT_PRIORITY 10

namespace android {

DrmDevice::DrmDevice() : event_listener_(this) {
}

DrmDevice::~DrmDevice() {
  event_listener_.Exit();
}
bool PlaneSortByZpos(const DrmPlane* plane1,const DrmPlane* plane2)
{
    int ret = 0;
    uint64_t zpos1,zpos2;
    std::tie(ret, zpos1) = plane1->zpos_property().value();
    std::tie(ret, zpos2) = plane2->zpos_property().value();
    return zpos1 < zpos2;
}

bool SortByWinType(const PlaneGroup* planeGroup1,const PlaneGroup* planeGroup2)
{
    return planeGroup1->win_type < planeGroup2->win_type;
}

bool PlaneSortByArea(const DrmPlane*  plane1,const DrmPlane* plane2)
{
    int ret = 0;
    uint64_t area1=0,area2=0;
    if(plane1->area_id_property().id() && plane2->area_id_property().id())
    {
        std::tie(ret, area1) = plane1->area_id_property().value();
        std::tie(ret, area2) = plane2->area_id_property().value();
    }
    return area1 < area2;
}

void DrmDevice::init_white_modes(void){
  tinyxml2::XMLDocument doc;

  doc.LoadFile("/system/usr/share/resolution_white.xml");

  tinyxml2::XMLElement* root=doc.RootElement();
  /* usr tingxml2 to parse resolution.xml */
  if (!root)
    return;

  tinyxml2::XMLElement* resolution =root->FirstChildElement("resolution");

  while (resolution) {
    drmModeModeInfo m;

  #define PARSE(x) \
    tinyxml2::XMLElement* _##x = resolution->FirstChildElement(#x); \
    if (!_##x) { \
      ALOGE("------> failed to parse %s\n", #x); \
      resolution = resolution->NextSiblingElement(); \
      continue; \
    } \
    m.x = atoi(_##x->GetText())
  #define PARSE_HEX(x) \
    tinyxml2::XMLElement* _##x = resolution->FirstChildElement(#x); \
    if (!_##x) { \
      ALOGE("------> failed to parse %s\n", #x); \
      resolution = resolution->NextSiblingElement(); \
      continue; \
    } \
    sscanf(_##x->GetText(), "%x", &m.x);

    PARSE(clock);
    PARSE(hdisplay);
    PARSE(hsync_start);
    PARSE(hsync_end);
    PARSE(hskew);
    PARSE(vdisplay);
    PARSE(vsync_start);
    PARSE(vsync_end);
    PARSE(vscan);
    PARSE(vrefresh);
    PARSE(htotal);
    PARSE(vtotal);
    PARSE_HEX(flags);

    DrmMode mode(&m);
    /* add modes in "resolution.xml" to white list */
    white_modes_.push_back(mode);
    resolution = resolution->NextSiblingElement();
  }
}

bool DrmDevice::mode_verify(const DrmMode &m) {
  if (!white_modes_.size())
    return true;

  for (const DrmMode &mode : white_modes_) {
    if (mode.h_display() == m.h_display() && mode.v_display() == m.v_display() &&
	mode.h_total() == m.h_total() && mode.v_total() == m.v_total() &&
	mode.clock() == m.clock() && mode.flags() == m.flags())
      return true;
  }
  return false;
}

std::tuple<int, int> DrmDevice::Init(const char *path, int num_displays) {


  init_white_modes();

  /* TODO: Use drmOpenControl here instead */
  fd_.Set(open(path, O_RDWR));
  if (fd() < 0) {
    ALOGE("Failed to open dri- %s", strerror(-errno));
    return std::make_tuple(-ENODEV, 0);
  }

  int ret = drmSetClientCap(fd(), DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);
  if (ret) {
    ALOGE("Failed to set universal plane cap %d", ret);
    return std::make_tuple(ret, 0);
  }

  ret = drmSetClientCap(fd(), DRM_CLIENT_CAP_ATOMIC, 1);
  if (ret) {
    ALOGE("Failed to set atomic cap %d", ret);
    return std::make_tuple(ret, 0);
  }

#ifdef DRM_CLIENT_CAP_WRITEBACK_CONNECTORS
  ret = drmSetClientCap(fd(), DRM_CLIENT_CAP_WRITEBACK_CONNECTORS, 1);
  if (ret) {
    ALOGI("Failed to set writeback cap %d", ret);
    ret = 0;
  }
#endif

    //Open Multi-area support.
    ret = drmSetClientCap(fd(), DRM_CLIENT_CAP_SHARE_PLANES, 1);
    if (ret) {
      ALOGE("Failed to set share planes %d", ret);
      return std::make_tuple(ret, 0);
    }

#if USE_NO_ASPECT_RATIO
    //Disable Aspect Ratio
    ret = drmSetClientCap(fd(), DRM_CLIENT_CAP_ASPECT_RATIO, 0);
    if (ret) {
      ALOGE("Failed to disable Aspect Ratio %d", ret);
      return std::make_tuple(ret, 0);
    }
#endif

  drmModeResPtr res = drmModeGetResources(fd());
  if (!res) {
    ALOGE("Failed to get DrmDevice resources");
    return std::make_tuple(-ENODEV, 0);
  }

  min_resolution_ = std::pair<uint32_t, uint32_t>(res->min_width,
                                                  res->min_height);
  max_resolution_ = std::pair<uint32_t, uint32_t>(res->max_width,
                                                  res->max_height);

  // Assumes that the primary display will always be in the first
  // drm_device opened.
  bool found_primary = num_displays != 0;

  for (int i = 0; !ret && i < res->count_crtcs; ++i) {
    drmModeCrtcPtr c = drmModeGetCrtc(fd(), res->crtcs[i]);
    if (!c) {
      ALOGE("Failed to get crtc %d", res->crtcs[i]);
      ret = -ENODEV;
      break;
    }

    std::unique_ptr<DrmCrtc> crtc(new DrmCrtc(this, c, i));
    drmModeFreeCrtc(c);

    ret = crtc->Init();
    if (ret) {
      ALOGE("Failed to initialize crtc %d", res->crtcs[i]);
      break;
    }
    soc_id_ = crtc->get_soc_id();
    crtcs_.emplace_back(std::move(crtc));
  }

  std::vector<int> possible_clones;
  for (int i = 0; !ret && i < res->count_encoders; ++i) {
    drmModeEncoderPtr e = drmModeGetEncoder(fd(), res->encoders[i]);
    if (!e) {
      ALOGE("Failed to get encoder %d", res->encoders[i]);
      ret = -ENODEV;
      break;
    }

    std::vector<DrmCrtc *> possible_crtcs;
    DrmCrtc *current_crtc = NULL;
    for (auto &crtc : crtcs_) {
      if ((1 << crtc->pipe()) & e->possible_crtcs)
        possible_crtcs.push_back(crtc.get());

      if (crtc->id() == e->crtc_id)
        current_crtc = crtc.get();
    }

    std::unique_ptr<DrmEncoder> enc(
        new DrmEncoder(e, current_crtc, possible_crtcs));
    possible_clones.push_back(e->possible_clones);
    drmModeFreeEncoder(e);

    encoders_.emplace_back(std::move(enc));
  }

  for (unsigned int i = 0; i < encoders_.size(); i++) {
    for (unsigned int j = 0; j < encoders_.size(); j++)
      if (possible_clones[i] & (1 << j))
        encoders_[i]->AddPossibleClone(encoders_[j].get());
  }

  for (int i = 0; !ret && i < res->count_connectors; ++i) {
    drmModeConnectorPtr c = drmModeGetConnector(fd(), res->connectors[i]);
    if (!c) {
      ALOGE("Failed to get connector %d", res->connectors[i]);
      ret = -ENODEV;
      break;
    }

    std::vector<DrmEncoder *> possible_encoders;
    DrmEncoder *current_encoder = NULL;
    for (int j = 0; j < c->count_encoders; ++j) {
      for (auto &encoder : encoders_) {
        if (encoder->id() == c->encoders[j])
          possible_encoders.push_back(encoder.get());
        if (encoder->id() == c->encoder_id)
          current_encoder = encoder.get();
      }
    }

    std::unique_ptr<DrmConnector> conn(
        new DrmConnector(this, c, current_encoder, possible_encoders));

    drmModeFreeConnector(c);

    ret = conn->Init();
    if (ret) {
      ALOGE("Init connector %d failed", res->connectors[i]);
      break;
    }
    conn->UpdateModes();

    if (conn->writeback())
      writeback_connectors_.emplace_back(std::move(conn));
    else
      connectors_.emplace_back(std::move(conn));
  }

  ConfigurePossibleDisplays();

  SetPrimaryDisplay(NULL);
  for (auto &conn : connectors_) {
    if (!(conn->possible_displays() & HWC_DISPLAY_PRIMARY_BIT))
      continue;
    if (conn->internal())
      continue;
    if (conn->state() != DRM_MODE_CONNECTED)
      continue;
    found_primary = true;
    if(NULL == GetConnectorFromType(HWC_DISPLAY_PRIMARY)){
      SetPrimaryDisplay(conn.get());
    }else{
      if(conn.get()->priority() <
        GetConnectorFromType(HWC_DISPLAY_PRIMARY)->priority())
        SetPrimaryDisplay(conn.get());
    }
  }

  if (!found_primary) {
    for (auto &conn : connectors_) {
      if (!(conn->possible_displays() & HWC_DISPLAY_PRIMARY_BIT))
        continue;
      if (conn->state() != DRM_MODE_CONNECTED)
        continue;
      found_primary = true;
      if(NULL == GetConnectorFromType(HWC_DISPLAY_PRIMARY)){
        SetPrimaryDisplay(conn.get());
      }else{
        if(conn.get()->priority() <
           GetConnectorFromType(HWC_DISPLAY_PRIMARY)->priority())
          SetPrimaryDisplay(conn.get());
      }
    }
  }

  if (!found_primary) {
    for (auto &conn : connectors_) {
      if (!(conn->possible_displays() & HWC_DISPLAY_PRIMARY_BIT))
        continue;
      found_primary = true;
      if(NULL == GetConnectorFromType(HWC_DISPLAY_PRIMARY)){
        SetPrimaryDisplay(conn.get());
      }else{
        if(conn.get()->priority() <
           GetConnectorFromType(HWC_DISPLAY_PRIMARY)->priority())
          SetPrimaryDisplay(conn.get());
      }
    }
  }

  if (!found_primary) {
    for (auto &conn : connectors_) {
      found_primary = true;
      conn->set_possible_displays(conn->possible_displays() | HWC_DISPLAY_PRIMARY_BIT);
      SetPrimaryDisplay(conn.get());
      break;
    }
  }

  if (!found_primary) {
    ALOGE("failed to find primary display\n");
    return std::make_tuple(-ENODEV, 0);
  }else{
    DrmConnector *primary = GetConnectorFromType(HWC_DISPLAY_PRIMARY);
    if(primary != NULL){
      primary->set_display(num_displays);
      displays_[num_displays] = num_displays;
      ++num_displays;
    }
  }


  SetExtendDisplay(NULL);
  for (auto &conn : connectors_) {
      if (GetConnectorFromType(HWC_DISPLAY_PRIMARY) == conn.get())
        continue;
      if (!(conn->possible_displays() & HWC_DISPLAY_EXTERNAL_BIT))
        continue;
      if (conn->state() != DRM_MODE_CONNECTED)
        continue;
      if(NULL == GetConnectorFromType(HWC_DISPLAY_EXTERNAL)){
        SetExtendDisplay(conn.get());
      }else{
        if(conn.get()->priority() <
           GetConnectorFromType(HWC_DISPLAY_EXTERNAL)->priority())
          SetExtendDisplay(conn.get());
      }
  }

  DrmConnector *extend = GetConnectorFromType(HWC_DISPLAY_EXTERNAL);
  if(extend != NULL){
    extend->set_display(num_displays);
    displays_[num_displays] = num_displays;
    ++num_displays;
  }

  for (auto &conn : connectors_) {
    if (GetConnectorFromType(HWC_DISPLAY_PRIMARY) == conn.get())
      continue;
    if (GetConnectorFromType(HWC_DISPLAY_EXTERNAL) == conn.get())
      continue;
    conn->set_display(num_displays);
    displays_[num_displays] = num_displays;
    ++num_displays;
  }


  if (res)
    drmModeFreeResources(res);

  // Catch-all for the above loops
  if (ret)
    return std::make_tuple(ret, 0);

  drmModePlaneResPtr plane_res = drmModeGetPlaneResources(fd());
  if (!plane_res) {
    ALOGE("Failed to get plane resources");
    return std::make_tuple(-ENOENT, 0);
  }

  for (uint32_t i = 0; i < plane_res->count_planes; ++i) {
    drmModePlanePtr p = drmModeGetPlane(fd(), plane_res->planes[i]);
    if (!p) {
      ALOGE("Failed to get plane %d", plane_res->planes[i]);
      ret = -ENODEV;
      break;
    }

    std::unique_ptr<DrmPlane> plane(new DrmPlane(this, p));

    ret = plane->Init();
    if (ret) {
      ALOGE("Init plane %d failed", plane_res->planes[i]);
      drmModeFreePlane(p);
      break;
    }
    uint64_t share_id,zpos,crtc_id;
    std::tie(ret, share_id) = plane->share_id_property().value();
    std::tie(ret, zpos) = plane->zpos_property().value();
    std::tie(ret, crtc_id) = plane->crtc_property().value();

    std::vector<PlaneGroup*>::const_iterator iter;
    for (iter = plane_groups_.begin();
     iter != plane_groups_.end(); ++iter){
      if((*iter)->share_id == share_id){
        (*iter)->planes.push_back(plane.get());
        break;
      }
    }
    if(iter == plane_groups_.end()){
      PlaneGroup* plane_group = new PlaneGroup();
      plane_group->bUse= false;
      plane_group->zpos = zpos;
      plane_group->possible_crtcs = p->possible_crtcs;
      plane_group->share_id = share_id;
      plane_group->win_type = plane->win_type();
      plane_group->planes.push_back(plane.get());
      plane_groups_.push_back(plane_group);
    }

    for (uint32_t j = 0; j < p->count_formats; j++) {
      if (p->formats[j] == DRM_FORMAT_NV12 ||
         p->formats[j] == DRM_FORMAT_NV21) {
             plane->set_yuv(true);
      }
    }
    sort_planes_.emplace_back(plane.get());

    drmModeFreePlane(p);

    planes_.emplace_back(std::move(plane));

  }


  std::sort(sort_planes_.begin(),sort_planes_.end(),PlaneSortByZpos);

  for (std::vector<DrmPlane*>::const_iterator iter= sort_planes_.begin();
     iter != sort_planes_.end(); ++iter) {
     uint64_t share_id,zpos;
     std::tie(ret, share_id) = (*iter)->share_id_property().value();
     std::tie(ret, zpos) = (*iter)->zpos_property().value();
     ALOGD_IF(LogLevel(DBG_DEBUG),"sort_planes_ share_id=%" PRIu64 ",zpos=%" PRIu64 "",share_id,zpos);
  }

  for (std::vector<PlaneGroup *> ::const_iterator iter = plane_groups_.begin();
         iter != plane_groups_.end(); ++iter)
  {
      ALOGD_IF(LogLevel(DBG_DEBUG),"Plane groups: zpos=%d,share_id=%" PRIu64 ",plane size=%zu",
          (*iter)->zpos,(*iter)->share_id,(*iter)->planes.size());
      for(std::vector<DrmPlane*> ::const_iterator iter_plane = (*iter)->planes.begin();
         iter_plane != (*iter)->planes.end(); ++iter_plane)
      {
          ALOGD_IF(LogLevel(DBG_DEBUG),"\tPlane id=%d",(*iter_plane)->id());
      }
  }
  ALOGD_IF(LogLevel(DBG_DEBUG),"--------------------sort plane--------------------");
  std::sort(plane_groups_.begin(),plane_groups_.end(),SortByWinType);
  for (std::vector<PlaneGroup *> ::const_iterator iter = plane_groups_.begin();
         iter != plane_groups_.end(); ++iter)
  {
      ALOGD_IF(LogLevel(DBG_DEBUG),"Plane groups: zpos=%d,share_id=%" PRIu64 ",plane size=%zu,possible_crtcs=0x%x",
          (*iter)->zpos,(*iter)->share_id,(*iter)->planes.size(),(*iter)->possible_crtcs);
      std::sort((*iter)->planes.begin(),(*iter)->planes.end(), PlaneSortByArea);
      for(std::vector<DrmPlane*> ::const_iterator iter_plane = (*iter)->planes.begin();
         iter_plane != (*iter)->planes.end(); ++iter_plane)
      {
          int ret = 0;
          uint64_t area=0;
          if((*iter_plane)->area_id_property().id())
              std::tie(ret, area) = (*iter_plane)->area_id_property().value();
          ALOGD_IF(LogLevel(DBG_DEBUG),"\tPlane id=%d,area id=%" PRIu64 "",(*iter_plane)->id(),area);
      }
  }


  drmModeFreePlaneResources(plane_res);
  if (ret)
    return std::make_tuple(ret, 0);

  ret = event_listener_.Init();
  if (ret) {
    ALOGE("Can't initialize event listener %d", ret);
    return std::make_tuple(ret, 0);
  }

  return std::make_tuple(ret, displays_.size());
}

bool DrmDevice::HandlesDisplay(int display) const {
  return displays_.find(display) != displays_.end();
}
void DrmDevice::DisplayChanged(void) {
    enable_changed_ = true;
}

void DrmDevice::SetPrimaryDisplay(DrmConnector *c) {
  if (primary_ != c) {
    primary_ = c;
  }
    enable_changed_ = true;
}

void DrmDevice::SetExtendDisplay(DrmConnector *c) {
  if (extend_ != c) {
    if (extend_)
      extend_->force_disconnect(false);
    extend_ = c;
    enable_changed_ = true;
  }
}

DrmConnector *DrmDevice::GetConnectorFromType(int display_type) const {
  if (display_type == HWC_DISPLAY_PRIMARY)
    return primary_;
  else if (display_type == HWC_DISPLAY_EXTERNAL)
    return extend_;
  return NULL;
}

DrmConnector *DrmDevice::GetConnectorForDisplay(int display) const {
  for (auto &conn : connectors_) {
    if (conn->display() == display)
      return conn.get();
  }
  return NULL;
}

int DrmDevice::GetTypeForConnector(DrmConnector *conn) const {
  if (conn == primary_)
    return HWC_DISPLAY_PRIMARY;
  else if (conn == extend_)
    return HWC_DISPLAY_EXTERNAL;
  return -1;
}

DrmConnector *DrmDevice::GetWritebackConnectorForDisplay(int display) const {
  for (auto &conn : writeback_connectors_) {
    if (conn->display() == display)
      return conn.get();
  }
  return NULL;
}

// TODO what happens when hotplugging
DrmConnector *DrmDevice::AvailableWritebackConnector(int display) const {
  DrmConnector *writeback_conn = GetWritebackConnectorForDisplay(display);
  DrmConnector *display_conn = GetConnectorForDisplay(display);
  // If we have a writeback already attached to the same CRTC just use that,
  // if possible.
  if (display_conn && writeback_conn &&
      writeback_conn->encoder()->CanClone(display_conn->encoder()))
    return writeback_conn;

  // Use another CRTC if available and doesn't have any connector
  for (auto &crtc : crtcs_) {
    if (crtc->display() == display)
      continue;
    display_conn = GetConnectorForDisplay(crtc->display());
    // If we have a display connected don't use it for writeback
    if (display_conn && display_conn->state() == DRM_MODE_CONNECTED)
      continue;
    writeback_conn = GetWritebackConnectorForDisplay(crtc->display());
    if (writeback_conn)
      return writeback_conn;
  }
  return NULL;
}

DrmCrtc *DrmDevice::GetCrtcForDisplay(int display) const {
  for (auto &crtc : crtcs_) {
    if (crtc->display() == display)
      return crtc.get();
  }
  return NULL;
}

DrmPlane *DrmDevice::GetPlane(uint32_t id) const {
  for (auto &plane : planes_) {
    if (plane->id() == id)
      return plane.get();
  }
  return NULL;
}

const std::vector<std::unique_ptr<DrmCrtc>> &DrmDevice::crtcs() const {
  return crtcs_;
}

uint32_t DrmDevice::next_mode_id() {
  return ++mode_id_;
}

int DrmDevice::TryEncoderForDisplay(int display, DrmEncoder *enc) {
  /* First try to use the currently-bound crtc */
  DrmCrtc *crtc = enc->crtc();
  if (crtc && crtc->can_bind(display)) {
    crtc->set_display(display);
    enc->set_crtc(crtc);
    return 0;
  }

  /* Try to find a possible crtc which will work */
  for (DrmCrtc *crtc : enc->possible_crtcs()) {
    /* We've already tried this earlier */
    if (crtc == enc->crtc())
      continue;

    if (crtc->can_bind(display)) {
      crtc->set_display(display);
      enc->set_crtc(crtc);
      return 0;
    }
  }

  /* We can't use the encoder, but nothing went wrong, try another one */
  return -EAGAIN;
}

int DrmDevice::CreateDisplayPipe(DrmConnector *connector) {
  int display = connector->display();
  /* Try to use current setup first */
  if (connector->encoder()) {
    int ret = TryEncoderForDisplay(display, connector->encoder());
    if (!ret) {
      return 0;
    } else if (ret != -EAGAIN) {
      ALOGE("Could not set mode %d/%d", display, ret);
      return ret;
    }
  }

  for (DrmEncoder *enc : connector->possible_encoders()) {
    int ret = TryEncoderForDisplay(display, enc);
    if (!ret) {
      connector->set_encoder(enc);
      return 0;
    } else if (ret != -EAGAIN) {
      ALOGE("Could not set mode %d/%d", display, ret);
      return ret;
    }
  }
  ALOGE("Could not find a suitable encoder/crtc for display %d",
        connector->display());
  return -ENODEV;
}

// Attach writeback connector to the CRTC linked to the display_conn
int DrmDevice::AttachWriteback(DrmConnector *display_conn) {
  DrmCrtc *display_crtc = display_conn->encoder()->crtc();
  if (GetWritebackConnectorForDisplay(display_crtc->display()) != NULL) {
    ALOGE("Display already has writeback attach to it");
    return -EINVAL;
  }
  for (auto &writeback_conn : writeback_connectors_) {
    if (writeback_conn->display() >= 0)
      continue;
    for (DrmEncoder *writeback_enc : writeback_conn->possible_encoders()) {
      for (DrmCrtc *possible_crtc : writeback_enc->possible_crtcs()) {
        if (possible_crtc != display_crtc)
          continue;
        // Use just encoders which had not been bound already
        if (writeback_enc->can_bind(display_crtc->display())) {
          writeback_enc->set_crtc(display_crtc);
          writeback_conn->set_encoder(writeback_enc);
          writeback_conn->set_display(display_crtc->display());
          writeback_conn->UpdateModes();
          return 0;
        }
      }
    }
  }
  return -EINVAL;
}

int DrmDevice::CreatePropertyBlob(void *data, size_t length,
                                  uint32_t *blob_id) {
  struct drm_mode_create_blob create_blob;
  memset(&create_blob, 0, sizeof(create_blob));
  create_blob.length = length;
  create_blob.data = (__u64)data;

  int ret = drmIoctl(fd(), DRM_IOCTL_MODE_CREATEPROPBLOB, &create_blob);
  if (ret) {
    ALOGE("Failed to create mode property blob %d", ret);
    return ret;
  }
  *blob_id = create_blob.blob_id;
  return 0;
}

int DrmDevice::DestroyPropertyBlob(uint32_t blob_id) {
  if (!blob_id)
    return 0;

  struct drm_mode_destroy_blob destroy_blob;
  memset(&destroy_blob, 0, sizeof(destroy_blob));
  destroy_blob.blob_id = (__u32)blob_id;
  int ret = drmIoctl(fd(), DRM_IOCTL_MODE_DESTROYPROPBLOB, &destroy_blob);
  if (ret) {
    ALOGE("Failed to destroy mode property blob %" PRIu32 "/%d", blob_id, ret);
    return ret;
  }
  return 0;
}

DrmEventListener *DrmDevice::event_listener() {
  return &event_listener_;
}

int DrmDevice::GetProperty(uint32_t obj_id, uint32_t obj_type,
                           const char *prop_name, DrmProperty *property) {
  drmModeObjectPropertiesPtr props;

  props = drmModeObjectGetProperties(fd(), obj_id, obj_type);
  if (!props) {
    ALOGE("Failed to get properties for %d/%x", obj_id, obj_type);
    return -ENODEV;
  }

  bool found = false;
  for (int i = 0; !found && (size_t)i < props->count_props; ++i) {
    drmModePropertyPtr p = drmModeGetProperty(fd(), props->props[i]);
    if (!strcmp(p->name, prop_name)) {
      property->Init(p, props->prop_values[i]);
      found = true;
    }
    drmModeFreeProperty(p);
  }

  drmModeFreeObjectProperties(props);
  return found ? 0 : -ENOENT;
}

int DrmDevice::GetPlaneProperty(const DrmPlane &plane, const char *prop_name,
                                DrmProperty *property) {
  return GetProperty(plane.id(), DRM_MODE_OBJECT_PLANE, prop_name, property);
}

int DrmDevice::GetCrtcProperty(const DrmCrtc &crtc, const char *prop_name,
                               DrmProperty *property) {
  return GetProperty(crtc.id(), DRM_MODE_OBJECT_CRTC, prop_name, property);
}

int DrmDevice::GetConnectorProperty(const DrmConnector &connector,
                                    const char *prop_name,
                                    DrmProperty *property) {
  return GetProperty(connector.id(), DRM_MODE_OBJECT_CONNECTOR, prop_name,
                     property);
}

// RK surport
void DrmDevice::ConfigurePossibleDisplays(){
  char primary_name[PROPERTY_VALUE_MAX];
  char extend_name[PROPERTY_VALUE_MAX];
  int primary_length, extend_length;
  int default_display_possible = 0;
  std::string conn_name;
  char acConnName[50];

  primary_length = property_get("vendor.hwc.device.primary", primary_name, NULL);
  extend_length = property_get("vendor.hwc.device.extend", extend_name, NULL);

  if (!primary_length)
    default_display_possible |= HWC_DISPLAY_PRIMARY_BIT;
  if (!extend_length)
    default_display_possible |= HWC_DISPLAY_EXTERNAL_BIT;

  for (auto &conn : connectors_) {
    /*
     * build_in connector default only support on primary display
     */
    if (conn->internal())
      conn->set_possible_displays(default_display_possible & HWC_DISPLAY_PRIMARY_BIT);
    else
      conn->set_possible_displays(default_display_possible & HWC_DISPLAY_EXTERNAL_BIT);
  }

  if (primary_length) {
    std::stringstream ss(primary_name);
    uint32_t connector_priority = 0;
    while(getline(ss, conn_name, ',')) {
      for (auto &conn : connectors_) {
        snprintf(acConnName,50,"%s-%d",connector_type_str(conn->type()),conn->type_id());
        if (!strcmp(connector_type_str(conn->type()), conn_name.c_str()) ||
            !strcmp(acConnName, conn_name.c_str()))
        {
          conn->set_priority(connector_priority);
          conn->set_possible_displays(HWC_DISPLAY_PRIMARY_BIT);
          connector_priority++;
        }
      }
    }
  }

  if (extend_length) {
    std::stringstream ss(extend_name);
    uint32_t connector_priority = 0;
    while(getline(ss, conn_name, ',')) {
      for (auto &conn : connectors_) {
        snprintf(acConnName,50,"%s-%d",connector_type_str(conn->type()),conn->type_id());
        if (!strcmp(connector_type_str(conn->type()), conn_name.c_str()) ||
            !strcmp(acConnName, conn_name.c_str()))
        {
          conn->set_priority(connector_priority);
          conn->set_possible_displays(conn->possible_displays() | HWC_DISPLAY_EXTERNAL_BIT);
          connector_priority++;
        }
      }
    }
  }
  return;
}

int DrmDevice::UpdatePropertys(void)
{
  int timeline = property_get_int32( PROPERTY_TYPE ".display.timeline", -1);
  int ret;
  /*
   * force update propetry when timeline is zero or not exist.
   */
  if (timeline && timeline == prop_timeline_)
    return 0;

  DrmConnector *primary = GetConnectorFromType(HWC_DISPLAY_PRIMARY);
  DrmConnector *extend = GetConnectorFromType(HWC_DISPLAY_EXTERNAL);

  drmModeAtomicReqPtr pset = drmModeAtomicAlloc();
  if (!pset) {
    ALOGE("Failed to allocate property set");
    return -ENOMEM;
  }

  if (primary) {
    DRM_ATOMIC_ADD_PROP(primary->id(), primary->brightness_id_property().id(),
                        property_get_int32("persist." PROPERTY_TYPE ".brightness.main",50))
    DRM_ATOMIC_ADD_PROP(primary->id(), primary->contrast_id_property().id(),
                        property_get_int32("persist." PROPERTY_TYPE ".contrast.main",50))
    DRM_ATOMIC_ADD_PROP(primary->id(), primary->saturation_id_property().id(),
                        property_get_int32("persist." PROPERTY_TYPE ".saturation.main",50))
    DRM_ATOMIC_ADD_PROP(primary->id(), primary->hue_id_property().id(),
                        property_get_int32("persist." PROPERTY_TYPE ".hue.main",50))
  }
  if (extend) {
    DRM_ATOMIC_ADD_PROP(extend->id(), extend->brightness_id_property().id(),
                        property_get_int32("persist." PROPERTY_TYPE ".brightness.aux",50))
    DRM_ATOMIC_ADD_PROP(extend->id(), extend->contrast_id_property().id(),
                        property_get_int32("persist." PROPERTY_TYPE ".contrast.aux",50))
    DRM_ATOMIC_ADD_PROP(extend->id(), extend->saturation_id_property().id(),
                        property_get_int32("persist." PROPERTY_TYPE ".saturation.aux",50))
    DRM_ATOMIC_ADD_PROP(extend->id(), extend->hue_id_property().id(),
                        property_get_int32("persist." PROPERTY_TYPE ".hue.aux",50))
  }

  uint32_t flags = 0;
  ret = drmModeAtomicCommit(fd_.get(), pset, flags, this);
  if (ret < 0) {
    ALOGE("Failed to commit pset ret=%d\n", ret);
    drmModeAtomicFree(pset);
    return ret;
  }
  drmModeAtomicFree(pset);
  prop_timeline_ = timeline;

  return 0;
}


static pthread_mutex_t diplay_route_mutex = PTHREAD_MUTEX_INITIALIZER;
int DrmDevice::UpdateDisplayMode(void){
  int i;
  pthread_mutex_lock(&diplay_route_mutex);
  bool mode_changed = false;
  for (i = 0; i < HWC_NUM_PHYSICAL_DISPLAY_TYPES; i++) {
    DrmConnector *conn = GetConnectorFromType(i);

    if (!conn || conn->state() != DRM_MODE_CONNECTED || !conn->current_mode().id() ||
        !conn->encoder() || !conn->encoder()->crtc())
      continue;
    if (conn->current_mode() == conn->active_mode())
      continue;
    mode_changed = true;
  }

  if (!mode_changed)
  {
    pthread_mutex_unlock(&diplay_route_mutex);
    return 0;
  }

  drmModeAtomicReqPtr pset = drmModeAtomicAlloc();
  if (!pset) {
    ALOGE("%s:line=%d Failed to allocate property set",__FUNCTION__, __LINE__);
    pthread_mutex_unlock(&diplay_route_mutex);
    return -ENOMEM;
  }

  int ret;
  uint32_t blob_id[HWC_NUM_PHYSICAL_DISPLAY_TYPES] = {0};

  for (i = 0; i < HWC_NUM_PHYSICAL_DISPLAY_TYPES; i++) {
    DrmConnector *conn = GetConnectorFromType(i);

    if (!conn)
      continue;
    if (conn->state() != DRM_MODE_CONNECTED ||
        !conn->current_mode().id() || !conn->encoder() ||
        !conn->encoder()->crtc()) {
        DRM_ATOMIC_ADD_PROP(conn->id(), conn->crtc_id_property().id(), 0);
      continue;
    }

    struct drm_mode_modeinfo drm_mode;
    memset(&drm_mode, 0, sizeof(drm_mode));
    conn->current_mode().ToDrmModeModeInfo(&drm_mode);
    ALOGD_IF(LogLevel(DBG_VERBOSE),"%s,line=%d, current_mode id=%d , w=%d,h=%d",__FUNCTION__,__LINE__,
              conn->current_mode().id(),conn->current_mode().h_display(),conn->current_mode().v_display());
    ret = CreatePropertyBlob(&drm_mode, sizeof(drm_mode), &blob_id[i]);
    if (ret)
      continue;

    DrmCrtc *crtc = conn->encoder()->crtc();

//    connector->SetDpmsMode(DRM_MODE_DPMS_ON);
//    DRM_ATOMIC_ADD_PROP(conn->id(), conn->dpms_property().id(), DRM_MODE_DPMS_ON);
    DRM_ATOMIC_ADD_PROP(conn->id(), conn->crtc_id_property().id(), crtc->id());
    DRM_ATOMIC_ADD_PROP(crtc->id(), crtc->mode_property().id(), blob_id[i]);
    DRM_ATOMIC_ADD_PROP(crtc->id(), crtc->active_property().id(), 1);
  }

  uint32_t flags = DRM_MODE_ATOMIC_ALLOW_MODESET;
  ret = drmModeAtomicCommit(fd_.get(), pset, flags, this);
  if (ret < 0) {
    ALOGE("%s:line=%d Failed to commit pset ret=%d\n", __FUNCTION__, __LINE__, ret);
    drmModeAtomicFree(pset);
    pthread_mutex_unlock(&diplay_route_mutex);
    return ret;
  }

  for (i = 0; i < HWC_NUM_PHYSICAL_DISPLAY_TYPES; i++) {
    if (blob_id[i])
      DestroyPropertyBlob(blob_id[i]);
  }
  for (i = 0; i < HWC_NUM_PHYSICAL_DISPLAY_TYPES; i++) {
    DrmConnector *conn = GetConnectorFromType(i);

    if (!conn || conn->state() != DRM_MODE_CONNECTED || !conn->current_mode().id())
      continue;

    if (!conn->encoder() || !conn->encoder()->crtc())
      continue;
    conn->set_active_mode(conn->current_mode());
  }

  drmModeAtomicFree(pset);

  hotplug_timeline++;

  pthread_mutex_unlock(&diplay_route_mutex);
  return 0;
}

int DrmDevice::UpdateDisplayRoute(void)
{
  int i;

  pthread_mutex_lock(&diplay_route_mutex);

  if (!enable_changed_)
  {
    pthread_mutex_unlock(&diplay_route_mutex);
    return 0;
  }

  DrmConnector *primary = GetConnectorFromType(HWC_DISPLAY_PRIMARY);
  if (!primary) {
    ALOGE("%s:line=%d Failed to find primary display\n", __FUNCTION__, __LINE__);
    pthread_mutex_unlock(&diplay_route_mutex);
    return -EINVAL;
  }
  DrmConnector *extend = GetConnectorFromType(HWC_DISPLAY_EXTERNAL);
  if (enable_changed_) {
    primary->set_encoder(NULL);
    if (extend)
      extend->set_encoder(NULL);
    if (primary->state() == DRM_MODE_CONNECTED) {
      for (DrmEncoder *enc : primary->possible_encoders()) {
        for (DrmCrtc *crtc : enc->possible_crtcs()) {
          if (crtc->get_afbc()) {
            crtc->set_display(primary->display());
            enc->set_crtc(crtc);
            primary->set_encoder(enc);
            ALOGD_IF(LogLevel(DBG_VERBOSE), "%s:line=%d set primary with conn[%d] crtc=%d\n",
                    __FUNCTION__, __LINE__, primary->id(), crtc->id());
          }
        }
      }
      /*
       * not limit
       */
      if (!primary->encoder() || !primary->encoder()->crtc()) {
        for (DrmEncoder *enc : primary->possible_encoders()) {
          for (DrmCrtc *crtc : enc->possible_crtcs()) {
              crtc->set_display(primary->display());
              enc->set_crtc(crtc);
              primary->set_encoder(enc);
              ALOGD_IF(LogLevel(DBG_VERBOSE), "%s:line=%d set primary with conn[%d] crtc=%d\n",
                        __FUNCTION__, __LINE__, primary->id(), crtc->id());
          }
        }
      }
    }
    if (extend && extend->state() == DRM_MODE_CONNECTED) {
      for (DrmEncoder *enc : extend->possible_encoders()) {
        for (DrmCrtc *crtc : enc->possible_crtcs()) {
          if (primary && primary->encoder() && primary->encoder()->crtc()) {
            if (crtc == primary->encoder()->crtc())
              continue;
          }
          ALOGD_IF(LogLevel(DBG_VERBOSE), "%s:line=%d set extend[%d] with crtc=%d\n",
                    __FUNCTION__, __LINE__, extend->id(), crtc->id());
          crtc->set_display(extend->display());
          enc->set_crtc(crtc);
          extend->set_encoder(enc);
        }
      }
      if (!extend->encoder() || !extend->encoder()->crtc()) {
        for (DrmEncoder *enc : extend->possible_encoders()) {
          for (DrmCrtc *crtc : enc->possible_crtcs()) {
            crtc->set_display(extend->display());
            enc->set_crtc(crtc);
            extend->set_encoder(enc);
            ALOGD_IF(LogLevel(DBG_VERBOSE), "%s:line=%d set extend[%d] with crtc=%d\n",
                    __FUNCTION__, __LINE__, extend->id(), crtc->id());
            if (primary && primary->encoder() && primary->encoder()->crtc()) {
              if (crtc == primary->encoder()->crtc()) {
                //RK618 use a encoder(RGB) to display two connector,so if encoder is same
                //not to set encoder.crtc NULL
                if(enc != primary->encoder()){
                    primary->encoder()->set_crtc(NULL);
                }
                primary->set_encoder(NULL);
                for (DrmEncoder *primary_enc : primary->possible_encoders()) {
                  for (DrmCrtc *primary_crtc : primary_enc->possible_crtcs()) {
                    if (extend && extend->encoder() && extend->encoder()->crtc()) {
                      if (primary_crtc == extend->encoder()->crtc())
                        continue;
                    }
                    crtc->set_display(primary->display());
                    primary_enc->set_crtc(primary_crtc);
                    primary->set_encoder(primary_enc);
                    ALOGD_IF(LogLevel(DBG_VERBOSE), "%s:line=%d set primary with conn[%d] crtc=%d\n",
                            __FUNCTION__, __LINE__, primary->id(), primary_crtc->id());
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  if(primary && primary->encoder() && primary->encoder()->crtc())
  {
    DrmCrtc *crtc = primary->encoder()->crtc();
    if(!crtc->get_afbc()){
      //property_set( PROPERTY_TYPE ".gralloc.disable_afbc", "1");
      ALOGD_IF(LogLevel(DBG_VERBOSE), "%s:line=%d primary conn[%d] crtc=%d support AFBC(%d), to disable AFBC\n",
               __FUNCTION__, __LINE__, primary->id(), crtc->id(),crtc->get_afbc());
    }
  }

  if(primary && !strcmp(connector_type_str(primary->type()), "HDMI-A"))
  {
    if (primary && primary->encoder() && primary->encoder()->crtc())
    {
      char primary_conn_name[50];
      snprintf(primary_conn_name,50,"%s-%d",connector_type_str(primary->type()),primary->type_id());
      property_set( PROPERTY_TYPE ".hwc.device.main", primary_conn_name);
    }
    else
      property_set( PROPERTY_TYPE ".hwc.device.main", "");
  }
  else
  {
    if (primary && primary->encoder() && primary->encoder()->crtc())
      property_set( PROPERTY_TYPE ".hwc.device.main", connector_type_str(primary->type()));
    else
      property_set( PROPERTY_TYPE ".hwc.device.main", "");
  }

  if(extend && !strcmp(connector_type_str(extend->type()), "HDMI-A"))
  {
    if (extend && extend->encoder() && extend->encoder()->crtc())
    {
      char extend_conn_name[50];
      snprintf(extend_conn_name,50,"%s-%d",connector_type_str(extend->type()),extend->type_id());
      property_set( PROPERTY_TYPE ".hwc.device.aux", extend_conn_name);
    }
    else
      property_set( PROPERTY_TYPE ".hwc.device.aux", "");
  }
  else
  {
    if (extend && extend->encoder() && extend->encoder()->crtc())
      property_set( PROPERTY_TYPE ".hwc.device.aux", connector_type_str(extend->type()));
    else
      property_set( PROPERTY_TYPE ".hwc.device.aux", "");
  }

  drmModeAtomicReqPtr pset = drmModeAtomicAlloc();
  if (!pset) {
    ALOGE("%s:line=%d Failed to allocate property set",__FUNCTION__, __LINE__);
    pthread_mutex_unlock(&diplay_route_mutex);
    return -ENOMEM;
  }

  int ret;
  uint32_t blob_id[HWC_NUM_PHYSICAL_DISPLAY_TYPES] = {0};

  for (i = 0; i < HWC_NUM_PHYSICAL_DISPLAY_TYPES; i++) {
    DrmConnector *conn = GetConnectorFromType(i);

    if (!conn)
      continue;
    if (conn->state() != DRM_MODE_CONNECTED ||
        !conn->current_mode().id() || !conn->encoder() ||
        !conn->encoder()->crtc()) {
        DRM_ATOMIC_ADD_PROP(conn->id(), conn->crtc_id_property().id(), 0);
      continue;
    }

    struct drm_mode_modeinfo drm_mode;
    memset(&drm_mode, 0, sizeof(drm_mode));
    conn->current_mode().ToDrmModeModeInfo(&drm_mode);
    ALOGD_IF(LogLevel(DBG_VERBOSE),"%s,line=%d, current_mode id=%d , w=%d,h=%d",__FUNCTION__,__LINE__,
              conn->current_mode().id(),conn->current_mode().h_display(),conn->current_mode().v_display());
    ret = CreatePropertyBlob(&drm_mode, sizeof(drm_mode), &blob_id[i]);
    if (ret)
      continue;

    DrmCrtc *crtc = conn->encoder()->crtc();

//    connector->SetDpmsMode(DRM_MODE_DPMS_ON);
//    DRM_ATOMIC_ADD_PROP(conn->id(), conn->dpms_property().id(), DRM_MODE_DPMS_ON);
    DRM_ATOMIC_ADD_PROP(conn->id(), conn->crtc_id_property().id(), crtc->id());
    DRM_ATOMIC_ADD_PROP(crtc->id(), crtc->mode_property().id(), blob_id[i]);
    DRM_ATOMIC_ADD_PROP(crtc->id(), crtc->active_property().id(), 1);
  }

  /*
   * Disable unused connector
   */
  for (auto &connector : connectors_) {
    bool in_use = false;
    for (i = 0; i < HWC_NUM_PHYSICAL_DISPLAY_TYPES; i++) {
      DrmConnector *conn = GetConnectorFromType(i);
      if (!conn || conn->state() != DRM_MODE_CONNECTED ||
          !conn->current_mode().id() || !conn->encoder() ||
          !conn->encoder()->crtc())
          continue;
      if (conn->id() == connector->id()) {
          in_use = true;
          break;
      }
    }
    if (!in_use) {
      DrmCrtc *mirror = NULL;
      if (connector->state() == DRM_MODE_CONNECTED) {
        for (i = 0; i < HWC_NUM_PHYSICAL_DISPLAY_TYPES; i++) {
          DrmConnector *conn = GetConnectorFromType(i);
          if (!conn || conn->state() != DRM_MODE_CONNECTED ||
              !conn->current_mode().id() || !conn->encoder() ||
              !conn->encoder()->crtc())
            continue;
          if((connector->possible_displays() & conn->possible_displays()) == 0)
            continue;
          for (const DrmMode &conn_mode : connector->modes()) {
            if (conn->current_mode().equal_no_flag_and_type(conn_mode)) {
              mirror = conn->encoder()->crtc();
              break;
            }
          }
          if (mirror)
            break;
        }
      }
      if (mirror) {
        connector->SetDpmsMode(DRM_MODE_DPMS_ON);
        DRM_ATOMIC_ADD_PROP(connector->id(), connector->crtc_id_property().id(),
                            mirror->id());
      } else {
        connector->SetDpmsMode(DRM_MODE_DPMS_OFF);
        DRM_ATOMIC_ADD_PROP(connector->id(), connector->crtc_id_property().id(), 0);
      }
    }
  }
  for (auto &crtc : crtcs_) {
    bool in_use = false;
    for (i = 0; i < HWC_NUM_PHYSICAL_DISPLAY_TYPES; i++) {
      DrmConnector *conn = GetConnectorFromType(i);
      if (!conn || conn->state() != DRM_MODE_CONNECTED ||
          !conn->current_mode().id() || !conn->encoder() ||
          !conn->encoder()->crtc())
          continue;
      if (crtc->id() == conn->encoder()->crtc()->id()) {
        in_use = true;
        break;
      }
    }
    if (!in_use) {
      for(auto &plane_group : plane_groups_){
        uint32_t crtc_mask = 1 << crtc->pipe();
        if(!plane_group->acquire(crtc_mask))
            continue;
        for(auto &plane : plane_group->planes){
          if(!plane)
            continue;
          ret = drmModeAtomicAddProperty(pset, plane->id(),
                                         plane->crtc_property().id(), 0) < 0 ||
                drmModeAtomicAddProperty(pset, plane->id(), plane->fb_property().id(),
                                         0) < 0;
          if (ret) {
            ALOGE("Failed to add plane %d disable to pset", plane->id());
            drmModeAtomicFree(pset);
            return ret;
          }
          ALOGD_IF(LogLevel(DBG_VERBOSE),"%s,line=%d, disable CRTC(%d), disable plane-id = %d",__FUNCTION__,__LINE__,
          crtc->id(),plane->id());
        }
      }
      DRM_ATOMIC_ADD_PROP(crtc->id(), crtc->mode_property().id(), 0);
      DRM_ATOMIC_ADD_PROP(crtc->id(), crtc->active_property().id(), 0);
    }
  }

  uint32_t flags = DRM_MODE_ATOMIC_ALLOW_MODESET;
  ret = drmModeAtomicCommit(fd_.get(), pset, flags, this);
  if (ret < 0) {
    ALOGE("%s:line=%d Failed to commit pset ret=%d\n", __FUNCTION__, __LINE__, ret);
    drmModeAtomicFree(pset);
    pthread_mutex_unlock(&diplay_route_mutex);
    return ret;
  }

  for (i = 0; i < HWC_NUM_PHYSICAL_DISPLAY_TYPES; i++) {
    if (blob_id[i])
      DestroyPropertyBlob(blob_id[i]);
  }
  for (i = 0; i < HWC_NUM_PHYSICAL_DISPLAY_TYPES; i++) {
    DrmConnector *conn = GetConnectorFromType(i);

    if (!conn || conn->state() != DRM_MODE_CONNECTED || !conn->current_mode().id())
      continue;

    if (!conn->encoder() || !conn->encoder()->crtc())
      continue;
    conn->set_active_mode(conn->current_mode());
  }
  enable_changed_ = false;

  drmModeAtomicFree(pset);

  hotplug_timeline++;

  pthread_mutex_unlock(&diplay_route_mutex);

  return 0;
}

int DrmDevice::timeline(void) {
  return hotplug_timeline;
}

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
static inline int64_t U642I64(uint64_t val)
{
  return (int64_t)*((int64_t *)&val);
}

struct type_name {
  int type;
  const char *name;
};

#define type_name_fn(res) \
  const char * DrmDevice::res##_str(int type) {      \
    unsigned int i;         \
    for (i = 0; i < ARRAY_SIZE(res##_names); i++) { \
      if (res##_names[i].type == type)  \
        return res##_names[i].name; \
    }           \
    return "(invalid)";       \
  }

struct type_name encoder_type_names[] = {
  { DRM_MODE_ENCODER_NONE, "none" },
  { DRM_MODE_ENCODER_DAC, "DAC" },
  { DRM_MODE_ENCODER_TMDS, "TMDS" },
  { DRM_MODE_ENCODER_LVDS, "LVDS" },
  { DRM_MODE_ENCODER_TVDAC, "TVDAC" },
};

type_name_fn(encoder_type)

struct type_name connector_status_names[] = {
  { DRM_MODE_CONNECTED, "connected" },
  { DRM_MODE_DISCONNECTED, "disconnected" },
  { DRM_MODE_UNKNOWNCONNECTION, "unknown" },
};

type_name_fn(connector_status)

struct type_name connector_type_names[] = {
  { DRM_MODE_CONNECTOR_Unknown, "unknown" },
  { DRM_MODE_CONNECTOR_VGA, "VGA" },
  { DRM_MODE_CONNECTOR_DVII, "DVI-I" },
  { DRM_MODE_CONNECTOR_DVID, "DVI-D" },
  { DRM_MODE_CONNECTOR_DVIA, "DVI-A" },
  { DRM_MODE_CONNECTOR_Composite, "composite" },
  { DRM_MODE_CONNECTOR_SVIDEO, "s-video" },
  { DRM_MODE_CONNECTOR_LVDS, "LVDS" },
  { DRM_MODE_CONNECTOR_Component, "component" },
  { DRM_MODE_CONNECTOR_9PinDIN, "9-pin DIN" },
  { DRM_MODE_CONNECTOR_DisplayPort, "DP" },
  { DRM_MODE_CONNECTOR_HDMIA, "HDMI-A" },
  { DRM_MODE_CONNECTOR_HDMIB, "HDMI-B" },
  { DRM_MODE_CONNECTOR_TV, "TV" },
  { DRM_MODE_CONNECTOR_eDP, "eDP" },
  { DRM_MODE_CONNECTOR_VIRTUAL, "Virtual" },
  { DRM_MODE_CONNECTOR_DSI, "DSI" },
  { DRM_MODE_CONNECTOR_DPI, "DPI" },
};

type_name_fn(connector_type)

#define bit_name_fn(res)					\
  const char * res##_str(int type, std::ostringstream *out) {       \
    unsigned int i;           \
    const char *sep = "";         \
    for (i = 0; i < ARRAY_SIZE(res##_names); i++) {   \
      if (type & (1 << i)) {        \
        *out << sep << res##_names[i];  \
        sep = ", ";       \
      }           \
    }             \
    return NULL;            \
  }

static const char *mode_type_names[] = {
  "builtin",
  "clock_c",
  "crtc_c",
  "preferred",
  "default",
  "userdef",
  "driver",
};

static bit_name_fn(mode_type)

static const char *mode_flag_names[] = {
  "phsync",
  "nhsync",
  "pvsync",
  "nvsync",
  "interlace",
  "dblscan",
  "csync",
  "pcsync",
  "ncsync",
  "hskew",
  "bcast",
  "pixmux",
  "dblclk",
  "clkdiv2"
};
static bit_name_fn(mode_flag)


void DrmDevice::DumpMode(drmModeModeInfo *mode, std::ostringstream *out) {
  *out << mode->name << " " << mode->vrefresh << " "
       << mode->hdisplay << " " << mode->hsync_start << " "
       << mode->hsync_end << " " << mode->htotal << " "
       << mode->vdisplay << " " << mode->vsync_start << " "
       << mode->vsync_end << " " << mode->vtotal;

  *out << " flags: ";
  mode_flag_str(mode->flags, out);
  *out << " types: " << mode->type << "\n";
    mode_type_str(mode->type, out);
}

void DrmDevice::DumpBlob(uint32_t blob_id, std::ostringstream *out) {
  uint32_t i;
  unsigned char *blob_data;
  drmModePropertyBlobPtr blob;

  blob = drmModeGetPropertyBlob(fd(), blob_id);
  if (!blob) {
    *out << "\n";
    return;
  }

  blob_data = (unsigned char*)blob->data;

  for (i = 0; i < blob->length; i++) {
    if (i % 16 == 0)
      *out << "\n\t\t\t";
    *out << std::hex << blob_data[i];
  }
  *out << "\n";

  drmModeFreePropertyBlob(blob);
}

void DrmDevice::DumpProp(drmModePropertyPtr prop,
          uint32_t prop_id, uint64_t value, std::ostringstream *out) {
  int i;

  *out << "\t" << prop_id;
  if (!prop) {
    *out << "\n";
    return;
  }
  out->str("");
  *out << " " << prop->name << ":\n";

  *out << "\t\tflags:";
  if (prop->flags & DRM_MODE_PROP_PENDING)
    *out << " pending";
  if (prop->flags & DRM_MODE_PROP_IMMUTABLE)
    *out << " immutable";
  if (drm_property_type_is(prop, DRM_MODE_PROP_SIGNED_RANGE))
    *out << " signed range";
  if (drm_property_type_is(prop, DRM_MODE_PROP_RANGE))
    *out << " range";
  if (drm_property_type_is(prop, DRM_MODE_PROP_ENUM))
    *out << " enum";
  if (drm_property_type_is(prop, DRM_MODE_PROP_BITMASK))
    *out << " bitmask";
  if (drm_property_type_is(prop, DRM_MODE_PROP_BLOB))
    *out << " blob";
  if (drm_property_type_is(prop, DRM_MODE_PROP_OBJECT))
    *out << " object";
  *out << "\n";

  if (drm_property_type_is(prop, DRM_MODE_PROP_SIGNED_RANGE)) {
    *out << "\t\tvalues:";
    for (i = 0; i < prop->count_values; i++)
      *out << U642I64(prop->values[i]);
    *out << "\n";
  }

  if (drm_property_type_is(prop, DRM_MODE_PROP_RANGE)) {
    *out << "\t\tvalues:";
    for (i = 0; i < prop->count_values; i++)
      *out << prop->values[i];
    *out << "\n";
  }

  if (drm_property_type_is(prop, DRM_MODE_PROP_ENUM)) {
    *out << "\t\tenums:";
    for (i = 0; i < prop->count_enums; i++)
      *out << prop->enums[i].name << "=" << prop->enums[i].value;
    *out << "\n";
  } else if (drm_property_type_is(prop, DRM_MODE_PROP_BITMASK)) {
    *out << "\t\tvalues:";
    for (i = 0; i < prop->count_enums; i++)
      *out << prop->enums[i].name << "=" << std::hex << (1LL << prop->enums[i].value);
    *out << "\n";
  } else {
    assert(prop->count_enums == 0);
  }

  if (drm_property_type_is(prop, DRM_MODE_PROP_BLOB)) {
    *out << "\t\tblobs:\n";
    for (i = 0; i < prop->count_blobs; i++)
      DumpBlob(prop->blob_ids[i], out);
    *out << "\n";
  } else {
    assert(prop->count_blobs == 0);
  }

  *out << "\t\tvalue:";
  if (drm_property_type_is(prop, DRM_MODE_PROP_BLOB))
    DumpBlob(value, out);
  else
    *out << value;

    *out << "\n";
}

int DrmDevice::DumpProperty(uint32_t obj_id, uint32_t obj_type, std::ostringstream *out) {
  drmModePropertyPtr* prop_info;
  drmModeObjectPropertiesPtr props;

  props = drmModeObjectGetProperties(fd(), obj_id, obj_type);
  if (!props) {
    ALOGE("Failed to get properties for %d/%x", obj_id, obj_type);
    return -ENODEV;
  }
  prop_info = (drmModePropertyPtr*)malloc(props->count_props * sizeof *prop_info);
  if (!prop_info) {
    ALOGE("Malloc drmModePropertyPtr array failed");
    return -ENOMEM;
  }

  *out << "  props:\n";
  for (int i = 0;(size_t)i < props->count_props; ++i) {
    prop_info[i] = drmModeGetProperty(fd(), props->props[i]);

    DumpProp(prop_info[i],props->props[i],props->prop_values[i],out);

    drmModeFreeProperty(prop_info[i]);
  }

  drmModeFreeObjectProperties(props);
  free(prop_info);
  return 0;
}

int DrmDevice::DumpPlaneProperty(const DrmPlane &plane, std::ostringstream *out) {
  return DumpProperty(plane.id(), DRM_MODE_OBJECT_PLANE, out);
}

int DrmDevice::DumpCrtcProperty(const DrmCrtc &crtc, std::ostringstream *out) {
  return DumpProperty(crtc.id(), DRM_MODE_OBJECT_CRTC, out);
}

int DrmDevice::DumpConnectorProperty(const DrmConnector &connector, std::ostringstream *out) {
   return DumpProperty(connector.id(), DRM_MODE_OBJECT_CONNECTOR, out);
}

bool DrmDevice::GetHdrPanelMetadata(DrmConnector *conn, struct hdr_static_metadata* blob_data) {
  drmModePropertyBlobPtr blob;
  drmModeObjectPropertiesPtr props;
  props = drmModeObjectGetProperties(fd(), conn->id(), DRM_MODE_OBJECT_CONNECTOR);
  if (!props) {
    ALOGE("Failed to get properties for %d/%x", conn->id(), DRM_MODE_OBJECT_CONNECTOR);
    return false;
  }

  bool found = false;
  int value;
  for (int i = 0; !found && (size_t)i < props->count_props; ++i) {
    drmModePropertyPtr p = drmModeGetProperty(fd(), props->props[i]);
    if (p && !strcmp(p->name, "HDR_PANEL_METADATA")) {

      if (!drm_property_type_is(p, DRM_MODE_PROP_BLOB))
      {
          ALOGE("%s:line=%d,is not blob",__FUNCTION__,__LINE__);
          drmModeFreeProperty(p);
          drmModeFreeObjectProperties(props);
          return false;
      }

      if (!p->count_blobs)
        value = props->prop_values[i];
      else
        value = p->blob_ids[0];
      blob = drmModeGetPropertyBlob(fd(), value);
      if (!blob) {
        ALOGE("%s:line=%d, blob is null",__FUNCTION__,__LINE__);
        drmModeFreeProperty(p);
        drmModeFreeObjectProperties(props);
        return false;
      }

      memcpy(blob_data,blob->data,sizeof(struct hdr_static_metadata));

      drmModeFreePropertyBlob(blob);

      found = true;
    }
    drmModeFreeProperty(p);
  }

  drmModeFreeObjectProperties(props);

  return found;
}

bool DrmDevice::is_hdr_panel_support_st2084(DrmConnector *conn) const {
  return (conn->get_hdr_metadata_ptr()->eotf & (1 << SMPTE_ST2084)) > 0;
}

bool DrmDevice::is_hdr_panel_support_HLG(DrmConnector *conn) const {
  return (conn->get_hdr_metadata_ptr()->eotf & (1 << HLG)) > 0;
}


bool DrmDevice::is_plane_support_hdr2sdr(DrmCrtc *crtc) const
{
    bool bHdr2sdr = false;
    for (std::vector<PlaneGroup *> ::const_iterator iter = plane_groups_.begin();
           iter != plane_groups_.end(); ++iter)
    {
           for(std::vector<DrmPlane*> ::const_iterator iter_plane = (*iter)->planes.begin();
           iter_plane != (*iter)->planes.end(); ++iter_plane)
        {
            if((*iter_plane)->GetCrtcSupported(*crtc) && (*iter_plane)->get_hdr2sdr())
            {
                bHdr2sdr = true;
                break;
            }
        }
    }

    return bHdr2sdr;
}

}  // namespace android
