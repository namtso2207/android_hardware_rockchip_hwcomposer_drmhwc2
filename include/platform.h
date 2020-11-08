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

#ifndef ANDROID_DRM_PLATFORM_H_
#define ANDROID_DRM_PLATFORM_H_

#include "drmdisplaycomposition.h"
#include "drmhwcomposer.h"

#include <hardware/hardware.h>
#include <hardware/hwcomposer.h>

#include <map>
#include <vector>
#include <stack>

#define UNUSED(x) (void)(x)

namespace android {

class DrmDevice;
class DrmPlanes;

typedef struct tagPlaneGroup{
	bool     b_reserved;
	bool     bUse;
	uint32_t zpos;
	uint32_t possible_crtcs;
	uint64_t share_id;

  // RK356x support dynamic switching
  uint32_t current_crtc_mask;
  uint32_t necessary_wait_cnt;
  std::stack<uint32_t> last_crtc_mask;
	std::vector<DrmPlane*> planes;


  bool set_current_crtc( uint32_t crtc_mask ){
    ALOGD_IF(LogLevel(DBG_INFO),"set_current_crtc = %x, current_crtc_mask=%x, possible_crtcs=%" PRIx32,
             crtc_mask,current_crtc_mask,possible_crtcs);
    if(!(possible_crtcs & crtc_mask))
      return false;
    last_crtc_mask.push(current_crtc_mask);
    current_crtc_mask = crtc_mask;
    necessary_wait_cnt = 0;
    return true;
  }

   void reset_current_crtc(){
    if(last_crtc_mask.size()>0){
      current_crtc_mask = last_crtc_mask.top();
      ALOGD_IF(LogLevel(DBG_INFO),"reset_current_crtc, reset current_crtc_mask to %x",current_crtc_mask);
      last_crtc_mask.pop();
    }
    return;
  }

  bool match_crtc(uint32_t crtc_mask){
    ALOGD_IF(LogLevel(DBG_INFO),"Match_crtc: PlaneGroup share_id = %" PRIu64 ", current_crtc_mask=%x ,"
             "crtc_mask=%x, possible_crtcs=%" PRIx32,share_id,current_crtc_mask,crtc_mask,possible_crtcs);
    if(!(possible_crtcs & crtc_mask))
      return false;
    if(crtc_mask & current_crtc_mask){
       if(necessary_wait_cnt < 3){
         necessary_wait_cnt++;
         return false;
       }else{
         return true;
       }
    }else{
       return false;
    }
 }
}PlaneGroup;

class Importer {
 public:
  virtual ~Importer() {
  }

  // Creates a platform-specific importer instance
  static Importer *CreateInstance(DrmDevice *drm);

  // Imports the buffer referred to by handle into bo.
  //
  // Note: This can be called from a different thread than ReleaseBuffer. The
  //       implementation is responsible for ensuring thread safety.
  virtual int ImportBuffer(buffer_handle_t handle, hwc_drm_bo_t *bo) = 0;

  // Releases the buffer object (ie: does the inverse of ImportBuffer)
  //
  // Note: This can be called from a different thread than ImportBuffer. The
  //       implementation is responsible for ensuring thread safety.
  virtual int ReleaseBuffer(hwc_drm_bo_t *bo) = 0;

  // Checks if importer can import the buffer.
  virtual bool CanImportBuffer(buffer_handle_t handle) = 0;
};

class Planner {
 public:
  class PlanStage {
   public:
    virtual ~PlanStage() {
    }

    virtual int TryHwcPolicy(std::vector<DrmCompositionPlane> *composition,
                                std::vector<DrmHwcLayer*> &layers,
                                DrmCrtc *crtc) = 0;
    virtual int MatchPlanes(std::vector<DrmCompositionPlane> *composition,
                                std::vector<DrmHwcLayer*> &layers,
                                DrmCrtc *crtc,
                                std::vector<PlaneGroup *> &plane_groups) = 0;
   protected:

    // Inserts the given layer:plane in the composition at the back
    virtual int MatchPlane(std::vector<DrmCompositionPlane> *composition_planes,
                     std::vector<PlaneGroup *> &plane_groups,
                     DrmCompositionPlane::Type type, DrmCrtc *crtc,
                     std::pair<int, std::vector<DrmHwcLayer*>> layers, int *zpos) = 0;
  };

  // Creates a planner instance with platform-specific planning stages
  static std::unique_ptr<Planner> CreateInstance(DrmDevice *drm);

  // Takes a stack of layers and provisions hardware planes for them. If the
  // entire stack can't fit in hardware, FIXME
  //
  // @layers: a map of index:layer of layers to composite
  // @primary_planes: a vector of primary planes available for this frame
  // @overlay_planes: a vector of overlay planes available for this frame
  //
  // Returns: A tuple with the status of the operation (0 for success) and
  //          a vector of the resulting plan (ie: layer->plane mapping).
  std::tuple<int, std::vector<DrmCompositionPlane>> TryHwcPolicy(
      std::vector<DrmHwcLayer*> &layers, DrmCrtc *crtc);

  template <typename T, typename... A>
  void AddStage(A &&... args) {
    stages_.emplace_back(
        std::unique_ptr<PlanStage>(new T(std::forward(args)...)));
  }

 private:

  std::vector<std::unique_ptr<PlanStage>> stages_;
};
}  // namespace android
#endif
