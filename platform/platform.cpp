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

#define LOG_TAG "hwc-platform"

#include "platform.h"
#include "drmdevice.h"
#include "rockchip/platform/drmvop3399.h"
#include "rockchip/platform/drmvop356x.h"
#include "rockchip/platform/drmvop3588.h"

#include <log/log.h>

namespace android {

std::unique_ptr<Planner> Planner::CreateInstance(DrmDevice *) {
  std::unique_ptr<Planner> planner(new Planner);
  planner->AddStage<Vop356x>();
  planner->AddStage<Vop3588>();
  planner->AddStage<Vop3399>();
  return planner;
}

std::tuple<int, std::vector<DrmCompositionPlane>> Planner::TryHwcPolicy(
    std::vector<DrmHwcLayer*> &layers,
    std::vector<PlaneGroup *> &plane_groups,
    DrmCrtc *crtc,
    bool gles_policy) {
  std::vector<DrmCompositionPlane> composition;
  int ret = -1;
  // Go through the provisioning stages and provision planes
  for (auto &i : stages_) {
    if(i->SupportPlatform(crtc->get_soc_id())){
      ret = i->TryHwcPolicy(&composition, layers, plane_groups,  crtc, gles_policy);
      if (ret) {
        ALOGE("Failed provision stage with ret %d", ret);
        return std::make_tuple(ret, std::vector<DrmCompositionPlane>());
      }
    }
  }
  return std::make_tuple(ret, std::move(composition));
}

}  // namespace android
