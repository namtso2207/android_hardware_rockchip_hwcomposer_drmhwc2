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

#include <log/log.h>

namespace android {
std::tuple<int, std::vector<DrmCompositionPlane>> Planner::TryHwcPolicy(
    std::vector<DrmHwcLayer*> &layers, DrmCrtc *crtc) {
  std::vector<DrmCompositionPlane> composition;

  // Go through the provisioning stages and provision planes
  for (auto &i : stages_) {
    int ret = i->TryHwcPolicy(&composition, layers, crtc);
    if (ret) {
      ALOGE("Failed provision stage with ret %d", ret);
      return std::make_tuple(ret, std::vector<DrmCompositionPlane>());
    }
  }

  return std::make_tuple(0, std::move(composition));
}

}  // namespace android
