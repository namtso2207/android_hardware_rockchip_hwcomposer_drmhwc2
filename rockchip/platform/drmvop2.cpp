/*
 * Copyright (C) 2020 Rockchip Electronics Co.Ltd.
 *
 * Modification based on code covered by the Apache License, Version 2.0 (the "License").
 * You may not use this software except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS TO YOU ON AN "AS IS" BASIS
 * AND ANY AND ALL WARRANTIES AND REPRESENTATIONS WITH RESPECT TO SUCH SOFTWARE, WHETHER EXPRESS,
 * IMPLIED, STATUTORY OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY IMPLIED WARRANTIES OF TITLE,
 * NON-INFRINGEMENT, MERCHANTABILITY, SATISFACTROY QUALITY, ACCURACY OR FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.
 *
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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

#include "rockchip/platform/drmvop2.h"
#include "drmdevice.h"

#include <log/log.h>

namespace android {

int PlanStageVop2::ValidatePlane(DrmPlane *plane, DrmHwcLayer *layer) {
  int ret = 0;
  uint64_t blend;

  if ((plane->rotation_property().id() == 0) &&
      layer->transform != DrmHwcTransform::kIdentity) {
    ALOGE("Rotation is not supported on plane %d", plane->id());
    return -EINVAL;
  }

  if (plane->alpha_property().id() == 0 && layer->alpha != 0xffff) {
    ALOGE("Alpha is not supported on plane %d", plane->id());
    return -EINVAL;
  }

  if (plane->blend_property().id() > 0) {
    if ((layer->blending != DrmHwcBlending::kNone) &&
        (layer->blending != DrmHwcBlending::kPreMult)) {
      ALOGE("Blending is not supported on plane %d", plane->id());
      return -EINVAL;
    }
  } else {
    switch (layer->blending) {
      case DrmHwcBlending::kPreMult:
        std::tie(blend, ret) = plane->blend_property().GetEnumValueWithName(
            "Pre-multiplied");
        break;
      case DrmHwcBlending::kCoverage:
        std::tie(blend, ret) = plane->blend_property().GetEnumValueWithName(
            "Coverage");
        break;
      case DrmHwcBlending::kNone:
      default:
        std::tie(blend,
                 ret) = plane->blend_property().GetEnumValueWithName("None");
        break;
    }
    if (ret)
      ALOGE("Expected a valid blend mode on plane %d", plane->id());
  }

  return ret;
}

int PlanStageVop2::Emplace(std::vector<DrmCompositionPlane> *composition,
                   std::vector<DrmPlane *> *planes,
                   DrmCompositionPlane::Type type, DrmCrtc *crtc,
                   std::pair<size_t, DrmHwcLayer *> layer) {

  DrmPlane *plane = PopPlane(planes);
  std::vector<DrmPlane *> unused_planes;
  int ret = -ENOENT;
  while (plane) {
    ret = ValidatePlane(plane, layer.second);
    if (!ret)
      break;
    if (!plane->zpos_property().is_immutable())
      unused_planes.push_back(plane);
    plane = PopPlane(planes);
  }

  if (!ret) {
    composition->emplace_back(type, plane, crtc, layer.first);
    planes->insert(planes->begin(), unused_planes.begin(),
                   unused_planes.end());
  }

  return ret;
}

int PlanStageVop2::MatchPlanes(
    std::vector<DrmCompositionPlane> *composition,
    std::map<size_t, DrmHwcLayer *> &layers, DrmCrtc *crtc,
    std::vector<DrmPlane *> *planes) {

  // Fill up the remaining planes
  for (auto i = layers.begin(); i != layers.end(); i = layers.erase(i)) {
    int ret = Emplace(composition, planes, DrmCompositionPlane::Type::kLayer,
                      crtc, std::make_pair(i->first, i->second));
    // We don't have any planes left
    if (ret == -ENOENT)
      break;
    else if (ret) {
      ALOGE("Failed to emplace layer %zu, dropping it", i->first);
      return ret;
    }
  }

  return 0;
}

}

