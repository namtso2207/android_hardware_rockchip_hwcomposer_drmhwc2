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
#ifndef ANDROID_DRM_VOP_H_
#define ANDROID_DRM_VOP_H_

#include "platform.h"
#include "drmdevice.h"

namespace android {
class DrmDevice;

typedef std::map<int, std::vector<DrmHwcLayer*>> LayerMap;

typedef enum tagComposeMode
{
    HWC_OVERLAY_LOPICY,
    HWC_MIX_LOPICY,
    HWC_GLES_POLICY,
    HWC_RGA_OVERLAY_LOPICY,
    HWC_SKIP_LOPICY,
    HWC_3D_LOPICY,
    HWC_DEBUG_POLICY
}ComposeMode;


// This plan stage places as many layers on dedicated planes as possible (first
// come first serve), and then sticks the rest in a precomposition plane (if
// needed).
class PlanStageVop : public Planner::PlanStage {
 public:
  int TryHwcPolicy(std::vector<DrmCompositionPlane> *composition,
                        std::vector<DrmHwcLayer*> &layers, DrmCrtc *crtc,
                        std::vector<DrmPlane *> *planes);
  int TryOverlayPolicy(std::vector<DrmCompositionPlane> *composition,
                        std::vector<DrmHwcLayer*> &layers, DrmCrtc *crtc,
                        std::vector<PlaneGroup *> &plane_groups);
  int TryMixPolicy(std::vector<DrmCompositionPlane> *composition,
                        std::vector<DrmHwcLayer*> &layers, DrmCrtc *crtc,
                        std::vector<PlaneGroup *> &plane_groups);
  int TryGLESPolicy(std::vector<DrmCompositionPlane> *composition,
                        std::vector<DrmHwcLayer*> &layers, DrmCrtc *crtc,
                        std::vector<PlaneGroup *> &plane_groups);
  int MatchPlanes(std::vector<DrmCompositionPlane> *composition,
                      std::vector<DrmHwcLayer*> &layers, DrmCrtc *crtc,
                      std::vector<PlaneGroup *> &plane_groups);

 protected:
  bool HasLayer(std::vector<DrmHwcLayer*>& layer_vector,DrmHwcLayer *layer);
  int  IsXIntersect(hwc_rect_t* rec,hwc_rect_t* rec2);
  bool IsRec1IntersectRec2(hwc_rect_t* rec1, hwc_rect_t* rec2);
  bool IsLayerCombine(DrmHwcLayer *layer_one,DrmHwcLayer *layer_two);
  bool HasGetNoAfbcUsablePlanes(DrmCrtc *crtc, std::vector<PlaneGroup *> &plane_groups);
  bool HasGetNoYuvUsablePlanes(DrmCrtc *crtc, std::vector<PlaneGroup *> &plane_groups);
  bool HasGetNoScaleUsablePlanes(DrmCrtc *crtc, std::vector<PlaneGroup *> &plane_groups);
  bool HasGetNoAlphaUsablePlanes(DrmCrtc *crtc, std::vector<PlaneGroup *> &plane_groups);
  bool HasGetNoEotfUsablePlanes(DrmCrtc *crtc, std::vector<PlaneGroup *> &plane_groups);
  bool GetCrtcSupported(const DrmCrtc &crtc, uint32_t possible_crtc_mask);
  bool HasPlanesWithSize(DrmCrtc *crtc, int layer_size, std::vector<PlaneGroup *> &plane_groups);
  int  CombineLayer(LayerMap& layer_map,std::vector<DrmHwcLayer*>& layers,uint32_t iPlaneSize);
  int  GetPlaneGroups(DrmCrtc *crtc,std::vector<DrmPlane *> *planes,std::vector<PlaneGroup *>&out_plane_groups);

  void ResetLayerFromTmp(std::vector<DrmHwcLayer*>& layers, std::vector<DrmHwcLayer*>& tmp_layers);
  void MoveFbToTmp(std::vector<DrmHwcLayer*>& layers,std::vector<DrmHwcLayer*>& tmp_layers);
  void ResetLayerMatch(std::vector<DrmHwcLayer*>& layers);
  int  MatchPlane(std::vector<DrmCompositionPlane> *composition_planes,
                     std::vector<PlaneGroup *> &plane_groups,
                     DrmCompositionPlane::Type type, DrmCrtc *crtc,
                     std::pair<int, std::vector<DrmHwcLayer*>> layers, int *zpos);
 private:
  std::set<ComposeMode> setHwcPolicy;

};

}  // namespace android
#endif

