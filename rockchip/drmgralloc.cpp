/*
 * Copyright (C) 2018 Fuzhou Rockchip Electronics Co.Ltd.
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

// #define ENABLE_DEBUG_LOG
#define LOG_TAG "drm_hwc2_gralloc"
#include "rockchip/drmgralloc.h"

#include <log/log.h>
#include <inttypes.h>
#include <errno.h>
namespace android {

DrmGralloc::DrmGralloc(){
  int ret = hw_get_module(GRALLOC_HARDWARE_MODULE_ID,
                         (const hw_module_t **)&gralloc_);
  if(ret)
    ALOGE("hw_get_module fail");

}

DrmGralloc::~DrmGralloc(){}
int DrmGralloc::hwc_get_handle_width(buffer_handle_t hnd)
{
#if RK_PER_MODE
    struct gralloc_drm_handle_t* drm_hnd = (struct gralloc_drm_handle_t *)hnd;

    UN_USED(gralloc_);
    return drm_hnd->width;
#else
    int ret = 0;
    int op = GRALLOC_MODULE_PERFORM_GET_HADNLE_WIDTH;
    int width = -1;

    if(gralloc_ && gralloc_->perform)
        ret = gralloc_->perform(gralloc_, op, hnd, &width);
    else
        ret = -EINVAL;

    if(ret != 0)
    {
        ALOGE("%s:cann't get value from gralloc", __FUNCTION__);
    }

    return width;
#endif
}

int DrmGralloc::hwc_get_handle_height(buffer_handle_t hnd)
{
#if RK_PER_MODE
    struct gralloc_drm_handle_t* drm_hnd = (struct gralloc_drm_handle_t *)hnd;

    UN_USED(gralloc_);
    return drm_hnd->height;
#else
    int ret = 0;
    int op = GRALLOC_MODULE_PERFORM_GET_HADNLE_HEIGHT;
    int height = -1;

    if(gralloc_ && gralloc_->perform)
        ret = gralloc_->perform(gralloc_, op, hnd, &height);
    else
        ret = -EINVAL;

    if(ret != 0)
    {
        ALOGE("%s:cann't get value from gralloc", __FUNCTION__);
    }

    return height;
#endif
}

int DrmGralloc::hwc_get_handle_stride(buffer_handle_t hnd)
{
#if RK_PER_MODE
    struct gralloc_drm_handle_t* drm_hnd = (struct gralloc_drm_handle_t *)hnd;

    UN_USED(gralloc_);
    return drm_hnd->pixel_stride;
#else
    int ret = 0;
    int op = GRALLOC_MODULE_PERFORM_GET_HADNLE_STRIDE;
    int stride = -1;

    if(gralloc_ && gralloc_->perform)
        ret = gralloc_->perform(gralloc_, op, hnd, &stride);
    else
        ret = -EINVAL;

    if(ret != 0)
    {
        ALOGE("%s:cann't get value from gralloc", __FUNCTION__);
    }

    return stride;
#endif
}

int DrmGralloc::hwc_get_handle_byte_stride(buffer_handle_t hnd)
{
#if RK_PER_MODE
    struct gralloc_drm_handle_t* drm_hnd = (struct gralloc_drm_handle_t *)hnd;

    UN_USED(gralloc_);
    return drm_hnd->stride;
#else
    int ret = 0;
    int op = GRALLOC_MODULE_PERFORM_GET_HADNLE_BYTE_STRIDE;
    int byte_stride = -1;

    if(gralloc_ && gralloc_->perform)
        ret = gralloc_->perform(gralloc_, op, hnd, &byte_stride);
    else
        ret = -EINVAL;

    if(ret != 0)
    {
        ALOGE("%s:cann't get value from gralloc", __FUNCTION__);
    }

    return byte_stride;
#endif
}

int DrmGralloc::hwc_get_handle_format(buffer_handle_t hnd)
{
#if RK_PER_MODE
    struct gralloc_drm_handle_t* drm_hnd = (struct gralloc_drm_handle_t *)hnd;

    UN_USED(gralloc_);
    return drm_hnd->format;
#else
    int ret = 0;
    int op = GRALLOC_MODULE_PERFORM_GET_HADNLE_FORMAT;
    int format = -1;

    if(gralloc_ && gralloc_->perform)
        ret = gralloc_->perform(gralloc_, op, hnd, &format);
    else
        ret = -EINVAL;

    if(ret != 0)
    {
        ALOGE("%s:cann't get value from gralloc", __FUNCTION__);
    }

    return format;
#endif
}

int DrmGralloc::hwc_get_handle_usage(buffer_handle_t hnd)
{
#if RK_PER_MODE
    struct gralloc_drm_handle_t* drm_hnd = (struct gralloc_drm_handle_t *)hnd;

    UN_USED(gralloc_);
    return drm_hnd->usage;
#else
    int ret = 0;
    int op = GRALLOC_MODULE_PERFORM_GET_USAGE;
    int usage = -1;

    if(gralloc_ && gralloc_->perform)
        ret = gralloc_->perform(gralloc_, op, hnd, &usage);
    else
        ret = -EINVAL;

    if(ret != 0)
    {
        ALOGE("%s:cann't get value from gralloc", __FUNCTION__);
    }

    return usage;
#endif
}

int DrmGralloc::hwc_get_handle_size(buffer_handle_t hnd)
{
#if RK_PER_MODE
    struct gralloc_drm_handle_t* drm_hnd = (struct gralloc_drm_handle_t *)hnd;

    UN_USED(gralloc_);
    return drm_hnd->size;
#else
    int ret = 0;
    int op = GRALLOC_MODULE_PERFORM_GET_HADNLE_SIZE;
    int size = -1;

    if(gralloc_ && gralloc_->perform)
        ret = gralloc_->perform(gralloc_, op, hnd, &size);
    else
        ret = -EINVAL;

    if(ret != 0)
    {
        ALOGE("%s:cann't get value from gralloc", __FUNCTION__);
    }

    return size;
#endif
}

/*
@func hwc_get_handle_attributes:get attributes from handle.Before call this api,As far as now,
    we need register the buffer first.May be the register is good for processer I think

@param hnd:
@param attrs: if size of attrs is small than 5,it will return EINVAL else
    width  = attrs[0]
    height = attrs[1]
    stride = attrs[2]
    format = attrs[3]
    size   = attrs[4]
*/
int DrmGralloc::hwc_get_handle_attributes(buffer_handle_t hnd, std::vector<int> *attrs)
{
    int ret = 0;
    int op = GRALLOC_MODULE_PERFORM_GET_HADNLE_ATTRIBUTES;

    if (!hnd)
        return -EINVAL;

    if(gralloc_ && gralloc_->perform)
    {
        ret = gralloc_->perform(gralloc_, op, hnd, attrs);
    }
    else
    {
        ret = -EINVAL;
    }


    if(ret) {
       ALOGE("hwc_get_handle_attributes fail %d for:%s hnd=%p",ret,strerror(ret),hnd);
    }

    return ret;
}

int DrmGralloc::hwc_get_handle_attibute(buffer_handle_t hnd, attribute_flag_t flag)
{
    std::vector<int> attrs;
    int ret=0;

    if(!hnd)
    {
        ALOGE("%s handle is null",__FUNCTION__);
        return -1;
    }

    ret = hwc_get_handle_attributes(hnd, &attrs);
    if(ret < 0)
    {
        ALOGE("getHandleAttributes fail %d for:%s",ret,strerror(ret));
        return ret;
    }
    else
    {
        return attrs.at(flag);
    }
}

/*
@func getHandlePrimeFd:get prime_fd  from handle.Before call this api,As far as now, we
    need register the buffer first.May be the register is good for processer I think

@param hnd:
@return fd: prime_fd. and driver can call the dma_buf_get to get the buffer

*/
int DrmGralloc::hwc_get_handle_primefd(buffer_handle_t hnd)
{
#if RK_PER_MODE
    struct gralloc_drm_handle_t* drm_hnd = (struct gralloc_drm_handle_t *)hnd;

    UN_USED(gralloc_);
    return drm_hnd->prime_fd;
#else
    int ret = 0;
    int op = GRALLOC_MODULE_PERFORM_GET_HADNLE_PRIME_FD;
    int fd = -1;

    if(gralloc_ && gralloc_->perform)
        ret = gralloc_->perform(gralloc_, op, hnd, &fd);
    else
        ret = -EINVAL;

    if(ret != 0)
    {
        ALOGE("%s:cann't get value from gralloc", __FUNCTION__);
    }

    return fd;
#endif
}

uint32_t DrmGralloc::hwc_get_handle_phy_addr(buffer_handle_t hnd)
{
#if RK_PER_MODE
    struct gralloc_drm_handle_t* drm_hnd = (struct gralloc_drm_handle_t *)hnd;

    UN_USED(gralloc_);
    return drm_hnd->phy_addr;
#else
    int ret = 0;
    int op = GRALLOC_MODULE_PERFORM_GET_HADNLE_PHY_ADDR;
    uint32_t phy_addr = 0;

    if(gralloc_ && gralloc_->perform)
        ret = gralloc_->perform(gralloc_, op, hnd, &phy_addr);
    else
        ret = -EINVAL;

    if(ret != 0)
    {
        ALOGE("%s:cann't get value from gralloc", __FUNCTION__);
    }

    return phy_addr;
#endif
}

uint64_t DrmGralloc::hwc_get_handle_internal_format(buffer_handle_t hnd)
{

    int ret = 0;
    int op = GRALLOC_MODULE_PERFORM_GET_INTERNAL_FORMAT;
    uint64_t uInternalFormat_ = 0;

    if(gralloc_ && gralloc_->perform)
        ret = gralloc_->perform(gralloc_, op, hnd, &uInternalFormat_);
    else
        ret = -EINVAL;

    if(ret != 0)
    {
        ALOGE("%s:cann't get value from gralloc", __FUNCTION__);
    }

    return uInternalFormat_;
}

}
