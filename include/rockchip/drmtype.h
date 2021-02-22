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

#ifndef _DRM_TYPE_H_
#define _DRM_TYPE_H_
#include <libsystem/include/system/graphics-base-v1.0.h>

#include <hardware/gralloc.h>

#define PROPERTY_TYPE "vendor"

/* hdr usage */
/*usage & 0x0F000000
  0x1000000 bt2020
  0x2000000 st2084
  0x3000000 hlg
  0x4000000 dobly version
 */
#define HDR_ST2084_USAGE                                       0x2000000
#define HDR_HLG_USAGE                                          0x3000000

#define GRALLOC_ARM_INTFMT_EXTENSION_BIT_START          32
/* This format will use AFBC */
#define	GRALLOC_ARM_INTFMT_AFBC                     (1ULL << (GRALLOC_ARM_INTFMT_EXTENSION_BIT_START+0))
#define MAGIC_USAGE_FOR_AFBC_LAYER                      (0x88)

typedef enum DrmHdrType{
    DRM_HWC_DOLBY_VISION = 1,
    DRM_HWC_HDR10 = 2,
    DRM_HWC_HLG = 3,
    DRM_HWC_HDR10_PLUS = 4
}DrmHdrType_t;

class DrmHdr{
public:
    DrmHdr(DrmHdrType_t drm_hdr_type, float out_max_luminance, float out_max_average_luminance, float out_min_luminance){
        drmHdrType = drm_hdr_type;
        outMaxLuminance = out_max_luminance;
        outMaxAverageLuminance = out_max_average_luminance;
        outMinLuminance = out_min_luminance;
    }

    DrmHdrType_t drmHdrType;
    float outMaxLuminance;
    float outMaxAverageLuminance;
    float outMinLuminance;
};


/* see also http://vektor.theorem.ca/graphics/ycbcr/ */
enum v4l2_colorspace {
        /*
         * Default colorspace, i.e. let the driver figure it out.
         * Can only be used with video capture.
         */
        V4L2_COLORSPACE_DEFAULT       = 0,

        /* SMPTE 170M: used for broadcast NTSC/PAL SDTV */
        V4L2_COLORSPACE_SMPTE170M     = 1,

        /* Obsolete pre-1998 SMPTE 240M HDTV standard, superseded by Rec 709 */
        V4L2_COLORSPACE_SMPTE240M     = 2,

        /* Rec.709: used for HDTV */
        V4L2_COLORSPACE_REC709        = 3,

        /*
         * Deprecated, do not use. No driver will ever return this. This was
         * based on a misunderstanding of the bt878 datasheet.
         */
        V4L2_COLORSPACE_BT878         = 4,

        /*
         * NTSC 1953 colorspace. This only makes sense when dealing with
         * really, really old NTSC recordings. Superseded by SMPTE 170M.
         */
        V4L2_COLORSPACE_470_SYSTEM_M  = 5,

        /*
         * EBU Tech 3213 PAL/SECAM colorspace. This only makes sense when
         * dealing with really old PAL/SECAM recordings. Superseded by
         * SMPTE 170M.
         */
        V4L2_COLORSPACE_470_SYSTEM_BG = 6,

        /*
         * Effectively shorthand for V4L2_COLORSPACE_SRGB, V4L2_YCBCR_ENC_601
         * and V4L2_QUANTIZATION_FULL_RANGE. To be used for (Motion-)JPEG.
         */
        V4L2_COLORSPACE_JPEG          = 7,

        /* For RGB colorspaces such as produces by most webcams. */
        V4L2_COLORSPACE_SRGB          = 8,

        /* AdobeRGB colorspace */
        V4L2_COLORSPACE_ADOBERGB      = 9,

        /* BT.2020 colorspace, used for UHDTV. */
        V4L2_COLORSPACE_BT2020        = 10,

        /* Raw colorspace: for RAW unprocessed images */
        V4L2_COLORSPACE_RAW           = 11,

        /* DCI-P3 colorspace, used by cinema projectors */
        V4L2_COLORSPACE_DCI_P3        = 12,
};


/* HDMI output pixel format */
enum drm_hdmi_output_type {
	DRM_HDMI_OUTPUT_DEFAULT_RGB, /* default RGB */
	DRM_HDMI_OUTPUT_YCBCR444, /* YCBCR 444 */
	DRM_HDMI_OUTPUT_YCBCR422, /* YCBCR 422 */
	DRM_HDMI_OUTPUT_YCBCR420, /* YCBCR 420 */
	DRM_HDMI_OUTPUT_YCBCR_HQ, /* Highest subsampled YUV */
	DRM_HDMI_OUTPUT_YCBCR_LQ, /* Lowest subsampled YUV */
	DRM_HDMI_OUTPUT_INVALID, /* Guess what ? */
};

enum dw_hdmi_rockchip_color_depth {
	ROCKCHIP_DEPTH_DEFAULT = 0,
	ROCKCHIP_HDMI_DEPTH_8 = 8,
	ROCKCHIP_HDMI_DEPTH_10 = 10,
};



typedef enum attribute_flag {
    ATT_WIDTH = 0,
    ATT_HEIGHT,
    ATT_STRIDE,
    ATT_FORMAT,
    ATT_SIZE,
    ATT_BYTE_STRIDE,
    ATT_BYTE_STRIDE_WORKROUND
}attribute_flag_t;

/*
 * Base_parameter is used for 3328_8.0  , by libin start.
 */
#define AUTO_BIT_RESET 0x00
#define RESOLUTION_AUTO			(1<<0)
#define COLOR_AUTO				(1<<1)
#define HDCP1X_EN				(1<<2)
#define RESOLUTION_WHITE_EN		(1<<3)
#define SCREEN_LIST_MAX 5
#define DEFAULT_BRIGHTNESS  50
#define DEFAULT_CONTRAST  50
#define DEFAULT_SATURATION  50
#define DEFAULT_HUE  50
#define DEFAULT_OVERSCAN_VALUE 100


struct drm_display_mode {
    /* Proposed mode values */
    int clock;      /* in kHz */
    int hdisplay;
    int hsync_start;
    int hsync_end;
    int htotal;
    int vdisplay;
    int vsync_start;
    int vsync_end;
    int vtotal;
    int vrefresh;
    int vscan;
    unsigned int flags;
    int picture_aspect_ratio;
};

enum output_format {
    output_rgb=0,
    output_ycbcr444=1,
    output_ycbcr422=2,
    output_ycbcr420=3,
    output_ycbcr_high_subsampling=4,  // (YCbCr444 > YCbCr422 > YCbCr420 > RGB)
    output_ycbcr_low_subsampling=5  , // (RGB > YCbCr420 > YCbCr422 > YCbCr444)
    invalid_output=6,
};

enum  output_depth{
    Automatic=0,
    depth_24bit=8,
    depth_30bit=10,
};

struct overscan {
    unsigned int maxvalue;
    unsigned short leftscale;
    unsigned short rightscale;
    unsigned short topscale;
    unsigned short bottomscale;
};

struct hwc_inital_info{
    char device[128];
    unsigned int framebuffer_width;
    unsigned int framebuffer_height;
    float fps;
};

struct bcsh_info {
    unsigned short brightness;
    unsigned short contrast;
    unsigned short saturation;
    unsigned short hue;
};
struct lut_data{
    uint16_t size;
    uint16_t lred[1024];
    uint16_t lgreen[1024];
    uint16_t lblue[1024];
};
struct screen_info {
	  int type;
    struct drm_display_mode resolution;// 52 bytes
    enum output_format  format; // 4 bytes
    enum output_depth depthc; // 4 bytes
    unsigned int feature;     //4 bytes
};


struct disp_info {
	struct screen_info screen_list[SCREEN_LIST_MAX];
  struct overscan scan;//12 bytes
	struct hwc_inital_info hwc_info; //140 bytes
	struct bcsh_info bcsh;
  unsigned int reserve[128];
  struct lut_data mlutdata;/*6k+4*/
};


struct file_base_parameter
{
    struct disp_info main;
    struct disp_info aux;
};

static char const *const device_template[] =
{
    "/dev/block/platform/1021c000.dwmmc/by-name/baseparameter",
    "/dev/block/platform/30020000.dwmmc/by-name/baseparameter",
    "/dev/block/platform/fe330000.sdhci/by-name/baseparameter",
    "/dev/block/platform/ff520000.dwmmc/by-name/baseparameter",
    "/dev/block/platform/ff0f0000.dwmmc/by-name/baseparameter",
    "/dev/block/rknand_baseparameter",
    "/dev/block/by-name/baseparameter",
    "/dev/block/platform/30030000.nandc/by-name/baseparameter",
    NULL,
};

enum flagBaseParameter
{
    BP_UPDATE = 0,
    BP_RESOLUTION,
    BP_FB_SIZE,
    BP_DEVICE,
    BP_COLOR,
    BP_BRIGHTNESS,
    BP_CONTRAST,
    BP_SATURATION,
    BP_HUE,
    BP_OVERSCAN,
};

typedef struct hwc_drm_display {
  bool bStandardSwitchResolution = false;
  int framebuffer_width;
  int framebuffer_height;
  int vrefresh;
  int rel_xres;
  int rel_yres;
  uint32_t dclk;
  uint32_t aclk;
  float w_scale;
  float h_scale;
  int bcsh_timeline;
  int display_timeline;
  int hotplug_timeline;
  bool hdr_mode;
  drm_hdmi_output_type    color_format = DRM_HDMI_OUTPUT_DEFAULT_RGB;
  dw_hdmi_rockchip_color_depth color_depth = ROCKCHIP_HDMI_DEPTH_8;
} hwc_drm_display_t;

uint32_t ConvertHalFormatToDrm(uint32_t hal_format);

#endif
