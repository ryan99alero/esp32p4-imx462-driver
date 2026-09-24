/*
 * imx462.c - Sony IMX462 (STARVIS, IMX290/IMX327 register family) driver for
 * esp_cam_sensor. MIPI-CSI, 2 lanes, RAW10.
 *
 * Register values are from the Sony IMX327LQR-C datasheet (the IMX462 is
 * register compatible) and the Linux imx290 driver, which carries the
 * IMX462-specific init table.
 *
 * Two formats, both window crops centred in the effective pixel array:
 * 1280x960 (4:3) and 1920x1080 (16:9, full sensor). They share the line time,
 * lane rate and MIPI timing; only the window differs.
 *
 * The line time is the 30 fps one. The frame is stretched with VMAX to 15 fps
 * by default, and further in the night levels, which raises the longest
 * exposure the AGC can choose.
 *
 * The module carries its own 74.25 MHz crystal, so no XCLK is needed.
 *
 * The sensor has no internal AEC. Exposure and gain are driven by the P4 ISP's
 * IPA AGC through EXPOSURE_VAL and GAIN, tuned by cfg/imx462.json.
 */

#include <string.h>
#include <math.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "imx462.h"

static const char *TAG = "imx462";

#define IMX462_REG_END      0xffff
#define IMX462_REG_DELAY    0xfffe

/* Chip ID 02h */
#define REG_STANDBY         0x3000
#define REG_REGHOLD         0x3001
#define REG_XMSTA           0x3002
#define REG_SW_RESET        0x3003
#define REG_ADBIT           0x3005
#define REG_WINMODE         0x3007   /* [6:4] window mode, [1] HREVERSE, [0] VREVERSE */
#define REG_FRSEL           0x3009   /* [1:0] FRSEL, [4] FDG_SEL (HCG) */
#define REG_BLKLEVEL_L      0x300a
#define REG_BLKLEVEL_H      0x300b
#define REG_GAIN            0x3014
#define REG_VMAX_L          0x3018
#define REG_HMAX_L          0x301c
#define REG_SHS1_L          0x3020
#define REG_WINWV_OB        0x303a
#define REG_WINPV_L         0x303c
#define REG_WINWV_L         0x303e
#define REG_WINPH_L         0x3040
#define REG_WINWH_L         0x3042
#define REG_ODBIT           0x3046
#define REG_XSOUTSEL        0x304b
/* Chip ID 06h */
#define REG_OPB_SIZE_V      0x3414
#define REG_Y_OUT_SIZE_L    0x3418
#define REG_X_OUT_SIZE_L    0x3472

#define WINMODE_CROP        0x40

/* 30 fps window cropping, 2 lanes (datasheet "List of Setting Register for
 * CSI-2 serial output"): FRSEL = 2, HMAX = 4400, 1H = 29.6 us.
 * VMAX may be changed freely; frame rate = 1 / (VMAX x 1H). */
#define IMX462_HMAX         4400
#define IMX462_VMAX_DAY     2250            /* 15 fps */
#define IMX462_VMAX_NIGHT   4500            /* 7.5 fps */

/* Night levels: VMAX per level and the frame rate it gives. */
static const uint32_t s_level_vmax[] = { IMX462_VMAX_DAY, IMX462_VMAX_NIGHT, 9000, 18000 };
static const uint8_t  s_level_fps[]  = { 15, 7, 3, 1 };
#define IMX462_LEVELS (int)(sizeof(s_level_vmax) / sizeof(s_level_vmax[0]))

static uint32_t s_vmax = IMX462_VMAX_DAY;
static bool     s_night;
/* The IPA profile name reported to esp_video (see cfg/gen_profiles.py). */
static char     s_name[24] = IMX462_SENSOR_NAME;
#define IMX462_VMAX         s_vmax
#define IMX462_PCLK         148500000
#define IMX462_TLINE_NS     29630

/* Effective pixel area. The window must satisfy WINPH + WINWH <= 1944 and
 * WINPV + WINWV <= 1096, with WINPH and WINWH multiples of 4. Even offsets
 * keep the Bayer phase. */
#define IMX462_EFF_W        1944
#define IMX462_EFF_H        1096

/* Integration time = VMAX - (SHS1 + 1) lines, SHS1 in [1, VMAX - 2]. */
#define IMX462_EXP_MIN      1
#define IMX462_EXP_MAX      (s_vmax - 2)
#define IMX462_EXP_DEF      (s_vmax / 2)

/* Black level added to the sensor output. Linux uses 60 because its ISP
 * subtracts it again; the P4 pipeline does not, so 0 is used here. */
#define IMX462_BLKLEVEL     0

#define IMX462_MIPI_BPS     445500000

typedef struct {
    uint16_t reg;
    uint8_t  val;
} imx462_reginfo_t;

#define R16(reg, v)   { (reg), (uint8_t)((v) & 0xff) }, { (uint16_t)((reg) + 1), (uint8_t)(((v) >> 8) & 0xff) }

static const imx462_reginfo_t imx462_common_raw10[] = {
    /* Written in standby. No SW_RESET: the datasheet prohibits it in CSI-2
     * mode. Every register the mode depends on is written here, so a warm
     * restart starts from the same state as a cold one. */
    { REG_STANDBY,  0x01 },
    { REG_XMSTA,    0x01 },
    { IMX462_REG_DELAY, 30 },

    /* Common init */
    { REG_XSOUTSEL, 0x0a },
    { 0x3012, 0x64 },
    { 0x3013, 0x00 },

    /* IMX462 analogue init */
    { 0x300f, 0x00 }, { 0x3010, 0x21 }, { 0x3011, 0x02 }, { 0x3016, 0x09 },
    { 0x3070, 0x02 }, { 0x3071, 0x11 }, { 0x309b, 0x10 }, { 0x309c, 0x22 },
    { 0x30a2, 0x02 }, { 0x30a6, 0x20 }, { 0x30a8, 0x20 }, { 0x30aa, 0x20 },
    { 0x30ac, 0x20 }, { 0x30b0, 0x43 }, { 0x3119, 0x9e }, { 0x311c, 0x1e },
    { 0x311e, 0x08 }, { 0x3128, 0x05 }, { 0x313d, 0x83 }, { 0x3150, 0x03 },
    { 0x317e, 0x00 }, { 0x32b8, 0x50 }, { 0x32b9, 0x10 }, { 0x32ba, 0x00 },
    { 0x32bb, 0x04 }, { 0x32c8, 0x50 }, { 0x32c9, 0x10 }, { 0x32ca, 0x00 },
    { 0x32cb, 0x04 }, { 0x332c, 0xd3 }, { 0x332d, 0x10 }, { 0x332e, 0x0d },
    { 0x3358, 0x06 }, { 0x3359, 0xe1 }, { 0x335a, 0x11 }, { 0x3360, 0x1e },
    { 0x3361, 0x61 }, { 0x3362, 0x10 }, { 0x33b0, 0x50 }, { 0x33b2, 0x1a },
    { 0x33b3, 0x04 },

    /* Timing. VMAX, SHS1 and FDG_SEL are written after this table. */
    { REG_WINMODE, WINMODE_CROP },
    R16(REG_HMAX_L, IMX462_HMAX),

    /* Window position and size are written after this table. */
    { REG_WINWV_OB, 12 },
    { REG_OPB_SIZE_V, 10 },               /* WINWV_OB - 2 */

    /* INCK 74.25 MHz */
    { 0x305c, 0x0c }, { 0x305d, 0x03 }, { 0x305e, 0x10 }, { 0x305f, 0x01 },
    { 0x315e, 0x1b }, { 0x3164, 0x1b }, { 0x3480, 0x92 },
    R16(0x3444, 0x4a40),                  /* EXTCK_FREQ */

    /* 2 lanes, 445.5 Mbps/lane */
    { 0x3405, 0x10 },                     /* REPETITION */
    { 0x3407, 0x01 },                     /* PHYSICAL_LANE_NUM */
    { 0x3443, 0x01 },                     /* CSI_LANE_MODE */
    R16(0x3446, 0x057),                   /* TCLKPOST */
    R16(0x3448, 0x037),                   /* THSZERO */
    R16(0x344a, 0x01f),                   /* THSPREPARE */
    R16(0x344c, 0x01f),                   /* TCLKTRAIL */
    R16(0x344e, 0x01f),                   /* THSTRAIL */
    R16(0x3450, 0x077),                   /* TCLKZERO */
    R16(0x3452, 0x01f),                   /* TCLKPREPARE */
    R16(0x3454, 0x017),                   /* TLPX */

    /* RAW10. ODBIT is 0 as in the Linux driver; with 1 the output saturates
     * at about half of full scale. */
    { REG_ADBIT, 0x00 },
    { REG_ODBIT, 0x00 },
    { 0x3129, 0x1d }, { 0x317c, 0x12 }, { 0x31ec, 0x37 },
    R16(0x3441, 0x0a0a),                  /* CSI_DT_FMT */
    R16(REG_BLKLEVEL_L, IMX462_BLKLEVEL),

    { REG_GAIN, 0x00 },
    { 0x308c, 0x00 },                     /* PGCTRL: test pattern off */

    { IMX462_REG_DELAY, 10 },
    { IMX462_REG_END, 0x00 },
};

static esp_cam_sensor_isp_info_t imx462_isp_info[] = {
    {
        .isp_v1_info = {
            .version    = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk       = IMX462_PCLK,
            .vts        = IMX462_VMAX_DAY,
            .hts        = IMX462_HMAX,
            .tline_ns   = IMX462_TLINE_NS,
            .gain_def   = 0,
            .exp_def    = IMX462_VMAX_DAY / 2,
            .bayer_type = CONFIG_CAMERA_IMX462_BAYER_PATTERN,
        },
    },
};

/* 1920 x 10 bits over 2 lanes at 445.5 Mbps is 21.5 us of the 29.6 us line,
 * so both formats fit the same link settings. */
#define IMX462_FORMAT(w, h) {                                               \
        .name      = "MIPI_2lane_74M25input_RAW10_" #w "x" #h,              \
        .format    = ESP_CAM_SENSOR_PIXFORMAT_RAW10,                        \
        .port      = ESP_CAM_SENSOR_MIPI_CSI,                               \
        .xclk      = 74250000,                                              \
        .width     = (w),                                                   \
        .height    = (h),                                                   \
        .regs      = imx462_common_raw10,                                   \
        .regs_size = sizeof(imx462_common_raw10) / sizeof(imx462_common_raw10[0]), \
        .fps       = 15,                                                    \
        .isp_info  = &imx462_isp_info[0],                                   \
        .mipi_info = {                                                      \
            .mipi_clk     = IMX462_MIPI_BPS,                                \
            .lane_num     = 2,                                              \
            .line_sync_en = false,                                          \
        },                                                                  \
        .reserved  = NULL,                                                  \
    }

static esp_cam_sensor_format_t imx462_formats[] = {
    IMX462_FORMAT(1280, 960),
    IMX462_FORMAT(1920, 1080),
};

/* The format used when esp_video asks for the default (NULL). */
static int s_fmt_index;

/* Gain is one register in 0.3 dB steps, 0 dB to 69 dB (0..230). Up to 27 dB is
 * analogue, above that digital. The AGC gets the table as absolute gain x1000. */
#define IMX462_GAIN_STEPS   231
static uint32_t s_gain_map[IMX462_GAIN_STEPS];
static size_t   s_gain_count;

typedef struct {
    uint32_t exposure;
    uint32_t gain_index;
    uint8_t  flip;          /* bit1 HREVERSE, bit0 VREVERSE */
} imx462_para_t;

#define EXPOSURE_V4L2_UNIT_US   100
#define EXP_LINES_TO_V4L2(v)    ((int32_t)(((double)(v)) * IMX462_TLINE_NS / 1000.0 / EXPOSURE_V4L2_UNIT_US + 0.5))
#define EXP_V4L2_TO_LINES(v)    ((uint32_t)(((double)(v)) * EXPOSURE_V4L2_UNIT_US * 1000.0 / IMX462_TLINE_NS + 0.5))

static esp_err_t imx462_read(esp_sccb_io_handle_t h, uint16_t reg, uint8_t *val)
{
    return esp_sccb_transmit_receive_reg_a16v8(h, reg, val);
}

static esp_err_t imx462_write(esp_sccb_io_handle_t h, uint16_t reg, uint8_t val)
{
    return esp_sccb_transmit_reg_a16v8(h, reg, val);
}

/* FRSEL = 2 (the 30 fps line time) plus FDG_SEL for HCG: higher conversion
 * gain and lower read noise, at the cost of highlight headroom. */
static uint8_t imx462_frsel(void)
{
#if CONFIG_CAMERA_IMX462_HCG
    return 0x12;
#else
    return 0x02;
#endif
}

void imx462_set_profile(int meter, int ev_index)
{
    if (meter < 0 || ev_index < 0) {
        snprintf(s_name, sizeof(s_name), "%s", IMX462_SENSOR_NAME);
    } else {
        snprintf(s_name, sizeof(s_name), "%s_M%d_E%d", IMX462_SENSOR_NAME, meter, ev_index);
    }
}

void imx462_set_night_level(int level)
{
    if (level < 0) {
        level = 0;
    }
    if (level >= IMX462_LEVELS) {
        level = IMX462_LEVELS - 1;
    }
    s_night = level > 0;
    s_vmax = s_level_vmax[level];
    imx462_isp_info[0].isp_v1_info.vts = s_vmax;
    imx462_isp_info[0].isp_v1_info.exp_def = s_vmax / 2;
    for (size_t i = 0; i < sizeof(imx462_formats) / sizeof(imx462_formats[0]); i++) {
        imx462_formats[i].fps = s_level_fps[level];
    }
}

void imx462_set_night_mode(bool on)
{
    imx462_set_night_level(on ? 1 : 0);
}

void imx462_set_full_res(bool on)
{
    s_fmt_index = on ? 1 : 0;
}

/* Window registers for a format, centred in the effective area. */
static esp_err_t imx462_write_window(esp_sccb_io_handle_t h, uint32_t w, uint32_t ht)
{
    const uint32_t ph = ((IMX462_EFF_W - w) / 2) & ~3u;
    const uint32_t pv = ((IMX462_EFF_H - ht) / 2) & ~1u;
    esp_err_t ret = ESP_OK;
    const struct { uint16_t reg; uint32_t v; } win[] = {
        { REG_WINPV_L, pv }, { REG_WINWV_L, ht },
        { REG_WINPH_L, ph }, { REG_WINWH_L, w  },
        { REG_Y_OUT_SIZE_L, ht },           /* = WINWV in CSI-2 */
        { REG_X_OUT_SIZE_L, w  },
    };
    for (size_t i = 0; i < sizeof(win) / sizeof(win[0]); i++) {
        ret |= imx462_write(h, win[i].reg,     win[i].v & 0xff);
        ret |= imx462_write(h, win[i].reg + 1, (win[i].v >> 8) & 0xff);
    }
    ESP_LOGI(TAG, "window %" PRIu32 "x%" PRIu32 " at (%" PRIu32 ", %" PRIu32 ")", w, ht, ph, pv);
    return ret;
}

static esp_err_t imx462_write_array(esp_sccb_io_handle_t h, const imx462_reginfo_t *regs)
{
    esp_err_t ret = ESP_OK;
    for (size_t i = 0; ret == ESP_OK && regs[i].reg != IMX462_REG_END; i++) {
        if (regs[i].reg == IMX462_REG_DELAY) {
            vTaskDelay(pdMS_TO_TICKS(regs[i].val));
        } else {
            ret = imx462_write(h, regs[i].reg, regs[i].val);
        }
    }
    return ret;
}

static esp_err_t imx462_write_exposure(esp_cam_sensor_device_t *dev, uint32_t lines)
{
    imx462_para_t *p = dev->priv;
    lines = MIN(MAX(lines, IMX462_EXP_MIN), IMX462_EXP_MAX);
    const uint32_t shs1 = IMX462_VMAX - lines - 1;
    esp_err_t ret = imx462_write(dev->sccb_handle, REG_SHS1_L, shs1 & 0xff);
    ret |= imx462_write(dev->sccb_handle, REG_SHS1_L + 1, (shs1 >> 8) & 0xff);
    ret |= imx462_write(dev->sccb_handle, REG_SHS1_L + 2, (shs1 >> 16) & 0x03);
    if (ret == ESP_OK) {
        p->exposure = lines;
    }
    return ret;
}

static esp_err_t imx462_write_gain(esp_cam_sensor_device_t *dev, uint32_t index)
{
    imx462_para_t *p = dev->priv;
    index = MIN(index, s_gain_count - 1);
    esp_err_t ret = imx462_write(dev->sccb_handle, REG_GAIN, (uint8_t)index);
    if (ret == ESP_OK) {
        p->gain_index = index;
    }
    return ret;
}

static esp_err_t imx462_write_flip(esp_cam_sensor_device_t *dev)
{
    imx462_para_t *p = dev->priv;
    return imx462_write(dev->sccb_handle, REG_WINMODE, WINMODE_CROP | p->flip);
}

static esp_err_t imx462_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *q)
{
    switch (q->id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        q->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        q->number.minimum = IMX462_EXP_MIN;
        q->number.maximum = IMX462_EXP_MAX;
        q->number.step = 1;
        q->default_value = IMX462_EXP_DEF;
        return ESP_OK;
    case ESP_CAM_SENSOR_EXPOSURE_US:
        q->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        q->number.minimum = MAX(EXP_LINES_TO_V4L2(IMX462_EXP_MIN), 1);
        q->number.maximum = EXP_LINES_TO_V4L2(IMX462_EXP_MAX);
        q->number.step = 1;
        q->default_value = EXP_LINES_TO_V4L2(IMX462_EXP_DEF);
        return ESP_OK;
    case ESP_CAM_SENSOR_GAIN:
        q->type = ESP_CAM_SENSOR_PARAM_TYPE_ENUMERATION;
        q->enumeration.count = s_gain_count;
        q->enumeration.elements = s_gain_map;
        q->default_value = 0;
        return ESP_OK;
    case ESP_CAM_SENSOR_GROUP_EXP_GAIN:
        q->type = ESP_CAM_SENSOR_PARAM_TYPE_U8;
        q->u8.size = sizeof(esp_cam_sensor_gh_exp_gain_t);
        return ESP_OK;
    case ESP_CAM_SENSOR_VFLIP:
    case ESP_CAM_SENSOR_HMIRROR:
        q->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        q->number.minimum = 0;
        q->number.maximum = 1;
        q->number.step = 1;
        q->default_value = 0;
        return ESP_OK;
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t imx462_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    imx462_para_t *p = dev->priv;
    switch (id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        *(uint32_t *)arg = p->exposure;
        return ESP_OK;
    case ESP_CAM_SENSOR_EXPOSURE_US:
        *(uint32_t *)arg = EXP_LINES_TO_V4L2(p->exposure);
        return ESP_OK;
    case ESP_CAM_SENSOR_GAIN:
        *(uint32_t *)arg = p->gain_index;
        return ESP_OK;
    case ESP_CAM_SENSOR_VFLIP:
        *(int *)arg = p->flip & 0x01;
        return ESP_OK;
    case ESP_CAM_SENSOR_HMIRROR:
        *(int *)arg = (p->flip >> 1) & 0x01;
        return ESP_OK;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

static esp_err_t imx462_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    imx462_para_t *p = dev->priv;
    switch (id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
    case ESP_CAM_SENSOR_EXPOSURE_US: {
        /* REGHOLD so the three SHS1 bytes apply on the same frame. */
        const uint32_t v = *(const uint32_t *)arg;
        esp_err_t ret = imx462_write(dev->sccb_handle, REG_REGHOLD, 0x01);
        ret |= imx462_write_exposure(dev, id == ESP_CAM_SENSOR_EXPOSURE_VAL ? v : EXP_V4L2_TO_LINES(v));
        ret |= imx462_write(dev->sccb_handle, REG_REGHOLD, 0x00);
        return ret;
    }
    case ESP_CAM_SENSOR_GAIN:
        return imx462_write_gain(dev, *(const uint32_t *)arg);
    case ESP_CAM_SENSOR_GROUP_EXP_GAIN: {
        const esp_cam_sensor_gh_exp_gain_t *g = arg;
        uint32_t lines;
        if (g->exposure_val != 0) {
            lines = g->exposure_val;
        } else if (g->exposure_us != 0) {
            lines = EXP_V4L2_TO_LINES(g->exposure_us);
        } else {
            return ESP_ERR_INVALID_ARG;
        }
        /* REGHOLD so exposure and gain apply on the same frame. */
        esp_err_t ret = imx462_write(dev->sccb_handle, REG_REGHOLD, 0x01);
        ret |= imx462_write_exposure(dev, lines);
        ret |= imx462_write_gain(dev, g->gain_index);
        ret |= imx462_write(dev->sccb_handle, REG_REGHOLD, 0x00);
        return ret;
    }
    case ESP_CAM_SENSOR_VFLIP:
        p->flip = (p->flip & ~0x01) | (*(const int *)arg ? 0x01 : 0);
        return imx462_write_flip(dev);
    case ESP_CAM_SENSOR_HMIRROR:
        p->flip = (p->flip & ~0x02) | (*(const int *)arg ? 0x02 : 0);
        return imx462_write_flip(dev);
    default:
        ESP_LOGE(TAG, "set id=%" PRIx32 " is not supported", id);
        return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t imx462_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);
    formats->count = sizeof(imx462_formats) / sizeof(imx462_formats[0]);
    formats->format_array = &imx462_formats[0];
    return ESP_OK;
}

static esp_err_t imx462_query_support_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *cap)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, cap);
    cap->fmt_raw = 1;
    return ESP_OK;
}

static esp_err_t imx462_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    imx462_para_t *p = dev->priv;
    if (format == NULL) {
        format = &imx462_formats[s_fmt_index];
    }

    esp_err_t ret = imx462_write_array(dev->sccb_handle, (const imx462_reginfo_t *)format->regs);
    ret |= imx462_write_window(dev->sccb_handle, format->width, format->height);
    ret |= imx462_write(dev->sccb_handle, REG_FRSEL, imx462_frsel());
    ret |= imx462_write(dev->sccb_handle, REG_VMAX_L, s_vmax & 0xff);
    ret |= imx462_write(dev->sccb_handle, REG_VMAX_L + 1, (s_vmax >> 8) & 0xff);
    ret |= imx462_write(dev->sccb_handle, REG_VMAX_L + 2, (s_vmax >> 16) & 0x03);
    if (ret == ESP_OK) {
        ret = imx462_write_exposure(dev, IMX462_EXP_DEF);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set format regs failed");
        return ESP_CAM_SENSOR_ERR_FAILED_SET_FORMAT;
    }
    dev->cur_format = format;
    p->exposure = IMX462_EXP_DEF;
    p->gain_index = 0;
    if (p->flip) {
        imx462_write_flip(dev);
    }
    return ESP_OK;
}

static esp_err_t imx462_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);
    if (dev->cur_format == NULL) {
        return ESP_FAIL;
    }
    memcpy(format, dev->cur_format, sizeof(esp_cam_sensor_format_t));
    return ESP_OK;
}

static esp_err_t imx462_set_stream(esp_cam_sensor_device_t *dev, int enable)
{
    esp_err_t ret;
    if (enable) {
        /* Stop fully first, for longer than one frame. After a warm restart
         * the sensor may still be streaming, and without a clean stop the
         * MIPI lanes can stay idle in LP-11. */
        ret = imx462_write(dev->sccb_handle, REG_XMSTA, 0x01);
        ret |= imx462_write(dev->sccb_handle, REG_STANDBY, 0x01);
        vTaskDelay(pdMS_TO_TICKS(150));
        ret |= imx462_write(dev->sccb_handle, REG_STANDBY, 0x00);
        /* The internal regulator needs 20 ms or more after standby cancel. */
        vTaskDelay(pdMS_TO_TICKS(30));
        ret |= imx462_write(dev->sccb_handle, REG_XMSTA, 0x00);
    } else {
        ret = imx462_write(dev->sccb_handle, REG_STANDBY, 0x01);
        vTaskDelay(pdMS_TO_TICKS(30));
        ret |= imx462_write(dev->sccb_handle, REG_XMSTA, 0x01);
    }
    if (ret == ESP_OK) {
        dev->stream_status = enable;
    }
    ESP_LOGD(TAG, "stream=%d", enable);
    return ret;
}

static esp_err_t imx462_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
{
    esp_cam_sensor_reg_val_t *r;
    uint8_t v;
    esp_err_t ret = ESP_OK;

    switch (cmd) {
    case ESP_CAM_SENSOR_IOC_HW_RESET:
        break;
    case ESP_CAM_SENSOR_IOC_SW_RESET:
        /* Prohibited in CSI-2 mode; standby is the closest safe equivalent. */
        ret = imx462_set_stream(dev, 0);
        break;
    case ESP_CAM_SENSOR_IOC_S_REG:
        r = arg;
        ret = imx462_write(dev->sccb_handle, r->regaddr, r->value);
        break;
    case ESP_CAM_SENSOR_IOC_G_REG:
        r = arg;
        ret = imx462_read(dev->sccb_handle, r->regaddr, &v);
        if (ret == ESP_OK) {
            r->value = v;
        }
        break;
    case ESP_CAM_SENSOR_IOC_S_STREAM:
        ret = imx462_set_stream(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_S_TEST_PATTERN:
        /* PGCTRL: REGEN | THRU | mode 2 (horizontal colour bars). */
        ret = imx462_write(dev->sccb_handle, 0x308c, *(int *)arg ? 0x23 : 0x00);
        break;
    case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
        *(esp_cam_sensor_id_t *)arg = dev->id;
        break;
    default:
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }
    return ret;
}

static esp_err_t imx462_delete(esp_cam_sensor_device_t *dev)
{
    if (dev) {
        free(dev->priv);
        free(dev);
    }
    return ESP_OK;
}

static const esp_cam_sensor_ops_t imx462_ops = {
    .query_para_desc          = imx462_query_para_desc,
    .get_para_value           = imx462_get_para_value,
    .set_para_value           = imx462_set_para_value,
    .query_support_formats    = imx462_query_support_formats,
    .query_support_capability = imx462_query_support_capability,
    .set_format               = imx462_set_format,
    .get_format               = imx462_get_format,
    .priv_ioctl               = imx462_priv_ioctl,
    .del                      = imx462_delete,
};

static void imx462_build_gain_map(void)
{
    const int cap = s_night ? CONFIG_CAMERA_IMX462_NIGHT_MAX_GAIN_STEP
                            : CONFIG_CAMERA_IMX462_MAX_GAIN_STEP;
    const size_t n = MIN((size_t)cap + 1, (size_t)IMX462_GAIN_STEPS);
    for (size_t i = 0; i < n; i++) {
        s_gain_map[i] = (uint32_t)(powf(10.0f, (0.3f * (float)i) / 20.0f) * 1000.0f + 0.5f);
    }
    s_gain_count = n;
}

esp_cam_sensor_device_t *imx462_detect(esp_cam_sensor_config_t *config)
{
    if (config == NULL) {
        return NULL;
    }

    /* There is no ID register. The sensor is identified by an ACK at 0x1A with
     * 16-bit register addressing, and STANDBY and XMSTA reading back as
     * one-bit registers (bits 7:1 fixed to 0). */
    uint8_t standby = 0xff, xmsta = 0xff;
    if (imx462_read(config->sccb_handle, REG_STANDBY, &standby) != ESP_OK ||
        imx462_read(config->sccb_handle, REG_XMSTA, &xmsta) != ESP_OK) {
        return NULL;
    }
    if ((standby & 0xfe) || (xmsta & 0xfe)) {
        ESP_LOGW(TAG, "0x%02x answers but 3000=0x%02x 3002=0x%02x, not an IMX462",
                 IMX462_SCCB_ADDR, standby, xmsta);
        return NULL;
    }

    esp_cam_sensor_device_t *dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    imx462_para_t *p = calloc(1, sizeof(imx462_para_t));
    if (!dev || !p) {
        free(dev);
        free(p);
        return NULL;
    }

    imx462_build_gain_map();

    dev->name        = s_name;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin    = config->xclk_pin;
    dev->reset_pin   = config->reset_pin;
    dev->pwdn_pin    = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops         = &imx462_ops;
    dev->priv        = p;
    dev->cur_format  = &imx462_formats[s_fmt_index];
    dev->id.pid      = IMX462_PID;

    ESP_LOGI(TAG, "detected at 0x%02x, profile %s", IMX462_SCCB_ADDR, s_name);
    ESP_LOGI(TAG, "%ux%u %s: %u fps, exposure up to %.0f ms, gain 0..%.1f dB, %s",
             (unsigned)imx462_formats[s_fmt_index].width,
             (unsigned)imx462_formats[s_fmt_index].height,
             s_night ? "night" : "day", (unsigned)imx462_formats[s_fmt_index].fps,
             (double)(s_vmax - 2) * IMX462_TLINE_NS / 1e6,
             0.3 * (double)(s_gain_count - 1),
             imx462_frsel() & 0x10 ? "HCG" : "LCG");
    return dev;
}

ESP_CAM_SENSOR_DETECT_FN(imx462_detect, ESP_CAM_SENSOR_MIPI_CSI, IMX462_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return imx462_detect(config);
}
