# ESP32-P4 IMX462 driver

Sony IMX462 (STARVIS) camera sensor driver for the ESP32-P4, built on Espressif's `esp_cam_sensor` and `esp_video`. The sensor streams RAW10 over 2-lane MIPI-CSI and the P4 ISP turns it into RGB or YUV.

The IMX462 is a low-light sensor with strong near-infrared response. It has no internal auto exposure, so exposure and gain are run by the P4 ISP's AGC using the tuning in `cfg/imx462.json`.

---

## Features

| | |
|---|---|
| Formats | 1280x960 (4:3 centre crop), 1920x1080 (16:9 full sensor) |
| Output | RAW10, 2 lanes, 445.5 Mbps per lane |
| Frame rate | 15 fps default, down to 1.9 fps in night levels |
| Exposure | Up to 67 ms by default, up to 533 ms at night level 3 |
| Gain | 0 to 69 dB in 0.3 dB steps, capped in Kconfig |
| Exposure profiles | 4 metering modes x 5 EV steps |
| Other | HCG mode, vertical flip, horizontal mirror, colour bar test pattern |

---

## Hardware

- ESP32-P4 board with a MIPI-CSI connector
- IMX462 module with its own 74.25 MHz crystal (Raspberry Pi style 15-pin modules). No XCLK is needed from the P4.
- SCCB (I2C) address `0x1A`

Tested on a Waveshare ESP32-P4-WIFI6. That board has a 22-pin 0.5 mm CSI connector, so a 22-pin to 15-pin camera cable is needed. A display cable of the same size will not work.

---

## Installing

Clone into your project's `components` folder:

```bash
cd your_project/components
git clone https://github.com/SAMS0N1TE/esp32p4-imx462 imx462
```

The project also needs `esp_video` 2.x:

```yaml
# main/idf_component.yml
dependencies:
  espressif/esp_video: "~2.3"
```

Required `sdkconfig` options:

```
CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE=y
CONFIG_ESP_VIDEO_ENABLE_ISP=y
CONFIG_ESP_VIDEO_ENABLE_ISP_VIDEO_DEVICE=y
CONFIG_ESP_VIDEO_ENABLE_ISP_PIPELINE_CONTROLLER=y
```

`project_include.cmake` adds `cfg/imx462.json` to the ISP tuning files automatically. The detect function is registered through a linker section, and `CMakeLists.txt` keeps it from being dropped.

---

## Usage

Set any options before `esp_video_init()`. They are read when the sensor starts, so changing them later needs a restart of the video stack.

```c
#include "esp_video_init.h"
#include "imx462.h"

imx462_set_full_res(false);       // 1280x960, or true for 1920x1080
imx462_set_night_level(0);        // 0..3
imx462_set_profile(1, 2);         // CENTRE metering, 0 EV

const esp_video_init_csi_config_t csi_cfg = {
    .sccb_config = {
        .init_sccb = true,
        .i2c_config = { .port = 0, .scl_pin = 8, .sda_pin = 7 },
        .freq = 100000,
    },
    .reset_pin = -1,
    .pwdn_pin  = -1,
};
const esp_video_init_config_t cfg = { .csi = &csi_cfg };
esp_video_init_with_flags(&cfg, ESP_VIDEO_INIT_FLAGS_MIPI_CSI | ESP_VIDEO_INIT_FLAGS_ISP);
```

Then open `ESP_VIDEO_MIPI_CSI_DEVICE_NAME` and use it as a normal V4L2 capture device. Ask for `V4L2_PIX_FMT_RGB565`, `V4L2_PIX_FMT_RGB24` or a YUV format at the sensor's width and height.

### Required after stream on

The IMX462 sends an embedded data line (CSI-2 data type `0x12`) at the top of every frame. The P4 CSI bridge accepts data types `0x12` to `0x2f` by default, so that line would be treated as image data and push the picture down by one line. Raise the lower bound after `VIDIOC_STREAMON`:

```c
#include "soc/mipi_csi_bridge_struct.h"

MIPI_CSI_BRIDGE.data_type_cfg.data_type_min = 0x13;
```

Do not narrow the upper bound. Setting the window to `0x2b..0x2b` stops all frames.

---

## Configuration

`idf.py menuconfig` > IMX462 camera sensor

| Option | Default | |
|---|---|---|
| `CAMERA_IMX462_BAYER_PATTERN` | 0 (RGGB) | Change if red and blue come out swapped (3) or there is a green or magenta cast (1 or 2) |
| `CAMERA_IMX462_MAX_GAIN_STEP` | 140 (42 dB) | Gain ceiling at night level 0 |
| `CAMERA_IMX462_NIGHT_MAX_GAIN_STEP` | 170 (51 dB) | Gain ceiling at night levels 1 to 3 |
| `CAMERA_IMX462_HCG` | on | High conversion gain: lower noise in low light, less highlight headroom |

---

## Exposure profiles

The ISP reads its AGC tuning once, keyed by sensor name, so each metering mode and EV step is a named profile in `cfg/imx462.json`. `imx462_set_profile()` picks one.

| Meter | |
|---|---|
| 0 AVG | Even weighting |
| 1 CENTRE | Centre and lower frame weighted, favours shadows (default) |
| 2 SPOT | Centre only |
| 3 BACKLIT | Ignores the top of the frame, for bright sky behind a subject |

EV index 0 to 4 is -2 to +2 EV.

To change the tuning, edit `cfg/imx462_base.json` and regenerate:

```bash
python cfg/gen_profiles.py
```

Do not edit `cfg/imx462.json` directly.

---

## Notes

- Black level is set to 0. The Linux driver uses 60 because its ISP subtracts it; the P4 pipeline does not.
- ODBIT is 0 for RAW10. With 1 the output saturates at about half of full scale.
- Software reset is never used. The datasheet prohibits it in CSI-2 mode.
- Stream on does a full standby cycle first. After a warm restart the sensor can otherwise leave its MIPI lanes idle.
- The module also answers at `0x36`. If the OV5647 driver is enabled too, it will log a failed detection there, which is harmless.
- Some modules have no IR cut filter. For normal colour, use a lens with one. For infrared photos, use an IR pass filter.

---

## Known limitations

- RAW12 is not supported. The `esp_video` ISP path only takes RAW8 and RAW10.
- White balance is fully automatic. The ISP applies the AWB result every frame.
- Night levels past 3 are possible in hardware (VMAX up to 262143) but make any preview impractically slow.

---

## Credits

- Register values: Sony IMX327LQR-C datasheet and the Linux `imx290` driver
- Driver framework: Espressif [esp_cam_sensor](https://components.espressif.com/components/espressif/esp_cam_sensor) and [esp_video](https://components.espressif.com/components/espressif/esp_video)

## License

GPL-3.0. See [LICENSE](LICENSE).
