/*
 * imx462.h - Sony IMX462 esp_cam_sensor driver.
 *
 * The settings below are read when esp_video starts the sensor. Set them
 * before esp_video_init(), or before restarting the video stack.
 */
#pragma once

#include <stdbool.h>
#include "esp_cam_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMX462_SCCB_ADDR    0x1A
#define IMX462_SENSOR_NAME  "IMX462"
/* No ID register on this family; a fixed value so G_CHIP_ID has an answer. */
#define IMX462_PID          0x0462

esp_cam_sensor_device_t *imx462_detect(esp_cam_sensor_config_t *config);

/* 1920x1080 (16:9, full sensor) instead of the default 1280x960 (4:3) crop. */
void imx462_set_full_res(bool on);

/* Frame length, and so the longest exposure the AGC may use:
 *   0  15 fps,   67 ms
 *   1  7.5 fps,  133 ms
 *   2  3.75 fps, 267 ms
 *   3  1.9 fps,  533 ms
 * Levels 1 to 3 use CONFIG_CAMERA_IMX462_NIGHT_MAX_GAIN_STEP. */
void imx462_set_night_level(int level);

/* Same as imx462_set_night_level(on ? 1 : 0). */
void imx462_set_night_mode(bool on);

/* Exposure profile from cfg/imx462.json: metering 0..3 (AVG, CENTRE, SPOT,
 * BACKLIT) and compensation index 0..4 (-2..+2 EV). A negative value selects
 * the default profile (CENTRE, 0 EV). */
void imx462_set_profile(int meter, int ev_index);

#ifdef __cplusplus
}
#endif
