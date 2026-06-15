/*
 * SPDX-License-Identifier: CC0-1.0
 *
 * app_video: thin V4L2 capture wrapper for the OV5647 on the WT99P4C5-S1.
 *
 * Derived from the p4_camera_stream reference; trimmed to just what
 * p4_face_stream needs and pinned to RGB565 so downstream face AI and
 * JPEG annotation share one frame format.
 */
#ifndef APP_VIDEO_H
#define APP_VIDEO_H

#include "esp_err.h"
#include "linux/videodev2.h"
#include "esp_video_device.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_VIDEO_FMT_RAW8   = V4L2_PIX_FMT_SBGGR8,
    APP_VIDEO_FMT_RAW10  = V4L2_PIX_FMT_SBGGR10,
    APP_VIDEO_FMT_GREY   = V4L2_PIX_FMT_GREY,
    APP_VIDEO_FMT_RGB565 = V4L2_PIX_FMT_RGB565,
    APP_VIDEO_FMT_RGB888 = V4L2_PIX_FMT_RGB24,
    APP_VIDEO_FMT_YUV422 = V4L2_PIX_FMT_YUV422P,
    APP_VIDEO_FMT_YUV420 = V4L2_PIX_FMT_YUV420,
} video_fmt_t;

/* The CSI node on the WT99P4C5-S1 — see p4_camera_stream/main/main.c. */
#define EXAMPLE_CAM_DEV_PATH    (ESP_VIDEO_MIPI_CSI_DEVICE_NAME)
#define EXAMPLE_CAM_BUF_NUM     (4)

/* Single canonical pixel format for the whole pipeline. */
#define APP_VIDEO_FMT           (APP_VIDEO_FMT_RGB565)

/* Per-frame callback. Runs on the capture task. */
typedef void (*app_video_frame_operation_cb_t)(uint8_t *camera_buf,
                                               uint8_t camera_buf_index,
                                               uint32_t camera_buf_hes,
                                               uint32_t camera_buf_ves,
                                               size_t   camera_buf_len);

esp_err_t app_video_main(i2c_master_bus_handle_t i2c_bus_handle);
int       app_video_open(char *dev, video_fmt_t init_fmt);
esp_err_t app_video_set_bufs(int video_fd, uint32_t fb_num, const void **fb);
esp_err_t app_video_get_bufs(int fb_num, void **fb);
uint32_t  app_video_get_buf_size(void);
uint32_t  app_video_get_width(void);
uint32_t  app_video_get_height(void);

esp_err_t app_video_stream_task_start(int video_fd, int core_id);
esp_err_t app_video_stream_task_stop(int video_fd);
esp_err_t app_video_register_frame_operation_cb(app_video_frame_operation_cb_t operation_cb);
esp_err_t app_video_stream_wait_stop(void);

#ifdef __cplusplus
}
#endif
#endif /* APP_VIDEO_H */
