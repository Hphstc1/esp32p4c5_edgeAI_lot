/*
 * SPDX-License-Identifier: CC0-1.0
 *
 * V4L2 capture loop for the OV5647 on the WT99P4C5-S1, with a single
 * user-supplied frame callback. Logic and structure mirror the
 * p4_camera_stream reference so we know it works on this exact board.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include "esp_err.h"
#include "esp_log.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "app_video.h"

static const char *TAG = "app_video";

#define MAX_BUFFER_COUNT  (6)
#define MIN_BUFFER_COUNT  (2)
#define VIDEO_TASK_STACK  (16 * 1024)
#define VIDEO_TASK_PRIO   (3)

typedef enum {
    VIDEO_TASK_DELETE      = BIT(0),
    VIDEO_TASK_DELETE_DONE = BIT(1),
} video_event_id_t;

typedef struct {
    uint8_t *camera_buffer[MAX_BUFFER_COUNT];
    size_t   camera_buf_size;
    uint32_t camera_buf_hes;
    uint32_t camera_buf_ves;
    struct v4l2_buffer v4l2_buf;
    uint8_t  camera_mem_mode;
    app_video_frame_operation_cb_t user_camera_video_frame_operation_cb;
    TaskHandle_t video_stream_task_handle;
    EventGroupHandle_t video_event_group;
} app_video_t;

static app_video_t s_cam;

esp_err_t app_video_main(i2c_master_bus_handle_t i2c_bus_handle)
{
#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
    esp_video_init_csi_config_t csi_config[] = {
        {
            .sccb_config = {
                .init_sccb = true,
                .i2c_config = {
                    .port    = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_PORT,
                    .scl_pin = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_SCL_PIN,
                    .sda_pin = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_SDA_PIN,
                },
                .freq = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_FREQ,
            },
            .reset_pin = CONFIG_EXAMPLE_MIPI_CSI_CAM_SENSOR_RESET_PIN,
            .pwdn_pin  = CONFIG_EXAMPLE_MIPI_CSI_CAM_SENSOR_PWDN_PIN,
        },
    };
    if (i2c_bus_handle != NULL) {
        csi_config[0].sccb_config.init_sccb = false;
        csi_config[0].sccb_config.i2c_handle = i2c_bus_handle;
    }
#endif
    esp_video_init_config_t cam_config = {
#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR > 0
        .csi = csi_config,
#endif
    };
    return esp_video_init(&cam_config);
}

int app_video_open(char *dev, video_fmt_t init_fmt)
{
    struct v4l2_format default_format;
    struct v4l2_capability capability;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    int fd = open(dev, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "Open %s failed: %d", dev, errno);
        return -1;
    }
    if (ioctl(fd, VIDIOC_QUERYCAP, &capability)) {
        ESP_LOGE(TAG, "QUERYCAP failed");
        goto exit_0;
    }
    ESP_LOGI(TAG, "driver=%s card=%s bus=%s ver=%d.%d.%d",
             capability.driver, capability.card, capability.bus_info,
             (uint16_t)(capability.version >> 16),
             (uint8_t)(capability.version >> 8),
             (uint8_t)capability.version);

    memset(&default_format, 0, sizeof(default_format));
    default_format.type = type;
    if (ioctl(fd, VIDIOC_G_FMT, &default_format) != 0) {
        ESP_LOGE(TAG, "G_FMT failed");
        goto exit_0;
    }
    ESP_LOGI(TAG, "sensor reports %" PRIu32 "x%" PRIu32 " pixfmt=0x%" PRIx32,
             default_format.fmt.pix.width,
             default_format.fmt.pix.height,
             (uint32_t)default_format.fmt.pix.pixelformat);

    /* Diagnostic: log the actual bytesperline and sizeimage to verify buffer sizing */
    ESP_LOGI(TAG, "bytesperline=%" PRIu32 " sizeimage=%" PRIu32 " colorspace=%u",
             (uint32_t)default_format.fmt.pix.bytesperline,
             (uint32_t)default_format.fmt.pix.sizeimage,
             (unsigned)default_format.fmt.pix.colorspace);

    s_cam.camera_buf_hes = default_format.fmt.pix.width;
    s_cam.camera_buf_ves = default_format.fmt.pix.height;

    if (default_format.fmt.pix.pixelformat != init_fmt) {
        struct v4l2_format format = {
            .type = type,
            .fmt.pix.width       = default_format.fmt.pix.width,
            .fmt.pix.height      = default_format.fmt.pix.height,
            .fmt.pix.pixelformat = init_fmt,
        };
        if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) {
            ESP_LOGE(TAG, "S_FMT failed");
            goto exit_0;
        }
        ESP_LOGI(TAG, "forced RGB565 %" PRIu32 "x%" PRIu32 " (was 0x%" PRIx32 ")",
                 format.fmt.pix.width, format.fmt.pix.height,
                 (uint32_t)default_format.fmt.pix.pixelformat);
    } else {
        ESP_LOGI(TAG, "default format already matches requested pixfmt=0x%" PRIx32,
                 (uint32_t)default_format.fmt.pix.pixelformat);
    }
    return fd;

exit_0:
    close(fd);
    return -1;
}

esp_err_t app_video_set_bufs(int video_fd, uint32_t fb_num, const void **fb)
{
    if (fb_num > MAX_BUFFER_COUNT || fb_num < MIN_BUFFER_COUNT) {
        ESP_LOGE(TAG, "buffer count %" PRIu32 " out of range", fb_num);
        return ESP_FAIL;
    }
    struct v4l2_requestbuffers req;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    memset(&req, 0, sizeof(req));
    req.count = fb_num;
    req.type  = type;
    s_cam.camera_mem_mode = req.memory = fb ? V4L2_MEMORY_USERPTR : V4L2_MEMORY_MMAP;

    if (ioctl(video_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "REQBUFS failed");
        goto err;
    }
    for (uint32_t i = 0; i < fb_num; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = type;
        buf.memory = req.memory;
        buf.index  = i;
        if (ioctl(video_fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "QUERYBUF failed");
            goto err;
        }
        if (req.memory == V4L2_MEMORY_MMAP) {
            void *mapped = mmap(NULL, buf.length,
                                PROT_READ | PROT_WRITE,
                                MAP_SHARED, video_fd, buf.m.offset);
            if (mapped == (void *)-1) {
                ESP_LOGE(TAG, "mmap failed");
                goto err;
            }
            s_cam.camera_buffer[i] = (uint8_t *)mapped;
        } else {
            if (!fb[i]) {
                ESP_LOGE(TAG, "null frame buffer %" PRIu32, i);
                goto err;
            }
            buf.m.userptr = (unsigned long)fb[i];
            s_cam.camera_buffer[i] = (uint8_t *)fb[i];
        }
        s_cam.camera_buf_size = buf.length;
        ESP_LOGI(TAG, "buf[%u] length=%u offset=%u (expected RGB565 size=%u)",
                 (unsigned)i, (unsigned)buf.length, (unsigned)buf.m.offset,
                 (unsigned)(s_cam.camera_buf_hes * s_cam.camera_buf_ves * 2));
        if (ioctl(video_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "QBUF failed");
            goto err;
        }
    }
    ESP_LOGI(TAG, "allocated %u buffers, each %u bytes, total frame %ux%u",
             (unsigned)fb_num, (unsigned)s_cam.camera_buf_size,
             (unsigned)s_cam.camera_buf_hes, (unsigned)s_cam.camera_buf_ves);
    return ESP_OK;
err:
    close(video_fd);
    return ESP_FAIL;
}

esp_err_t app_video_get_bufs(int fb_num, void **fb)
{
    if (fb_num > MAX_BUFFER_COUNT || fb_num < MIN_BUFFER_COUNT) {
        return ESP_FAIL;
    }
    for (int i = 0; i < fb_num; i++) {
        if (s_cam.camera_buffer[i] == NULL) {
            return ESP_FAIL;
        }
        fb[i] = s_cam.camera_buffer[i];
    }
    return ESP_OK;
}

uint32_t app_video_get_buf_size(void)
{
    return s_cam.camera_buf_hes * s_cam.camera_buf_ves *
           (APP_VIDEO_FMT == APP_VIDEO_FMT_RGB565 ? 2 : 3);
}

uint32_t app_video_get_width(void)  { return s_cam.camera_buf_hes; }
uint32_t app_video_get_height(void) { return s_cam.camera_buf_ves; }

static inline esp_err_t video_receive(int fd)
{
    memset(&s_cam.v4l2_buf, 0, sizeof(s_cam.v4l2_buf));
    s_cam.v4l2_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    s_cam.v4l2_buf.memory = s_cam.camera_mem_mode;
    if (ioctl(fd, VIDIOC_DQBUF, &s_cam.v4l2_buf) != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static inline void video_dispatch(int fd)
{
    s_cam.v4l2_buf.m.userptr = (unsigned long)s_cam.camera_buffer[s_cam.v4l2_buf.index];
    s_cam.v4l2_buf.length    = s_cam.camera_buf_size;
    uint8_t idx = s_cam.v4l2_buf.index;
    s_cam.user_camera_video_frame_operation_cb(
        s_cam.camera_buffer[idx], idx,
        s_cam.camera_buf_hes, s_cam.camera_buf_ves, s_cam.camera_buf_size);
}

static inline esp_err_t video_release(int fd)
{
    if (ioctl(fd, VIDIOC_QBUF, &s_cam.v4l2_buf) != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static inline esp_err_t video_stream_start(int fd)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type)) return ESP_FAIL;
    return ESP_OK;
}

static inline esp_err_t video_stream_stop(int fd)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMOFF, &type)) return ESP_FAIL;
    xEventGroupSetBits(s_cam.video_event_group, VIDEO_TASK_DELETE_DONE);
    return ESP_OK;
}

static void video_stream_task(void *arg)
{
    int fd = *((int *)arg);
    while (1) {
        if (video_receive(fd) != ESP_OK) {
            ESP_LOGE(TAG, "DQBUF failed");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        video_dispatch(fd);
        video_release(fd);
        if (xEventGroupGetBits(s_cam.video_event_group) & VIDEO_TASK_DELETE) {
            xEventGroupClearBits(s_cam.video_event_group, VIDEO_TASK_DELETE);
            video_stream_stop(fd);
            vTaskDelete(NULL);
        }
    }
}

esp_err_t app_video_stream_task_start(int video_fd, int core_id)
{
    if (s_cam.video_event_group == NULL) {
        s_cam.video_event_group = xEventGroupCreate();
    }
    xEventGroupClearBits(s_cam.video_event_group, VIDEO_TASK_DELETE_DONE);
    video_stream_start(video_fd);
    if (xTaskCreatePinnedToCore(video_stream_task, "vid_stream",
                                VIDEO_TASK_STACK, &video_fd,
                                VIDEO_TASK_PRIO,
                                &s_cam.video_stream_task_handle, core_id) != pdPASS) {
        video_stream_stop(video_fd);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t app_video_stream_task_stop(int video_fd)
{
    xEventGroupSetBits(s_cam.video_event_group, VIDEO_TASK_DELETE);
    return ESP_OK;
}

esp_err_t app_video_register_frame_operation_cb(app_video_frame_operation_cb_t cb)
{
    s_cam.user_camera_video_frame_operation_cb = cb;
    return ESP_OK;
}

esp_err_t app_video_stream_wait_stop(void)
{
    xEventGroupWaitBits(s_cam.video_event_group, VIDEO_TASK_DELETE_DONE,
                        pdTRUE, pdTRUE, portMAX_DELAY);
    return ESP_OK;
}
