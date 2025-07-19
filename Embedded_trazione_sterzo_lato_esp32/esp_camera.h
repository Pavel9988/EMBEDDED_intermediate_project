#ifndef __ESP_CAMERA_H__
#define __ESP_CAMERA_H__

#include "esp_err.h"
#include "esp_log.h"
#include "sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

// Camera frame format
typedef enum {
    CAMERA_PF_RGB565 = 0,
    CAMERA_PF_YUV422,
    CAMERA_PF_GRAYSCALE,
    CAMERA_PF_JPEG,
    CAMERA_PF_RGB888,
} camera_pixelformat_t;

// Camera frame size (resolution)
typedef enum {
    FRAMESIZE_QQVGA = 10,   // 160x120
    FRAMESIZE_QVGA,         // 320x240
    FRAMESIZE_VGA,          // 640x480
    FRAMESIZE_SVGA,         // 800x600
    FRAMESIZE_XGA,          // 1024x768
    FRAMESIZE_SXGA,         // 1280x1024
    FRAMESIZE_UXGA,         // 1600x1200
} camera_framesize_t;

// Camera frame buffer location
typedef enum {
    CAMERA_FB_IN_DRAM,
    CAMERA_FB_IN_PSRAM,
} camera_fb_location_t;

// Camera frame buffer
typedef struct {
    uint8_t *buf;           /*!< Pointer to the frame buffer */
    size_t len;             /*!< Length of the frame buffer */
    size_t width;           /*!< Width of the frame */
    size_t height;          /*!< Height of the frame */
    camera_pixelformat_t format; /*!< Pixel format */
    int64_t timestamp;      /*!< Timestamp */
    camera_fb_location_t fb_location; /*!< Frame buffer location */
} camera_fb_t;

// Camera configuration structure
typedef struct {
    int pin_pwdn;
    int pin_reset;
    int pin_xclk;
    int pin_sccb_sda;
    int pin_sccb_scl;

    int pin_d7;
    int pin_d6;
    int pin_d5;
    int pin_d4;
    int pin_d3;
    int pin_d2;
    int pin_d1;
    int pin_d0;

    int xclk_freq_hz;
    camera_pixelformat_t pixel_format;
    camera_framesize_t frame_size;
    int jpeg_quality;
    int fb_count;
    camera_fb_location_t fb_location;
} camera_config_t;

// Initialize the camera with specified config
esp_err_t esp_camera_init(const camera_config_t *config);

// Deinitialize the camera driver
esp_err_t esp_camera_deinit(void);

// Capture a frame
camera_fb_t *esp_camera_fb_get(void);

// Return frame buffer to the driver
esp_err_t esp_camera_fb_return(camera_fb_t *fb);

// Get sensor handle for advanced configuration
sensor_t *esp_camera_sensor_get(void);

#ifdef __cplusplus
}
#endif

#endif /* __ESP_CAMERA_H__ */
