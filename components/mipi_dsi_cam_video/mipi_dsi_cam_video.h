#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

#include <string>
#include <vector>

extern "C" {
  #include <fcntl.h>
  #include <sys/ioctl.h>
  #include <sys/mman.h>
  #include <unistd.h>
  #include "driver/v4l2_common.h"
  #include "esp_video.h"
}

namespace esphome {
namespace mipi_dsi_cam_video {

// Structures pour les buffers mémoire
struct VideoBuffer {
  void *start;
  size_t length;
  int index;
};

// Formats de pixels supportés
enum class PixelFormat {
  RGB565,
  RGB888,
  YUV422,
  JPEG,
  H264,
  RAW8
};

// Résolutions prédéfinies
struct Resolution {
  uint16_t width;
  uint16_t height;
  const char* name;
};

class MipiDsiCamVideo : public Component {
public:
  MipiDsiCamVideo();
  ~MipiDsiCamVideo();
  
  // Méthodes ESPHome
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  
  // === Configuration ===
  void set_name(const std::string &name) { name_ = name; }
  void set_sensor_name(const std::string &name) { sensor_name_ = name; }
  void set_resolution(uint16_t width, uint16_t height);
  void set_resolution_preset(const std::string &preset);
  void set_pixel_format(PixelFormat format) { pixel_format_ = format; }
  void set_framerate(uint8_t fps) { framerate_ = fps; }
  void set_jpeg_quality(uint8_t quality) { jpeg_quality_ = quality; }
  
  // I2C et horloge externe
  void set_i2c_parent(i2c::I2CComponent *parent) { i2c_parent_ = parent; }
  void set_external_clock_pin(GPIOPin *pin) { external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { external_clock_freq_ = freq; }
  
  // === Streaming ===
  bool start_streaming();
  bool stop_streaming();
  bool is_streaming() const { return streaming_; }
  
  // === Capture ===
  bool capture_frame();
  uint8_t* get_frame_data() { return current_frame_data_; }
  size_t get_frame_size() const { return current_frame_size_; }
  uint16_t get_width() const { return width_; }
  uint16_t get_height() const { return height_; }
  
  // === H.264 ===
  bool enable_h264(bool enable);
  void set_h264_bitrate(uint32_t bitrate) { h264_bitrate_ = bitrate; }
  void set_h264_gop(uint32_t gop) { h264_gop_ = gop; }
  void set_h264_qp_min(uint8_t qp) { h264_qp_min_ = qp; }
  void set_h264_qp_max(uint8_t qp) { h264_qp_max_ = qp; }
  bool get_h264_frame(uint8_t **data, size_t *size);
  
  // === Enregistrement SD ===
  bool start_recording(const std::string &filename);
  bool stop_recording();
  bool is_recording() const { return recording_; }
  
  // === Contrôles Caméra (V4L2) ===
  void set_brightness(int value);
  void set_contrast(int value);
  void set_saturation(int value);
  void set_hue(int value);
  void set_exposure(int value);
  void set_gain(int value);
  void set_white_balance(bool auto_wb);
  void set_white_balance_temp(int temp);
  void set_horizontal_flip(bool enable);
  void set_vertical_flip(bool enable);
  
  // === ISP Advanced ===
  void set_sharpen(uint8_t level);
  void set_denoise(uint8_t level);
  
  // === Statistiques ===
  uint32_t get_frame_count() const { return frame_count_; }
  float get_actual_fps() const { return actual_fps_; }
  uint32_t get_dropped_frames() const { return dropped_frames_; }

protected:
  // Configuration
  std::string name_{"MIPI Camera"};
  std::string sensor_name_{"sc202cs"};
  uint16_t width_{1280};
  uint16_t height_{720};
  PixelFormat pixel_format_{PixelFormat::RGB565};
  uint8_t framerate_{30};
  uint8_t jpeg_quality_{80};
  
  // I2C et horloge externe
  i2c::I2CComponent *i2c_parent_{nullptr};
  GPIOPin *external_clock_pin_{nullptr};
  uint32_t external_clock_freq_{24000000};  // 24 MHz par défaut
  
  // H.264
  bool h264_enabled_{false};
  uint32_t h264_bitrate_{2000000};  // 2 Mbps
  uint32_t h264_gop_{30};
  uint8_t h264_qp_min_{10};
  uint8_t h264_qp_max_{40};
  
  // État
  bool initialized_{false};
  bool streaming_{false};
  bool recording_{false};
  
  // File descriptors V4L2
  int video_fd_{-1};      // /dev/video0 (MIPI-CSI)
  int isp_fd_{-1};        // /dev/video1 (ISP)
  int h264_fd_{-1};       // /dev/video5 (H.264)
  int jpeg_fd_{-1};       // /dev/video6 (JPEG)
  int rec_fd_{-1};        // File descriptor pour l'enregistrement
  
  // Buffers
  std::vector<VideoBuffer> buffers_;
  std::vector<VideoBuffer> h264_buffers_;
  uint8_t *current_frame_data_{nullptr};
  size_t current_frame_size_{0};
  
  // Statistiques
  uint32_t frame_count_{0};
  uint32_t dropped_frames_{0};
  float actual_fps_{0.0f};
  uint32_t last_fps_calc_time_{0};
  uint32_t frames_since_last_calc_{0};
  
  // Méthodes internes
  bool open_video_device_();
  bool configure_video_device_();
  bool setup_buffers_();
  bool setup_h264_();
  bool setup_isp_();
  void cleanup_();
  
  // Helpers V4L2
  bool set_v4l2_control_(int fd, uint32_t id, int32_t value);
  bool get_v4l2_control_(int fd, uint32_t id, int32_t *value);
  uint32_t get_v4l2_pixel_format_(PixelFormat format);
  
  // Recording
  bool write_frame_to_file_(const uint8_t *data, size_t size);
  
  // FPS calculation
  void update_fps_();
};

}  // namespace mipi_dsi_cam_video
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
