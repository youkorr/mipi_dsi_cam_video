#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

#include "esphome/components/mipi_dsi_cam_video/mipi_dsi_cam_video.h"

// Include LVGL
extern "C" {
  #include "lvgl.h"
}

namespace esphome {
namespace lvgl_camera_display {

// Mode d'affichage
enum class DisplayMode {
  FIT,        // Ajuster à l'écran (peut déformer)
  FILL,       // Remplir l'écran (peut couper)
  CENTER,     // Centrer sans déformer
  STRETCH     // Étirer pour remplir
};

class LVGLCameraDisplay : public Component {
public:
  LVGLCameraDisplay();
  ~LVGLCameraDisplay();
  
  // Méthodes ESPHome
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }
  
  // === Configuration ===
  void set_camera(mipi_dsi_cam_video::MipiDsiCamVideo *camera) { 
    camera_ = camera; 
  }
  
  void set_parent_obj(lv_obj_t *parent) { 
    parent_obj_ = parent; 
  }
  
  void set_update_interval(uint32_t interval) { 
    update_interval_ = interval; 
  }
  
  void set_display_mode(DisplayMode mode) { 
    display_mode_ = mode; 
  }
  
  void set_rotation(uint16_t rotation) {
    rotation_ = rotation;
  }
  
  void set_auto_start(bool enable) {
    auto_start_ = enable;
  }
  
  // === Contrôle ===
  bool start_display();
  bool stop_display();
  bool is_displaying() const { return displaying_; }
  
  void pause();
  void resume();
  bool is_paused() const { return paused_; }
  
  // === Widget LVGL ===
  lv_obj_t* get_image_obj() { return img_obj_; }
  lv_obj_t* create_widget(lv_obj_t *parent = nullptr);
  void destroy_widget();
  
  // === Statistiques ===
  uint32_t get_displayed_frames() const { return displayed_frames_; }
  float get_display_fps() const { return display_fps_; }
  uint32_t get_dropped_frames() const { return dropped_frames_; }
  
  // === Snapshot ===
  bool save_snapshot(const std::string &filename);

protected:
  // Configuration
  mipi_dsi_cam_video::MipiDsiCamVideo *camera_{nullptr};
  lv_obj_t *parent_obj_{nullptr};
  uint32_t update_interval_{33};  // ~30 FPS par défaut
  DisplayMode display_mode_{DisplayMode::FIT};
  uint16_t rotation_{0};
  bool auto_start_{true};
  
  // État
  bool initialized_{false};
  bool displaying_{false};
  bool paused_{false};
  
  // LVGL objects
  lv_obj_t *container_obj_{nullptr};
  lv_obj_t *img_obj_{nullptr};
  lv_img_dsc_t img_dsc_;
  
  // Buffer pour conversion si nécessaire
  uint8_t *converted_buffer_{nullptr};
  size_t converted_buffer_size_{0};
  
  // Timing
  uint32_t last_update_time_{0};
  uint32_t last_fps_calc_time_{0};
  
  // Statistiques
  uint32_t displayed_frames_{0};
  uint32_t frames_since_last_calc_{0};
  float display_fps_{0.0f};
  uint32_t dropped_frames_{0};
  
  // Méthodes internes
  void update_display_();
  void calculate_scaling_();
  void update_fps_();
  bool convert_format_if_needed_(const uint8_t *data, size_t size);
  
  // Dimensions calculées
  int16_t display_x_{0};
  int16_t display_y_{0};
  lv_coord_t display_width_{0};
  lv_coord_t display_height_{0};
};

}  // namespace lvgl_camera_display
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4




