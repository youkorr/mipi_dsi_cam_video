#include "mipi_dsi_cam_video.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

#include <cstring>
#include <sys/stat.h>
#include <errno.h>

namespace esphome {
namespace mipi_dsi_cam_video {

static const char *const TAG = "mipi_dsi_cam_video";

// Résolutions prédéfinies
static const Resolution RESOLUTIONS[] = {
  {640, 480, "VGA"},
  {800, 600, "SVGA"},
  {1280, 720, "720P"},
  {1600, 1200, "UXGA"},
  {1920, 1080, "1080P"},
};

MipiDsiCamVideo::MipiDsiCamVideo() {}

MipiDsiCamVideo::~MipiDsiCamVideo() {
  this->cleanup_();
}

void MipiDsiCamVideo::setup() {
  ESP_LOGI(TAG, "=== Initializing Camera with esp-video ===");
  ESP_LOGI(TAG, "Sensor: %s", this->sensor_name_.c_str());
  ESP_LOGI(TAG, "Resolution: %ux%u @ %u fps", this->width_, this->height_, this->framerate_);
  
  // Ouvrir le device vidéo
  if (!this->open_video_device_()) {
    ESP_LOGE(TAG, "Failed to open video device");
    this->mark_failed();
    return;
  }
  
  // Configurer le device
  if (!this->configure_video_device_()) {
    ESP_LOGE(TAG, "Failed to configure video device");
    this->mark_failed();
    return;
  }
  
  // Setup ISP si disponible
  if (!this->setup_isp_()) {
    ESP_LOGW(TAG, "ISP setup failed or not available");
  }
  
  // Setup H.264 si demandé
  if (this->h264_enabled_) {
    if (!this->setup_h264_()) {
      ESP_LOGE(TAG, "Failed to setup H.264");
      this->h264_enabled_ = false;
    }
  }
  
  // Allouer les buffers
  if (!this->setup_buffers_()) {
    ESP_LOGE(TAG, "Failed to setup buffers");
    this->mark_failed();
    return;
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "=== Camera initialized successfully ===");
}

bool MipiDsiCamVideo::open_video_device_() {
  // Ouvrir /dev/video0 (MIPI-CSI + ISP output)
  this->video_fd_ = open("/dev/video0", O_RDWR | O_NONBLOCK);
  if (this->video_fd_ < 0) {
    ESP_LOGE(TAG, "Cannot open /dev/video0: %s", strerror(errno));
    return false;
  }
  
  // Vérifier les capacités
  struct v4l2_capability cap = {};
  if (ioctl(this->video_fd_, VIDIOC_QUERYCAP, &cap) < 0) {
    ESP_LOGE(TAG, "Failed to query capabilities: %s", strerror(errno));
    close(this->video_fd_);
    this->video_fd_ = -1;
    return false;
  }
  
  ESP_LOGI(TAG, "Video device opened:");
  ESP_LOGI(TAG, "  Driver: %s", cap.driver);
  ESP_LOGI(TAG, "  Card: %s", cap.card);
  ESP_LOGI(TAG, "  Bus: %s", cap.bus_info);
  ESP_LOGI(TAG, "  Version: %u.%u.%u", 
           (cap.version >> 16) & 0xFF,
           (cap.version >> 8) & 0xFF,
           cap.version & 0xFF);
  
  // Vérifier que c'est un device de capture
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    ESP_LOGE(TAG, "Device does not support video capture");
    close(this->video_fd_);
    this->video_fd_ = -1;
    return false;
  }
  
  // Vérifier le streaming
  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    ESP_LOGE(TAG, "Device does not support streaming");
    close(this->video_fd_);
    this->video_fd_ = -1;
    return false;
  }
  
  return true;
}

bool MipiDsiCamVideo::configure_video_device_() {
  // Définir le format de pixel
  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = this->width_;
  fmt.fmt.pix.height = this->height_;
  fmt.fmt.pix.pixelformat = this->get_v4l2_pixel_format_(this->pixel_format_);
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  
  if (ioctl(this->video_fd_, VIDIOC_S_FMT, &fmt) < 0) {
    ESP_LOGE(TAG, "Failed to set format: %s", strerror(errno));
    return false;
  }
  
  // Vérifier le format appliqué
  if (fmt.fmt.pix.width != this->width_ || fmt.fmt.pix.height != this->height_) {
    ESP_LOGW(TAG, "Resolution changed to %ux%u", fmt.fmt.pix.width, fmt.fmt.pix.height);
    this->width_ = fmt.fmt.pix.width;
    this->height_ = fmt.fmt.pix.height;
  }
  
  ESP_LOGI(TAG, "Video format configured:");
  ESP_LOGI(TAG, "  Resolution: %ux%u", fmt.fmt.pix.width, fmt.fmt.pix.height);
  ESP_LOGI(TAG, "  Pixel format: 0x%08X", fmt.fmt.pix.pixelformat);
  ESP_LOGI(TAG, "  Bytes per line: %u", fmt.fmt.pix.bytesperline);
  ESP_LOGI(TAG, "  Image size: %u bytes", fmt.fmt.pix.sizeimage);
  
  // Définir le framerate
  struct v4l2_streamparm parm = {};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = this->framerate_;
  
  if (ioctl(this->video_fd_, VIDIOC_S_PARM, &parm) < 0) {
    ESP_LOGW(TAG, "Failed to set framerate: %s", strerror(errno));
  } else {
    ESP_LOGI(TAG, "Framerate set to %u fps", this->framerate_);
  }
  
  // Configuration JPEG si nécessaire
  if (this->pixel_format_ == PixelFormat::JPEG) {
    this->set_v4l2_control_(this->video_fd_, V4L2_CID_JPEG_COMPRESSION_QUALITY, this->jpeg_quality_);
    ESP_LOGI(TAG, "JPEG quality set to %u", this->jpeg_quality_);
  }
  
  return true;
}

bool MipiDsiCamVideo::setup_buffers_() {
  // Demander des buffers
  struct v4l2_requestbuffers req = {};
  req.count = 4;  // 4 buffers pour le streaming
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  
  if (ioctl(this->video_fd_, VIDIOC_REQBUFS, &req) < 0) {
    ESP_LOGE(TAG, "Failed to request buffers: %s", strerror(errno));
    return false;
  }
  
  if (req.count < 2) {
    ESP_LOGE(TAG, "Insufficient buffer memory");
    return false;
  }
  
  ESP_LOGI(TAG, "Allocated %u buffers", req.count);
  
  // Mapper les buffers en mémoire
  this->buffers_.resize(req.count);
  
  for (uint32_t i = 0; i < req.count; i++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    
    if (ioctl(this->video_fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      ESP_LOGE(TAG, "Failed to query buffer %u: %s", i, strerror(errno));
      return false;
    }
    
    this->buffers_[i].length = buf.length;
    this->buffers_[i].index = i;
    this->buffers_[i].start = mmap(NULL, buf.length,
                                   PROT_READ | PROT_WRITE,
                                   MAP_SHARED,
                                   this->video_fd_, buf.m.offset);
    
    if (this->buffers_[i].start == MAP_FAILED) {
      ESP_LOGE(TAG, "Failed to mmap buffer %u: %s", i, strerror(errno));
      return false;
    }
    
    ESP_LOGD(TAG, "Buffer %u: %zu bytes at %p", i, buf.length, this->buffers_[i].start);
  }
  
  return true;
}

bool MipiDsiCamVideo::setup_h264_() {
  ESP_LOGI(TAG, "Setting up H.264 encoder...");
  
  // Ouvrir /dev/video5 (H.264 encoder)
  this->h264_fd_ = open("/dev/video5", O_RDWR | O_NONBLOCK);
  if (this->h264_fd_ < 0) {
    ESP_LOGE(TAG, "Cannot open /dev/video5: %s", strerror(errno));
    return false;
  }
  
  // Configurer le format H.264
  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  fmt.fmt.pix.width = this->width_;
  fmt.fmt.pix.height = this->height_;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
  
  if (ioctl(this->h264_fd_, VIDIOC_S_FMT, &fmt) < 0) {
    ESP_LOGE(TAG, "Failed to set H.264 format: %s", strerror(errno));
    close(this->h264_fd_);
    this->h264_fd_ = -1;
    return false;
  }
  
  // Configurer les paramètres H.264
  this->set_v4l2_control_(this->h264_fd_, V4L2_CID_MPEG_VIDEO_BITRATE, this->h264_bitrate_);
  this->set_v4l2_control_(this->h264_fd_, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD, this->h264_gop_);
  this->set_v4l2_control_(this->h264_fd_, V4L2_CID_MPEG_VIDEO_H264_MIN_QP, this->h264_qp_min_);
  this->set_v4l2_control_(this->h264_fd_, V4L2_CID_MPEG_VIDEO_H264_MAX_QP, this->h264_qp_max_);
  
  ESP_LOGI(TAG, "H.264 configured:");
  ESP_LOGI(TAG, "  Bitrate: %u bps", this->h264_bitrate_);
  ESP_LOGI(TAG, "  GOP: %u", this->h264_gop_);
  ESP_LOGI(TAG, "  QP range: %u-%u", this->h264_qp_min_, this->h264_qp_max_);
  
  // Setup buffers H.264
  struct v4l2_requestbuffers req = {};
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  req.memory = V4L2_MEMORY_MMAP;
  
  if (ioctl(this->h264_fd_, VIDIOC_REQBUFS, &req) < 0) {
    ESP_LOGE(TAG, "Failed to request H.264 buffers: %s", strerror(errno));
    close(this->h264_fd_);
    this->h264_fd_ = -1;
    return false;
  }
  
  // Mapper les buffers H.264
  this->h264_buffers_.resize(req.count);
  
  for (uint32_t i = 0; i < req.count; i++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    
    if (ioctl(this->h264_fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      ESP_LOGE(TAG, "Failed to query H.264 buffer %u: %s", i, strerror(errno));
      return false;
    }
    
    this->h264_buffers_[i].length = buf.length;
    this->h264_buffers_[i].index = i;
    this->h264_buffers_[i].start = mmap(NULL, buf.length,
                                        PROT_READ | PROT_WRITE,
                                        MAP_SHARED,
                                        this->h264_fd_, buf.m.offset);
    
    if (this->h264_buffers_[i].start == MAP_FAILED) {
      ESP_LOGE(TAG, "Failed to mmap H.264 buffer %u: %s", i, strerror(errno));
      return false;
    }
  }
  
  ESP_LOGI(TAG, "H.264 encoder ready");
  return true;
}

bool MipiDsiCamVideo::setup_isp_() {
  // Ouvrir /dev/video1 (ISP)
  this->isp_fd_ = open("/dev/video1", O_RDWR);
  if (this->isp_fd_ < 0) {
    ESP_LOGD(TAG, "ISP device not available (optional)");
    return false;
  }
  
  ESP_LOGI(TAG, "ISP device opened");
  
  // Configuration ISP par défaut via V4L2 controls
  this->set_brightness(50);
  this->set_contrast(50);
  this->set_saturation(50);
  this->set_sharpen(3);
  this->set_denoise(2);
  
  return true;
}

bool MipiDsiCamVideo::start_streaming() {
  if (this->streaming_) {
    return true;
  }
  
  if (!this->initialized_) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }
  
  ESP_LOGI(TAG, "Starting streaming...");
  
  // Enqueue tous les buffers
  for (size_t i = 0; i < this->buffers_.size(); i++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    
    if (ioctl(this->video_fd_, VIDIOC_QBUF, &buf) < 0) {
      ESP_LOGE(TAG, "Failed to queue buffer %zu: %s", i, strerror(errno));
      return false;
    }
  }
  
  // Démarrer le streaming
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->video_fd_, VIDIOC_STREAMON, &type) < 0) {
    ESP_LOGE(TAG, "Failed to start streaming: %s", strerror(errno));
    return false;
  }
  
  // Démarrer H.264 si activé
  if (this->h264_enabled_ && this->h264_fd_ >= 0) {
    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    if (ioctl(this->h264_fd_, VIDIOC_STREAMON, &type) < 0) {
      ESP_LOGW(TAG, "Failed to start H.264 streaming: %s", strerror(errno));
    } else {
      ESP_LOGI(TAG, "H.264 streaming started");
    }
  }
  
  this->streaming_ = true;
  this->frame_count_ = 0;
  this->dropped_frames_ = 0;
  this->last_fps_calc_time_ = millis();
  this->frames_since_last_calc_ = 0;
  
  ESP_LOGI(TAG, "Streaming started successfully");
  return true;
}

bool MipiDsiCamVideo::stop_streaming() {
  if (!this->streaming_) {
    return true;
  }
  
  ESP_LOGI(TAG, "Stopping streaming...");
  
  // Arrêter H.264 si actif
  if (this->h264_enabled_ && this->h264_fd_ >= 0) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ioctl(this->h264_fd_, VIDIOC_STREAMOFF, &type);
  }
  
  // Arrêter le streaming principal
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->video_fd_, VIDIOC_STREAMOFF, &type) < 0) {
    ESP_LOGE(TAG, "Failed to stop streaming: %s", strerror(errno));
    return false;
  }
  
  this->streaming_ = false;
  ESP_LOGI(TAG, "Streaming stopped");
  
  return true;
}

bool MipiDsiCamVideo::capture_frame() {
  if (!this->streaming_) {
    return false;
  }
  
  // Dequeue un buffer
  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  
  if (ioctl(this->video_fd_, VIDIOC_DQBUF, &buf) < 0) {
    if (errno == EAGAIN) {
      return false;  // Pas de frame disponible
    }
    ESP_LOGE(TAG, "Failed to dequeue buffer: %s", strerror(errno));
    return false;
  }
  
  // Copier les données de la frame
  if (buf.index < this->buffers_.size()) {
    this->current_frame_data_ = static_cast<uint8_t*>(this->buffers_[buf.index].start);
    this->current_frame_size_ = buf.bytesused;
    
    this->frame_count_++;
    this->frames_since_last_calc_++;
    
    // Requeue le buffer
    if (ioctl(this->video_fd_, VIDIOC_QBUF, &buf) < 0) {
      ESP_LOGW(TAG, "Failed to requeue buffer: %s", strerror(errno));
    }
    
    // Écrire dans le fichier si en enregistrement
    if (this->recording_ && this->rec_fd_ >= 0) {
      this->write_frame_to_file_(this->current_frame_data_, this->current_frame_size_);
    }
    
    return true;
  }
  
  this->dropped_frames_++;
  return false;
}

bool MipiDsiCamVideo::enable_h264(bool enable) {
  if (enable == this->h264_enabled_) {
    return true;
  }
  
  if (enable) {
    return this->setup_h264_();
  } else {
    if (this->h264_fd_ >= 0) {
      close(this->h264_fd_);
      this->h264_fd_ = -1;
    }
    this->h264_enabled_ = false;
    return true;
  }
}

bool MipiDsiCamVideo::get_h264_frame(uint8_t **data, size_t *size) {
  if (!this->h264_enabled_ || this->h264_fd_ < 0) {
    return false;
  }
  
  // Dequeue un buffer H.264
  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  buf.memory = V4L2_MEMORY_MMAP;
  
  if (ioctl(this->h264_fd_, VIDIOC_DQBUF, &buf) < 0) {
    if (errno == EAGAIN) {
      return false;
    }
    return false;
  }
  
  if (buf.index < this->h264_buffers_.size()) {
    *data = static_cast<uint8_t*>(this->h264_buffers_[buf.index].start);
    *size = buf.bytesused;
    
    // Requeue
    ioctl(this->h264_fd_, VIDIOC_QBUF, &buf);
    return true;
  }
  
  return false;
}

bool MipiDsiCamVideo::start_recording(const std::string &filename) {
  if (this->recording_) {
    ESP_LOGW(TAG, "Already recording");
    return false;
  }
  
  // Ouvrir le fichier
  this->rec_fd_ = open(filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (this->rec_fd_ < 0) {
    ESP_LOGE(TAG, "Failed to open file %s: %s", filename.c_str(), strerror(errno));
    return false;
  }
  
  this->recording_ = true;
  ESP_LOGI(TAG, "Recording started to: %s", filename.c_str());
  
  return true;
}

bool MipiDsiCamVideo::stop_recording() {
  if (!this->recording_) {
    return true;
  }
  
  if (this->rec_fd_ >= 0) {
    close(this->rec_fd_);
    this->rec_fd_ = -1;
  }
  
  this->recording_ = false;
  ESP_LOGI(TAG, "Recording stopped");
  
  return true;
}

bool MipiDsiCamVideo::write_frame_to_file_(const uint8_t *data, size_t size) {
  if (this->rec_fd_ < 0 || !data || size == 0) {
    return false;
  }
  
  ssize_t written = write(this->rec_fd_, data, size);
  if (written != static_cast<ssize_t>(size)) {
    ESP_LOGE(TAG, "Failed to write frame to file: %s", strerror(errno));
    return false;
  }
  
  return true;
}

// === Contrôles Caméra (V4L2) ===

void MipiDsiCamVideo::set_brightness(int value) {
  this->set_v4l2_control_(this->video_fd_, V4L2_CID_BRIGHTNESS, value);
}

void MipiDsiCamVideo::set_contrast(int value) {
  this->set_v4l2_control_(this->video_fd_, V4L2_CID_CONTRAST, value);
}

void MipiDsiCamVideo::set_saturation(int value) {
  this->set_v4l2_control_(this->video_fd_, V4L2_CID_SATURATION, value);
}

void MipiDsiCamVideo::set_hue(int value) {
  this->set_v4l2_control_(this->video_fd_, V4L2_CID_HUE, value);
}

void MipiDsiCamVideo::set_exposure(int value) {
  this->set_v4l2_control_(this->video_fd_, V4L2_CID_EXPOSURE_ABSOLUTE, value);
}

void MipiDsiCamVideo::set_gain(int value) {
  this->set_v4l2_control_(this->video_fd_, V4L2_CID_GAIN, value);
}

void MipiDsiCamVideo::set_white_balance(bool auto_wb) {
  this->set_v4l2_control_(this->video_fd_, V4L2_CID_AUTO_WHITE_BALANCE, auto_wb ? 1 : 0);
}

void MipiDsiCamVideo::set_white_balance_temp(int temp) {
  this->set_v4l2_control_(this->video_fd_, V4L2_CID_WHITE_BALANCE_TEMPERATURE, temp);
}

void MipiDsiCamVideo::set_horizontal_flip(bool enable) {
  this->set_v4l2_control_(this->video_fd_, V4L2_CID_HFLIP, enable ? 1 : 0);
}

void MipiDsiCamVideo::set_vertical_flip(bool enable) {
  this->set_v4l2_control_(this->video_fd_, V4L2_CID_VFLIP, enable ? 1 : 0);
}

void MipiDsiCamVideo::set_sharpen(uint8_t level) {
  if (this->isp_fd_ >= 0) {
    this->set_v4l2_control_(this->isp_fd_, V4L2_CID_SHARPNESS, level);
  }
}

void MipiDsiCamVideo::set_denoise(uint8_t level) {
  // Custom control pour ISP denoise
  // À adapter selon l'implémentation esp-video
  if (this->isp_fd_ >= 0) {
    // V4L2_CID_USER_ESP_ISP_BF pour le Bayer Filter (denoise)
    // Nécessite une structure spécifique
  }
}

// === Helpers V4L2 ===

bool MipiDsiCamVideo::set_v4l2_control_(int fd, uint32_t id, int32_t value) {
  if (fd < 0) {
    return false;
  }
  
  struct v4l2_control ctrl = {};
  ctrl.id = id;
  ctrl.value = value;
  
  if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0) {
    ESP_LOGD(TAG, "Failed to set control 0x%08X: %s", id, strerror(errno));
    return false;
  }
  
  return true;
}

bool MipiDsiCamVideo::get_v4l2_control_(int fd, uint32_t id, int32_t *value) {
  if (fd < 0 || !value) {
    return false;
  }
  
  struct v4l2_control ctrl = {};
  ctrl.id = id;
  
  if (ioctl(fd, VIDIOC_G_CTRL, &ctrl) < 0) {
    ESP_LOGD(TAG, "Failed to get control 0x%08X: %s", id, strerror(errno));
    return false;
  }
  
  *value = ctrl.value;
  return true;
}

uint32_t MipiDsiCamVideo::get_v4l2_pixel_format_(PixelFormat format) {
  switch (format) {
    case PixelFormat::RGB565:
      return V4L2_PIX_FMT_RGB565;
    case PixelFormat::RGB888:
      return V4L2_PIX_FMT_RGB24;
    case PixelFormat::YUV422:
      return V4L2_PIX_FMT_YUYV;
    case PixelFormat::JPEG:
      return V4L2_PIX_FMT_JPEG;
    case PixelFormat::H264:
      return V4L2_PIX_FMT_H264;
    case PixelFormat::RAW8:
      return V4L2_PIX_FMT_SRGGB8;
    default:
      return V4L2_PIX_FMT_RGB565;
  }
}

// === Résolutions ===

void MipiDsiCamVideo::set_resolution(uint16_t width, uint16_t height) {
  this->width_ = width;
  this->height_ = height;
}

void MipiDsiCamVideo::set_resolution_preset(const std::string &preset) {
  for (const auto &res : RESOLUTIONS) {
    if (preset == res.name) {
      this->width_ = res.width;
      this->height_ = res.height;
      return;
    }
  }
  ESP_LOGW(TAG, "Unknown resolution preset: %s", preset.c_str());
}

// === FPS Calculation ===

void MipiDsiCamVideo::update_fps_() {
  uint32_t now = millis();
  uint32_t elapsed = now - this->last_fps_calc_time_;
  
  if (elapsed >= 1000) {  // Calculer toutes les secondes
    this->actual_fps_ = (this->frames_since_last_calc_ * 1000.0f) / elapsed;
    this->frames_since_last_calc_ = 0;
    this->last_fps_calc_time_ = now;
  }
}

// === Cleanup ===

void MipiDsiCamVideo::cleanup_() {
  // Arrêter le streaming
  this->stop_streaming();
  this->stop_recording();
  
  // Unmapper les buffers
  for (auto &buf : this->buffers_) {
    if (buf.start && buf.start != MAP_FAILED) {
      munmap(buf.start, buf.length);
    }
  }
  this->buffers_.clear();
  
  for (auto &buf : this->h264_buffers_) {
    if (buf.start && buf.start != MAP_FAILED) {
      munmap(buf.start, buf.length);
    }
  }
  this->h264_buffers_.clear();
  
  // Fermer les file descriptors
  if (this->video_fd_ >= 0) {
    close(this->video_fd_);
    this->video_fd_ = -1;
  }
  
  if (this->isp_fd_ >= 0) {
    close(this->isp_fd_);
    this->isp_fd_ = -1;
  }
  
  if (this->h264_fd_ >= 0) {
    close(this->h264_fd_);
    this->h264_fd_ = -1;
  }
  
  if (this->jpeg_fd_ >= 0) {
    close(this->jpeg_fd_);
    this->jpeg_fd_ = -1;
  }
  
  if (this->rec_fd_ >= 0) {
    close(this->rec_fd_);
    this->rec_fd_ = -1;
  }
}

// === Loop ===

void MipiDsiCamVideo::loop() {
  if (this->streaming_) {
    this->update_fps_();
  }
}

// === Dump Config ===

void MipiDsiCamVideo::dump_config() {
  ESP_LOGCONFIG(TAG, "MIPI Camera (esp-video):");
  ESP_LOGCONFIG(TAG, "  Sensor: %s", this->sensor_name_.c_str());
  ESP_LOGCONFIG(TAG, "  Resolution: %ux%u", this->width_, this->height_);
  
  const char *format_str = "Unknown";
  switch (this->pixel_format_) {
    case PixelFormat::RGB565: format_str = "RGB565"; break;
    case PixelFormat::RGB888: format_str = "RGB888"; break;
    case PixelFormat::YUV422: format_str = "YUV422"; break;
    case PixelFormat::JPEG: format_str = "JPEG"; break;
    case PixelFormat::H264: format_str = "H.264"; break;
    case PixelFormat::RAW8: format_str = "RAW8"; break;
  }
  ESP_LOGCONFIG(TAG, "  Format: %s", format_str);
  ESP_LOGCONFIG(TAG, "  Framerate: %u fps", this->framerate_);
  
  if (this->h264_enabled_) {
    ESP_LOGCONFIG(TAG, "  H.264 Enabled:");
    ESP_LOGCONFIG(TAG, "    Bitrate: %u bps", this->h264_bitrate_);
    ESP_LOGCONFIG(TAG, "    GOP: %u", this->h264_gop_);
    ESP_LOGCONFIG(TAG, "    QP: %u-%u", this->h264_qp_min_, this->h264_qp_max_);
  }
  
  ESP_LOGCONFIG(TAG, "  Streaming: %s", this->streaming_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Recording: %s", this->recording_ ? "YES" : "NO");
  
  if (this->streaming_) {
    ESP_LOGCONFIG(TAG, "  Stats:");
    ESP_LOGCONFIG(TAG, "    Frames: %u", this->frame_count_);
    ESP_LOGCONFIG(TAG, "    FPS: %.1f", this->actual_fps_);
    ESP_LOGCONFIG(TAG, "    Dropped: %u", this->dropped_frames_);
  }
}

}  // namespace mipi_dsi_cam_video
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
