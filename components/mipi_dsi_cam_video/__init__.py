"""ESPHome component for MIPI CSI Camera using esp-video framework."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_NAME,
)

DEPENDENCIES = ["i2c"]
AUTO_LOAD = []
CODEOWNERS = ["@your_github"]

mipi_dsi_cam_video_ns = cg.esphome_ns.namespace("mipi_dsi_cam_video")
MipiDsiCamVideo = mipi_dsi_cam_video_ns.class_("MipiDsiCamVideo", cg.Component, i2c.I2CDevice)

# Pixel formats
PixelFormat = mipi_dsi_cam_video_ns.enum("PixelFormat")
PIXEL_FORMATS = {
    "RGB565": PixelFormat.RGB565,
    "RGB888": PixelFormat.RGB888,
    "YUV422": PixelFormat.YUV422,
    "JPEG": PixelFormat.JPEG,
    "H264": PixelFormat.H264,
    "RAW8": PixelFormat.RAW8,
}

# Configuration keys
CONF_SENSOR_NAME = "sensor"
CONF_RESOLUTION = "resolution"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_FRAMERATE = "framerate"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_EXTERNAL_CLOCK = "external_clock"
CONF_FREQUENCY = "frequency"

# H.264 configuration
CONF_H264 = "h264"
CONF_H264_ENABLED = "enabled"
CONF_H264_BITRATE = "bitrate"
CONF_H264_GOP = "gop"
CONF_H264_QP_MIN = "qp_min"
CONF_H264_QP_MAX = "qp_max"

# Camera controls
CONF_CONTROLS = "controls"
CONF_BRIGHTNESS = "brightness"
CONF_CONTRAST = "contrast"
CONF_SATURATION = "saturation"
CONF_HUE = "hue"
CONF_EXPOSURE = "exposure"
CONF_GAIN = "gain"
CONF_WHITE_BALANCE = "white_balance"
CONF_AUTO_WHITE_BALANCE = "auto"
CONF_WHITE_BALANCE_TEMP = "temperature"
CONF_HORIZONTAL_FLIP = "horizontal_flip"
CONF_VERTICAL_FLIP = "vertical_flip"
CONF_SHARPEN = "sharpen"
CONF_DENOISE = "denoise"

# Résolutions prédéfinies
RESOLUTIONS = {
    "VGA": (640, 480),
    "SVGA": (800, 600),
    "720P": (1280, 720),
    "UXGA": (1600, 1200),
    "1080P": (1920, 1080),
}

def validate_resolution(value):
    """Valider et convertir la résolution."""
    if isinstance(value, str):
        value = value.upper()
        if value in RESOLUTIONS:
            return value
        raise cv.Invalid(f"Unknown resolution preset: {value}")
    if isinstance(value, dict):
        width = cv.int_(value.get("width"))
        height = cv.int_(value.get("height"))
        return {"width": width, "height": height}
    raise cv.Invalid("Resolution must be a preset name or dict with width/height")

# Schema de configuration
H264_SCHEMA = cv.Schema({
    cv.Optional(CONF_H264_ENABLED, default=False): cv.boolean,
    cv.Optional(CONF_H264_BITRATE, default=2000000): cv.int_range(min=100000, max=20000000),
    cv.Optional(CONF_H264_GOP, default=30): cv.int_range(min=1, max=300),
    cv.Optional(CONF_H264_QP_MIN, default=10): cv.int_range(min=0, max=51),
    cv.Optional(CONF_H264_QP_MAX, default=40): cv.int_range(min=0, max=51),
})

EXTERNAL_CLOCK_SCHEMA = cv.Schema({
    cv.Required(pins.GPIO_FULL_OUTPUT_PIN_SCHEMA): pins.GPIO_FULL_OUTPUT_PIN_SCHEMA,
    cv.Optional(CONF_FREQUENCY, default="24MHz"): cv.All(
        cv.frequency, cv.int_range(min=1000000, max=40000000)
    ),
})

WHITE_BALANCE_SCHEMA = cv.Schema({
    cv.Optional(CONF_AUTO_WHITE_BALANCE, default=True): cv.boolean,
    cv.Optional(CONF_WHITE_BALANCE_TEMP, default=5000): cv.int_range(min=2000, max=10000),
})

CONTROLS_SCHEMA = cv.Schema({
    cv.Optional(CONF_BRIGHTNESS, default=50): cv.int_range(min=0, max=100),
    cv.Optional(CONF_CONTRAST, default=50): cv.int_range(min=0, max=100),
    cv.Optional(CONF_SATURATION, default=50): cv.int_range(min=0, max=100),
    cv.Optional(CONF_HUE, default=0): cv.int_range(min=-180, max=180),
    cv.Optional(CONF_EXPOSURE): cv.int_range(min=0, max=10000),
    cv.Optional(CONF_GAIN): cv.int_range(min=0, max=100),
    cv.Optional(CONF_WHITE_BALANCE): WHITE_BALANCE_SCHEMA,
    cv.Optional(CONF_HORIZONTAL_FLIP, default=False): cv.boolean,
    cv.Optional(CONF_VERTICAL_FLIP, default=False): cv.boolean,
    cv.Optional(CONF_SHARPEN, default=3): cv.int_range(min=0, max=10),
    cv.Optional(CONF_DENOISE, default=2): cv.int_range(min=0, max=10),
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MipiDsiCamVideo),
    cv.Optional(CONF_NAME, default="MIPI Camera"): cv.string,
    cv.Optional(CONF_SENSOR_NAME, default="sc202cs"): cv.string,
    cv.Optional(CONF_RESOLUTION, default="720P"): validate_resolution,
    cv.Optional(CONF_PIXEL_FORMAT, default="RGB565"): cv.enum(PIXEL_FORMATS, upper=True),
    cv.Optional(CONF_FRAMERATE, default=30): cv.int_range(min=1, max=60),
    cv.Optional(CONF_JPEG_QUALITY, default=80): cv.int_range(min=1, max=100),
    cv.Optional(CONF_EXTERNAL_CLOCK): EXTERNAL_CLOCK_SCHEMA,
    cv.Optional(CONF_H264): H264_SCHEMA,
    cv.Optional(CONF_CONTROLS): CONTROLS_SCHEMA,
}).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x36))

async def to_code(config):
    """Générer le code C++ pour le composant."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    # Configuration de base
    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_sensor_name(config[CONF_SENSOR_NAME]))
    
    # External clock (horloge MCLK pour le capteur)
    if CONF_EXTERNAL_CLOCK in config:
        clock_conf = config[CONF_EXTERNAL_CLOCK]
        pin = await cg.gpio_pin_expression(clock_conf[pins.GPIO_FULL_OUTPUT_PIN_SCHEMA])
        cg.add(var.set_external_clock_pin(pin))
        cg.add(var.set_external_clock_frequency(clock_conf[CONF_FREQUENCY]))
    
    # Résolution
    resolution = config[CONF_RESOLUTION]
    if isinstance(resolution, str):
        cg.add(var.set_resolution_preset(resolution))
    else:
        cg.add(var.set_resolution(resolution["width"], resolution["height"]))
    
    # Format de pixel
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    
    # Framerate
    cg.add(var.set_framerate(config[CONF_FRAMERATE]))
    
    # JPEG quality
    if config[CONF_PIXEL_FORMAT] == PIXEL_FORMATS["JPEG"]:
        cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    
    # H.264 configuration
    if CONF_H264 in config:
        h264_conf = config[CONF_H264]
        if h264_conf[CONF_H264_ENABLED]:
            cg.add(var.enable_h264(True))
            cg.add(var.set_h264_bitrate(h264_conf[CONF_H264_BITRATE]))
            cg.add(var.set_h264_gop(h264_conf[CONF_H264_GOP]))
            cg.add(var.set_h264_qp_min(h264_conf[CONF_H264_QP_MIN]))
            cg.add(var.set_h264_qp_max(h264_conf[CONF_H264_QP_MAX]))
    
    # Contrôles caméra
    if CONF_CONTROLS in config:
        controls = config[CONF_CONTROLS]
        
        # Attendre la fin du setup pour appliquer les contrôles
        cg.add(var.set_brightness(controls[CONF_BRIGHTNESS]))
        cg.add(var.set_contrast(controls[CONF_CONTRAST]))
        cg.add(var.set_saturation(controls[CONF_SATURATION]))
        cg.add(var.set_hue(controls[CONF_HUE]))
        
        if CONF_EXPOSURE in controls:
            cg.add(var.set_exposure(controls[CONF_EXPOSURE]))
        
        if CONF_GAIN in controls:
            cg.add(var.set_gain(controls[CONF_GAIN]))
        
        if CONF_WHITE_BALANCE in controls:
            wb = controls[CONF_WHITE_BALANCE]
            cg.add(var.set_white_balance(wb[CONF_AUTO_WHITE_BALANCE]))
            if not wb[CONF_AUTO_WHITE_BALANCE]:
                cg.add(var.set_white_balance_temp(wb[CONF_WHITE_BALANCE_TEMP]))
        
        cg.add(var.set_horizontal_flip(controls[CONF_HORIZONTAL_FLIP]))
        cg.add(var.set_vertical_flip(controls[CONF_VERTICAL_FLIP]))
        cg.add(var.set_sharpen(controls[CONF_SHARPEN]))
        cg.add(var.set_denoise(controls[CONF_DENOISE]))
    
    # Ajouter les définitions de build
    cg.add_build_flag("-DUSE_ESP32_VARIANT_ESP32P4")
    
    # Ajouter les dépendances de bibliothèques
    cg.add_library("esp_video", None)
    cg.add_library("esp_cam_sensor", None)
    cg.add_library("esp_h264", None)
    cg.add_library("esp_ipa", None)
    cg.add_library("esp_sccb_intf", None)
