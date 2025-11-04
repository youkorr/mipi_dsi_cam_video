"""ESPHome component for LVGL Camera Display."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import lvgl
from esphome.const import CONF_ID

DEPENDENCIES = ["lvgl", "mipi_dsi_cam_video"]
AUTO_LOAD = []
CODEOWNERS = ["@youkorr"]

lvgl_camera_display_ns = cg.esphome_ns.namespace("lvgl_camera_display")
LVGLCameraDisplay = lvgl_camera_display_ns.class_(
    "LVGLCameraDisplay", cg.Component
)

# Display modes
DisplayMode = lvgl_camera_display_ns.enum("DisplayMode")
DISPLAY_MODES = {
    "FIT": DisplayMode.FIT,
    "FILL": DisplayMode.FILL,
    "CENTER": DisplayMode.CENTER,
    "STRETCH": DisplayMode.STRETCH,
}

# Configuration keys
CONF_CAMERA_ID = "camera_id"
CONF_UPDATE_INTERVAL = "update_interval"
CONF_DISPLAY_MODE = "display_mode"
CONF_ROTATION = "rotation"
CONF_AUTO_START = "auto_start"
CONF_PARENT_OBJ = "parent_obj"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(LVGLCameraDisplay),
    cv.Required(CONF_CAMERA_ID): cv.use_id("MipiDsiCamVideo"),
    cv.Optional(CONF_UPDATE_INTERVAL, default="33ms"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_DISPLAY_MODE, default="FIT"): cv.enum(DISPLAY_MODES, upper=True),
    cv.Optional(CONF_ROTATION, default=0): cv.int_range(min=0, max=360),
    cv.Optional(CONF_AUTO_START, default=True): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    """Générer le code C++ pour le composant."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    # Obtenir la référence à la caméra
    camera = await cg.get_variable(config[CONF_CAMERA_ID])
    cg.add(var.set_camera(camera))
    
    # Update interval
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    
    # Display mode
    cg.add(var.set_display_mode(config[CONF_DISPLAY_MODE]))
    
    # Rotation
    if config[CONF_ROTATION] > 0:
        cg.add(var.set_rotation(config[CONF_ROTATION]))
    
    # Auto start
    cg.add(var.set_auto_start(config[CONF_AUTO_START]))
    
    # Ajouter les build flags
    cg.add_build_flag("-DUSE_ESP32_VARIANT_ESP32P4")
    
    # Inclure LVGL
    cg.add_library("lvgl", None)
