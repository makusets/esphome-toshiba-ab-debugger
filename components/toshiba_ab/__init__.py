import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.const import CONF_ID
from esphome.core import CORE

DEPENDENCIES = ["esp8266"]
MULTI_CONF = False

toshiba_ab_ns = cg.esphome_ns.namespace("toshiba_ab")
ToshibaAbLogger = toshiba_ab_ns.class_("ToshibaAbLogger", cg.Component)


def validate_config(config):
    if not CORE.is_esp8266:
        raise cv.Invalid("This receiver owns ESP8266 hardware UART0")
    bits = 1 + config["data_bits"] + (config["parity"] != "NONE") + config["stop_bits"]
    requested_us = config["frame_gap"].total_microseconds
    symbols = (requested_us * config["baud_rate"] + bits * 1000000 - 1) // (bits * 1000000)
    if not 1 <= symbols <= 127:
        raise cv.Invalid("frame_gap must fit 1..127 UART character times at the selected baud rate")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(ToshibaAbLogger),
        cv.Optional("master", default=0x00): cv.uint8_t,
        cv.Optional("debug_mode", default="raw"): cv.one_of("raw", "normal"),
        cv.Optional("rx_pin", default=13): cv.one_of(3, 13, int=True),
        cv.Optional("baud_rate", default=2400): cv.int_range(min=300, max=115200),
        cv.Optional("data_bits", default=8): cv.int_range(min=5, max=8),
        cv.Optional("parity", default="EVEN"): cv.one_of("NONE", "EVEN", "ODD", upper=True),
        cv.Optional("stop_bits", default=1): cv.one_of(1, 2, int=True),
        cv.Optional("frame_gap", default="20ms"): cv.positive_time_period_microseconds,
    }).extend(cv.COMPONENT_SCHEMA),
    validate_config,
)


def validate_ownership(config):
    full = fv.full_config.get()
    if full.get("logger", {}).get("baud_rate", 115200) != 0:
        raise cv.Invalid("The receiver owns UART0; set logger.baud_rate: 0")
    if full.get("uart"):
        raise cv.Invalid("Remove uart:; configure the receiver directly under toshiba_ab:")
    return config


FINAL_VALIDATE_SCHEMA = validate_ownership


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_master_address(config["master"]))
    cg.add(var.set_debug_mode(config["debug_mode"]))
    cg.add(var.set_frame_gap(config["frame_gap"].total_microseconds))
    cg.add(var.set_uart_config(config["rx_pin"], config["baud_rate"], config["data_bits"],
                               {"NONE": 0, "EVEN": 1, "ODD": 2}[config["parity"]], config["stop_bits"]))
