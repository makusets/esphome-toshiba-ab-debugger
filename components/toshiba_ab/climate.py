import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
AUTO_LOAD = []

toshiba_ab_ns = cg.esphome_ns.namespace("toshiba_ab")

CONF_MASTER = "master"

ToshibaAbLogger = toshiba_ab_ns.class_("ToshibaAbLogger", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ToshibaAbLogger),
        cv.Optional(CONF_MASTER, default=0x00): cv.uint8_t,
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)

def _validate_uart(config):
    uart.final_validate_device_schema(
        "toshiba_ab", baud_rate=2400, require_rx=True, require_tx=False
    )(config)

FINAL_VALIDATE_SCHEMA = _validate_uart

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_master_address(config[CONF_MASTER]))