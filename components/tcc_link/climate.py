# standard expressions defined by esphome to validate configuration and generate the code
import esphome.codegen as cg   
import esphome.config_validation as cv

#import automation, seems standard practice:
from esphome import automation

#import the components that we will be using:
#these have to reflect esphome components structure and syntax
#they need to be loaded below either under DEPENDENCIES or AUTO_LOAD
#later, the libraries need to be loaded in the .h file
from esphome.components import climate, uart, i2c, bme280_i2c, binary_sensor, sensor, switch, text_sensor, template
# from ..bme280_base import CONFIG_SCHEMA_BASE, to_code_base  #added for bme280 sensor, gives error

#import all the conf constants that we will be using
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_HARDWARE_UART,
    CONF_BAUD_RATE,
    CONF_UPDATE_INTERVAL,
    CONF_MODE,
    CONF_FAN_MODE,
    CONF_SWING_MODE,
    CONF_TRIGGER_ID,
    DEVICE_CLASS_CONNECTIVITY,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    UNIT_CELSIUS,
    ###added for bme280 sensor
    CONF_HUMIDITY,
    CONF_ID,
    CONF_IIR_FILTER,
    CONF_OVERSAMPLING,
    CONF_PRESSURE,
    CONF_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_PRESSURE,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_HECTOPASCAL,
    UNIT_PERCENT,
    CONF_ADDRESS,
    CONF_FREQUENCY,
    CONF_I2C_ID,
    CONF_INPUT,
    CONF_OUTPUT,
    CONF_SCAN,
    CONF_SCL,
    CONF_SDA,
    CONF_TIMEOUT,
    PLATFORM_ESP32,
    PLATFORM_ESP8266,
    PLATFORM_RP2040,
    ###end of bme280 sensor section
)




#Checks that uart is correctly defined in the YAML as a requirement:
DEPENDENCIES = ["uart"]
DEPENDENCIES = ["i2c"]  #added for bme280 sensor


AUTO_LOAD = ["climate", "binary_sensor", "sensor", "switch", "i2c", "bme280_base", "bme280_i2c", "text_sensor", "template"]
CODEOWNERS = ["@muxa", "@theeuwke"]

tcc_link_ns = cg.esphome_ns.namespace("tcc_link")
## bme280_ns = cg.esphome_ns.namespace("bme280_i2c")   #bme280 namespace added for temp sensor


###added for bme280 sensor
BME280Oversampling = bme280_ns.enum("BME280Oversampling") #define the BME280 oversampling enum
OVERSAMPLING_OPTIONS = {
    "NONE": BME280Oversampling.BME280_OVERSAMPLING_NONE,
    "1X": BME280Oversampling.BME280_OVERSAMPLING_1X,
    "2X": BME280Oversampling.BME280_OVERSAMPLING_2X,
    "4X": BME280Oversampling.BME280_OVERSAMPLING_4X,
    "8X": BME280Oversampling.BME280_OVERSAMPLING_8X,
    "16X": BME280Oversampling.BME280_OVERSAMPLING_16X,
}
BME280IIRFilter = bme280_ns.enum("BME280IIRFilter")
IIR_FILTER_OPTIONS = {
    "OFF": BME280IIRFilter.BME280_IIR_FILTER_OFF,
    "2X": BME280IIRFilter.BME280_IIR_FILTER_2X,
    "4X": BME280IIRFilter.BME280_IIR_FILTER_4X,
    "8X": BME280IIRFilter.BME280_IIR_FILTER_8X,
    "16X": BME280IIRFilter.BME280_IIR_FILTER_16X,
}
###end of bme280 sensor section


CONF_CONNECTED = "connected"
CONF_VENT = "vent"
CONF_FAILED_CRCS = "failed_crcs"

CONF_ON_DATA_RECEIVED = "on_data_received"


TccLinkClimate =  tcc_link_ns.class_(
    "TccLinkClimate", climate.Climate, uart.UARTDevice, cg.Component
)

TccLinkVentSwitch =  tcc_link_ns.class_(
    "TccLinkVentSwitch", switch.Switch, cg.Component
)

TccLinkOnDataReceivedTrigger = tcc_link_ns.class_(
    "TccLinkOnDataReceivedTrigger", automation.Trigger.template()
)

BME280I2CComponent = tcc_link_ns.class_(
    "BME280I2CComponent", cg.PollingComponent, i2c.I2CDevice
) #bme280 class added for temp sensor


CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(TccLinkClimate),  # We set the class for the component, TccLinkClimate, in this case
        cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class = DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_VENT): cv.maybe_simple_value(
            switch.SWITCH_SCHEMA.extend(
                cv.Schema(
                    {
                        cv.GenerateID(): cv.declare_id(TccLinkVentSwitch),
                    }
                )
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_FAILED_CRCS): sensor.sensor_schema(
            accuracy_decimals=0,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ON_DATA_RECEIVED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    TccLinkOnDataReceivedTrigger
                ),
            }
        ),
        ### section for the bme280 sensor
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(
            {
                cv.Optional(CONF_OVERSAMPLING, default="16X"): cv.enum(
                    OVERSAMPLING_OPTIONS, upper=True
                ),
            }
        ),
        cv.Optional(CONF_PRESSURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_HECTOPASCAL,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_PRESSURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(
            {
                cv.Optional(CONF_OVERSAMPLING, default="16X"): cv.enum(
                    OVERSAMPLING_OPTIONS, upper=True
                ),
            }
        ),
        cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(
            {
                cv.Optional(CONF_OVERSAMPLING, default="16X"): cv.enum(
                    OVERSAMPLING_OPTIONS, upper=True
                ),
            }
        ),
        cv.Optional(CONF_IIR_FILTER, default="OFF"): cv.enum(
            IIR_FILTER_OPTIONS, upper=True
        ),
        ### end of bme280 sensor section

    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA).extend(cv.polling_component_schema("60s")).extend(
    i2c.i2c_device_schema(default_address=0x77)
).extend({cv.GenerateID(): cv.declare_id(BME280I2CComponent)})
#added bme280 device schema to the config schema with extends at the end, polling, i2c device and generateID


def validate_uart(config):
    uart.final_validate_device_schema(
        "tcc_link", baud_rate=2400, require_rx=True, require_tx=False
    )(config)


FINAL_VALIDATE_SCHEMA = validate_uart

#next this setups and transforms the conf into C++ code to run as setup and loop

async def to_code(config): #standard syntax
    var = cg.new_Pvariable(config[CONF_ID])  #standard syntax
    await cg.register_component(var, config)   #standard syntax
    await climate.register_climate(var, config)  #wait for the climate component to create
    await uart.register_uart_device(var, config)  #wait for the uart device to create
    await i2c.register_i2c_device(var, config) #use the loaded 12c base config for the i2c device

    if CONF_CONNECTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONNECTED])
        cg.add(var.set_connected_binary_sensor(sens))

    if CONF_FAILED_CRCS in config:
        sens = await sensor.new_sensor(config[CONF_FAILED_CRCS])
        cg.add(var.set_failed_crcs_sensor(sens))

    if CONF_VENT in config:
        sw = await switch.new_switch(config[CONF_VENT], var)
        cg.add(var.set_vent_switch(sw))

    if CONF_ON_DATA_RECEIVED in config:
        for on_data_received in config.get(CONF_ON_DATA_RECEIVED, []):
            data_trigger = cg.new_Pvariable(on_data_received[CONF_TRIGGER_ID], var)
            await automation.build_automation(
                data_trigger, [(cg.std_vector.template(cg.uint8), "x")], on_data_received
            )
### added for bme280 sensor
    if temperature_config := config.get(CONF_TEMPERATURE):
        sens = await sensor.new_sensor(temperature_config)
        cg.add(var.set_temperature_sensor(sens))
        cg.add(var.set_temperature_oversampling(temperature_config[CONF_OVERSAMPLING]))

    if pressure_config := config.get(CONF_PRESSURE):
        sens = await sensor.new_sensor(pressure_config)
        cg.add(var.set_pressure_sensor(sens))
        cg.add(var.set_pressure_oversampling(pressure_config[CONF_OVERSAMPLING]))

    if humidity_config := config.get(CONF_HUMIDITY):
        sens = await sensor.new_sensor(humidity_config)
        cg.add(var.set_humidity_sensor(sens))
        cg.add(var.set_humidity_oversampling(humidity_config[CONF_OVERSAMPLING]))

    cg.add(var.set_iir_filter(config[CONF_IIR_FILTER]))

    ### end of bme280 sensor section
