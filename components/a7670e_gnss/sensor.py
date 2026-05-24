import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, uart
from esphome.const import (
    CONF_ALTITUDE,
    CONF_ID,
    CONF_LATITUDE,
    CONF_LONGITUDE,
    CONF_SPEED,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_SPEED,
    STATE_CLASS_MEASUREMENT,
    UNIT_KILOMETER_PER_HOUR,
    UNIT_METER,
)

AUTO_LOAD = ["sensor"]
DEPENDENCIES = ["uart"]

a7670e_gnss_ns = cg.esphome_ns.namespace("a7670e_gnss")
A7670EGNSSComponent = a7670e_gnss_ns.class_(
    "A7670EGNSSComponent", cg.PollingComponent, uart.UARTDevice
)

CONF_COURSE = "course"
CONF_FIX_MODE = "fix_mode"
CONF_HDOP = "hdop"
CONF_SATELLITES = "satellites"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(A7670EGNSSComponent),
            cv.Optional(CONF_LATITUDE): sensor.sensor_schema(
                unit_of_measurement="°",
                accuracy_decimals=6,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_LONGITUDE): sensor.sensor_schema(
                unit_of_measurement="°",
                accuracy_decimals=6,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ALTITUDE): sensor.sensor_schema(
                unit_of_measurement=UNIT_METER,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_DISTANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SPEED): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_SPEED,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_COURSE): sensor.sensor_schema(
                unit_of_measurement="°",
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_HDOP): sensor.sensor_schema(
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SATELLITES): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_FIX_MODE): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("30s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if latitude_config := config.get(CONF_LATITUDE):
        sens = await sensor.new_sensor(latitude_config)
        cg.add(var.set_latitude_sensor(sens))

    if longitude_config := config.get(CONF_LONGITUDE):
        sens = await sensor.new_sensor(longitude_config)
        cg.add(var.set_longitude_sensor(sens))

    if altitude_config := config.get(CONF_ALTITUDE):
        sens = await sensor.new_sensor(altitude_config)
        cg.add(var.set_altitude_sensor(sens))

    if speed_config := config.get(CONF_SPEED):
        sens = await sensor.new_sensor(speed_config)
        cg.add(var.set_speed_sensor(sens))

    if course_config := config.get(CONF_COURSE):
        sens = await sensor.new_sensor(course_config)
        cg.add(var.set_course_sensor(sens))

    if hdop_config := config.get(CONF_HDOP):
        sens = await sensor.new_sensor(hdop_config)
        cg.add(var.set_hdop_sensor(sens))

    if satellites_config := config.get(CONF_SATELLITES):
        sens = await sensor.new_sensor(satellites_config)
        cg.add(var.set_satellites_sensor(sens))

    if fix_mode_config := config.get(CONF_FIX_MODE):
        sens = await sensor.new_sensor(fix_mode_config)
        cg.add(var.set_fix_mode_sensor(sens))
