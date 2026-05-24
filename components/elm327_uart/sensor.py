import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, uart
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_SPEED,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_KILOMETER_PER_HOUR,
    UNIT_PERCENT,
    UNIT_REVOLUTIONS_PER_MINUTE,
    UNIT_VOLT,
)

AUTO_LOAD = ["sensor"]
DEPENDENCIES = ["uart"]

elm327_uart_ns = cg.esphome_ns.namespace("elm327_uart")
ELM327UARTComponent = elm327_uart_ns.class_(
    "ELM327UARTComponent", cg.PollingComponent, uart.UARTDevice
)

CONF_COOLANT_TEMPERATURE = "coolant_temperature"
CONF_CONTROL_MODULE_VOLTAGE = "control_module_voltage"
CONF_RPM = "rpm"
CONF_SPEED = "speed"
CONF_THROTTLE_POSITION = "throttle_position"
CONF_INTAKE_AIR_TEMPERATURE = "intake_air_temperature"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ELM327UARTComponent),
            cv.Optional(CONF_RPM): sensor.sensor_schema(
                unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SPEED): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOMETER_PER_HOUR,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_SPEED,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_COOLANT_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CONTROL_MODULE_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_THROTTLE_POSITION): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_INTAKE_AIR_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("2s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if rpm_config := config.get(CONF_RPM):
        sens = await sensor.new_sensor(rpm_config)
        cg.add(var.set_rpm_sensor(sens))

    if speed_config := config.get(CONF_SPEED):
        sens = await sensor.new_sensor(speed_config)
        cg.add(var.set_speed_sensor(sens))

    if coolant_config := config.get(CONF_COOLANT_TEMPERATURE):
        sens = await sensor.new_sensor(coolant_config)
        cg.add(var.set_coolant_temperature_sensor(sens))

    if voltage_config := config.get(CONF_CONTROL_MODULE_VOLTAGE):
        sens = await sensor.new_sensor(voltage_config)
        cg.add(var.set_control_module_voltage_sensor(sens))

    if throttle_config := config.get(CONF_THROTTLE_POSITION):
        sens = await sensor.new_sensor(throttle_config)
        cg.add(var.set_throttle_position_sensor(sens))

    if intake_config := config.get(CONF_INTAKE_AIR_TEMPERATURE):
        sens = await sensor.new_sensor(intake_config)
        cg.add(var.set_intake_air_temperature_sensor(sens))
