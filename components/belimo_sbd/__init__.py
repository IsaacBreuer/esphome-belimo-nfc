"""Read-only Belimo NFC (NXP NTAG I2C plus, MP tunnel) reader for ESPHome, using a PN532."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import pn532, sensor
from esphome.const import CONF_ID

CODEOWNERS = ["@isaacbreuer"]
DEPENDENCIES = ["pn532"]
AUTO_LOAD = ["sensor"]

CONF_PN532_ID = "pn532_id"
CONF_SENSORS = "sensors"
CONF_POOL_ID = "pool_id"
CONF_SCALE = "scale"
CONF_PROTOCOL_VERSION = "protocol_version"
CONF_DUMP_MEMORY = "dump_memory"
CONF_STALE_TIMEOUT = "stale_timeout"

belimo_sbd_ns = cg.esphome_ns.namespace("belimo_sbd")
BelimoSbd = belimo_sbd_ns.class_("BelimoSbd", cg.PollingComponent)

POOL_SENSOR_SCHEMA = sensor.sensor_schema(accuracy_decimals=2).extend(
    {
        cv.Required(CONF_POOL_ID): cv.int_range(min=0, max=0xFFFF),
        cv.Optional(CONF_SCALE, default=1.0): cv.float_,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BelimoSbd),
        cv.Required(CONF_PN532_ID): cv.use_id(pn532.PN532),
        # 0 = auto-detect with the Get-BNPV query, 2 or 3 = force
        cv.Optional(CONF_PROTOCOL_VERSION, default=0): cv.one_of(0, 2, 3, int=True),
        # Log a read-only dump of the NTAG user memory once per tag
        cv.Optional(CONF_DUMP_MEMORY, default=True): cv.boolean,
        # How long to keep the last values before marking sensors unknown on sustained failure
        cv.Optional(CONF_STALE_TIMEOUT, default="20s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_SENSORS, default=[]): cv.ensure_list(POOL_SENSOR_SCHEMA),
    }
).extend(cv.polling_component_schema("10s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    reader = await cg.get_variable(config[CONF_PN532_ID])
    cg.add(var.set_pn532(reader))
    cg.add(var.set_protocol_version(config[CONF_PROTOCOL_VERSION]))
    cg.add(var.set_dump_memory(config[CONF_DUMP_MEMORY]))
    cg.add(var.set_stale_timeout(config[CONF_STALE_TIMEOUT]))
    for conf in config[CONF_SENSORS]:
        sens = await sensor.new_sensor(conf)
        cg.add(var.add_pool_sensor(conf[CONF_POOL_ID], conf[CONF_SCALE], sens))
