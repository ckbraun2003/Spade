# Runs INSIDE the pinned px4-dev container, network off, script on stdin:
#   docker run --rm -i --network none --entrypoint python3 px4io/px4-dev@sha256:82e3ddcf4bf3fa4dbc0be6489c6de4158d0efbc8530182c691184302027335ce - < golden_gen.py
# Makes the reference MAVLink v2 frames (pymavlink, common dialect) that tests/test_fc_link_mavlink.cpp pins the codec against.
import os, json, struct
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
import pymavlink
from pymavlink.dialects.v20 import common as m

class Sink:
    def write(self, b):
        pass

def mav(seq):
    c = m.MAVLink(Sink(), srcSystem=1, srcComponent=200)
    c.seq = seq
    return c

out = {'pymavlink_version': getattr(pymavlink, '__version__', 'unknown'), 'dialect': 'v20.common', 'frames': []}

def add(name, seq, msg):
    c = mav(seq)
    b = msg.pack(c)
    out['frames'].append({'name': name, 'seq': seq, 'sysid': 1, 'compid': 200, 'msgid': msg.get_msgId(), 'hex': bytes(b).hex(), 'len': len(b) - 12})

add('heartbeat', 0, m.MAVLink_heartbeat_message(type=2, autopilot=12, base_mode=0x81, custom_mode=0x01020304, system_status=4, mavlink_version=3))
add('heartbeat_zeros_truncated', 1, m.MAVLink_heartbeat_message(type=0, autopilot=0, base_mode=0, custom_mode=0, system_status=0, mavlink_version=3))
add('command_long_set_message_interval', 2, m.MAVLink_command_long_message(target_system=1, target_component=1, command=511, confirmation=0, param1=115.0, param2=5000.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0))
add('hil_sensor_full', 3, m.MAVLink_hil_sensor_message(time_usec=1234567890123, xacc=0.25, yacc=-0.5, zacc=-9.80665, xgyro=0.01, ygyro=-0.02, zgyro=0.03,
    xmag=0.2, ymag=0.01, zmag=0.4, abs_pressure=1013.25, diff_pressure=0.0, pressure_alt=0.0, temperature=15.0, fields_updated=0x1fff, id=0))
add('hil_sensor_imu_only_id1', 4, m.MAVLink_hil_sensor_message(time_usec=4000, xacc=0.0, yacc=0.0, zacc=-9.80665, xgyro=0.0, ygyro=0.0, zgyro=0.0,
    xmag=0.0, ymag=0.0, zmag=0.0, abs_pressure=0.0, diff_pressure=0.0, pressure_alt=0.0, temperature=0.0, fields_updated=0x3f, id=1))
add('hil_gps_with_yaw', 5, m.MAVLink_hil_gps_message(time_usec=987654321, fix_type=3, lat=473977418, lon=85455939, alt=488000, eph=100, epv=150, vel=523,
    vn=300, ve=-400, vd=-50, cog=31500, satellites_visible=10, id=0, yaw=4500))
add('hil_gps_no_ext', 6, m.MAVLink_hil_gps_message(time_usec=1000, fix_type=3, lat=-335000000, lon=-1511000000, alt=-2000, eph=65535, epv=65535, vel=0,
    vn=0, ve=0, vd=0, cog=65535, satellites_visible=255, id=0, yaw=0))
add('hil_actuator_controls_armed', 7, m.MAVLink_hil_actuator_controls_message(time_usec=8000, controls=[0.5, 0.6, 0.7, 0.8] + [0.0] * 12, mode=129, flags=1))
add('hil_actuator_controls_disarmed', 8, m.MAVLink_hil_actuator_controls_message(time_usec=12000, controls=[0.0] * 16, mode=0, flags=0))
add('hil_actuator_controls_servos_negative', 9, m.MAVLink_hil_actuator_controls_message(time_usec=16000, controls=[1.0, 0.25, 0.0, 0.75, 0, 0, 0, 0, -1.0, 0, 0, 0, 0, 0, 0, 0.125], mode=129, flags=1))
add('hil_state_quaternion', 10, m.MAVLink_hil_state_quaternion_message(time_usec=2000000, attitude_quaternion=[0.70710678, 0.0, 0.0, -0.70710678], rollspeed=0.1, pitchspeed=-0.2, yawspeed=0.3,
    lat=473977418, lon=85455939, alt=488000, vx=100, vy=-200, vz=-300, ind_airspeed=0, true_airspeed=0, xacc=0, yacc=0, zacc=-1000))

# CRC_EXTRA and lengths for the messages we use (from the dialect tables)
info = {}
for i in (0, 76, 93, 107, 113, 115):
    c = m.mavlink_map[i]
    info[str(i)] = {'name': getattr(c, 'msgname', None) or getattr(c, 'name', None), 'crc_extra': c.crc_extra,
                    'wire_order': list(c.ordered_fieldnames)}
out['messages_info'] = info
# CRC-16/MCRF4XX check value
crc = 0xFFFF
for ch in b'123456789':
    tmp = ch ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
out['crc_check_123456789'] = hex(crc)
print('JSON-BEGIN')
print(json.dumps(out, indent=1))
print('JSON-END')
