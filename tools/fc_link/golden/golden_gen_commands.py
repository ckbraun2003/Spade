# Runs INSIDE the pinned px4-dev container, network off, script on stdin (the same image and form as golden_gen.py):
#   docker run --rm -i --pull never --network none --entrypoint python3 px4io/px4-dev@sha256:82e3ddcf4bf3fa4dbc0be6489c6de4158d0efbc8530182c691184302027335ce - < golden_gen_commands.py
# Makes the reference MAVLink v2 frames (pymavlink, common dialect) for the four flight commands the video #3 script sends, which
# tests/test_fc_link_mavlink.cpp pins the codec's helpers against: arm (MAV_CMD_COMPONENT_ARM_DISARM 400, param1 1), disarm (the same, param1 0),
# takeoff (MAV_CMD_NAV_TAKEOFF 22, yaw and position NaN = unchanged / here, altitude in param7, AMSL) and land (MAV_CMD_NAV_LAND 21, yaw and
# position NaN); and the COMMAND_ACK frames PX4 answers them with (accepted, temporarily rejected, denied, for 400 and 22), with the
# message's CRC_EXTRA and wire order as pymavlink has them. Reads nothing, writes nothing but its stdout.
import os, json
os.environ['MAVLINK20'] = '1'
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

nan = float('nan')
add('command_long_arm', 11, m.MAVLink_command_long_message(target_system=1, target_component=1, command=400, confirmation=0,
    param1=1.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0))
add('command_long_disarm', 12, m.MAVLink_command_long_message(target_system=1, target_component=1, command=400, confirmation=0,
    param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0))
add('command_long_takeoff', 13, m.MAVLink_command_long_message(target_system=1, target_component=1, command=22, confirmation=0,
    param1=0.0, param2=0.0, param3=0.0, param4=nan, param5=nan, param6=nan, param7=490.0))
add('command_long_land', 14, m.MAVLink_command_long_message(target_system=1, target_component=1, command=21, confirmation=0,
    param1=0.0, param2=0.0, param3=0.0, param4=nan, param5=nan, param6=nan, param7=0.0))
# COMMAND_ACK: PX4 answers from system 1 component 1, addressed to the sender (target 1/191). result: MAV_RESULT 0 accepted, 1 temporarily
# rejected, 2 denied. The last frame carries the extension fields (progress, result_param2) nonzero.
for cmd, cname, seq in ((400, 'arm', 15), (22, 'takeoff', 18)):
    for res, rname in ((0, 'accepted'), (1, 'temporarily_rejected'), (2, 'denied')):
        add('command_ack_%s_%s' % (cname, rname), seq + res, m.MAVLink_command_ack_message(command=cmd, result=res, progress=0, result_param2=0, target_system=1, target_component=191))
add('command_ack_extension_fields', 21, m.MAVLink_command_ack_message(command=22, result=2, progress=255, result_param2=-3, target_system=1, target_component=191))
c77 = m.mavlink_map[77]
out['messages_info'] = {'77': {'name': 'COMMAND_ACK', 'crc_extra': c77.crc_extra, 'wire_order': list(c77.ordered_fieldnames)}}
print('JSON-BEGIN')
print(json.dumps(out, indent=1))
print('JSON-END')
