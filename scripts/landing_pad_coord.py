import krpc
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--host_ip', nargs=1, default=['127.0.0.1'])
args = parser.parse_args()

conn = krpc.connect(name='Launch Site', address=args.host_ip[0])
vessel = conn.space_center.active_vessel

earth_reference_frame = vessel.orbit.body.reference_frame
position = vessel.position(earth_reference_frame)

print('x: {:f}, y: {:f}, z: {:f}'.format(position[0], position[1], position[2]))
