
import time
from NatNetClient import NatNetClient
from util import quaternion_to_euler

positions = {}
rotations = {}


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(id, position, rotation_quaternion):
    # Position and rotation received
    positions[id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler(rotation_quaternion)
    # Store the roll pitch and yaw angles
    rotations[id] = (rotx, roty, rotz)



if __name__ == "__main__":
    # CHANGE THIS: Match your PC's actual IPv4 address from ipconfig
    clientAddress = "192.168.0.25"
    # CHANGE THIS: Match your Motive PC's IP address (from your earlier run logs)
    optitrackServerAddress = "192.168.0.4"
    # CHANGE THIS: Set to your blimp/rigid body ID (from your earlier logs, body was 531)
    robot_id = 531

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    while is_running:
        if robot_id in positions:
            # last position
            print('Last position', positions[robot_id], ' rotation', rotations[robot_id])

        time.sleep(.1)
