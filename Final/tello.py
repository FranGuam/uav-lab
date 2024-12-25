import time
import logging
import socket
import threading
from typing import Callable

import numpy as np
import h264decoder

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
file_handler = logging.FileHandler(f'./logs/tello_debug_{timestamp}.log')

console_handler.setLevel(logging.INFO)
file_handler.setLevel(logging.DEBUG)

formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
console_handler.setFormatter(formatter)
file_handler.setFormatter(formatter)

logger.addHandler(console_handler)
logger.addHandler(file_handler)

class Tello:
    """Base class to interact with the Tello drone."""

    def __init__(self, local_ip='', local_port_cmd=8888, local_port_state=8890, local_port_video=11111, tello_ip='192.168.10.1', tello_port=8889, after_receive_state=None, after_receive_video=None):
        """
        Binds to the local IP/port and puts the Tello into command mode.

        :param local_ip (str): Local IP address to bind.
        :param local_port_cmd (int): Local port to bind for sending commands.
        :param local_port_state (int): Local port to bind for receiving state.
        :param local_port_video (int): Local port to bind for receiving video.
        :param tello_ip (str): Tello IP.
        :param tello_port (int): Tello port.
        :param after_receive_state (function): Callback function after receiving state.
        :param after_receive_video (function): Callback function after receiving video.
        """

        self.tello_address = (tello_ip, tello_port)
        
        # Open command mode and video stream on Tello
        self.socket_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_cmd.bind((local_ip, local_port_cmd))
        response = self.send_command('command')
        if response != 'ok':
            raise Exception('Tello failed to enter command mode')
        response = self.send_command('streamon')
        if response != 'ok':
            raise Exception('Tello failed to start video stream')
        
        # Initialize state stream
        self.state = {}
        self.socket_state=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_state.bind((local_ip, local_port_state))
        self.socket_state.settimeout(1)
        self.thread_state = threading.Thread(target=self.receive_state, args=(after_receive_state,), daemon=True)
        self.thread_state.start()

        # Initialize video stream
        self.image = None
        self.decoder = h264decoder.H264Decoder()
        self.socket_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_video.bind((local_ip, local_port_video))
        self.socket_video.settimeout(1)
        self.thread_video = threading.Thread(target=self.receive_video, args=(after_receive_video,), daemon=True)
        self.thread_video.start()

    def __del__(self):
        """Closes the sockets."""
        if hasattr(self, 'socket_cmd') and self.socket_cmd:
            self.socket_cmd.close()
        if hasattr(self, 'socket_state') and self.socket_state:
            self.socket_state.close()
        if hasattr(self, 'socket_video') and self.socket_video:
            self.socket_video.close()
    
    def send_command(self, command: str, timeout=1):
        """
        Send a command to the Tello and wait for a response.

        :param command: Command to send.
        :param timeout: Time (in seconds) to wait for a response.
        :return (str): Response from Tello.

        """
        self.socket_cmd.sendto(command.encode('utf-8'), self.tello_address)
        logger.info("Sending Command: {}".format(command))
        if timeout:
            self.socket_cmd.settimeout(timeout)
        try:
            response, _ = self.socket_cmd.recvfrom(256)
            if response:
                response = response.decode('utf-8')
                logger.info("Response: {}".format(response))
                return response
        except socket.timeout:
            logger.warning("Command timed out: {}".format(command))
            return None
        finally:
            self.socket_cmd.settimeout(None)

    def receive_state(self, after_receive: Callable):
        """Receive state from Tello."""
        while True:
            try:
                states, _ = self.socket_state.recvfrom(1024)
                states = states.decode('utf-8').strip()
                logger.debug("Received State: {}".format(states))
                states = states.split(';')
                self.state = {}
                for state in states:
                    if state:
                        key, value = state.split(':')
                        if key == 'mpry':
                            value = value.split(',')
                            self.state[key] = [int(x) for x in value]
                        elif key in ['baro', 'agx', 'agy', 'agz']:
                            self.state[key] = float(value)
                        else:
                            self.state[key] = int(value)
                if after_receive:
                    after_receive()
            except socket.error as err:
                logger.error("Socket Error : %s" % err)
            except socket.timeout:
                logger.warning("Socket timeout")

    def receive_video(self, after_receive: Callable):
        """Receive video data from Tello."""
        packet_data = b''
        while True:
            try:
                response, _ = self.socket_video.recvfrom(2048)
                packet_data += response
                if len(response) != 1460: # Detect the end of a frame
                    frames = self.decoder.decode(packet_data)
                    for framedata in frames:
                        (frame, w, h, rowsize) = framedata
                        logger.debug("Received Video Frame: %dx%d" % (w, h))
                        if frame is not None:
                            image = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='')
                            image = image.reshape((h, int(rowsize/3), 3))
                            self.image = image[:, :w, :]
                        if after_receive:
                            after_receive()
                    packet_data = b''
            except socket.error as err:
                packet_data = b''
                logger.error("Socket Error : %s" % err)
            except socket.timeout:
                packet_data = b''
                logger.warning("Socket timeout")

    def get_state(self):
        """Return the current state of the Tello."""
        return self.state
    
    def get_image(self):
        """Return the current image from the camera."""
        return self.image


class TelloROS(Tello):
    """Wrapper class to interact with the Tello drone using ROS."""

    def __init__(self, local_ip='', local_port_cmd=8888, local_port_state=8890, local_port_video=11111, tello_ip='192.168.10.1', tello_port=8889):
        """
        Binds to the local IP/port and puts the Tello into command mode.

        :param local_ip (str): Local IP address to bind.
        :param local_port_cmd (int): Local port to bind for sending commands.
        :param local_port_state (int): Local port to bind for receiving state.
        :param local_port_video (int): Local port to bind for receiving video.
        :param tello_ip (str): Tello IP.
        :param tello_port (int): Tello port.
        """

        self.command_pub = rospy.Publisher('tello_command', String, queue_size=1)
        self.state_pub = rospy.Publisher('tello_state', String, queue_size=1)
        self.img_pub = rospy.Publisher('tello_image', Image, queue_size=1)
        self.bridge = CvBridge()
        super().__init__(local_ip, local_port_cmd, local_port_state, local_port_video, tello_ip, tello_port, self.publish_state, self.publish_image)

    def __del__(self):
        """Closes the sockets."""
        super().__del__()

    def publish_state(self):
        """Publish the current state of the Tello."""
        self.state_pub.publish(str(self.get_state()))

    def publish_image(self):
        """Publish the current image from the camera."""
        try:
            img_msg = self.bridge.cv2_to_imgmsg(self.get_image(), 'rgb8')
            img_msg.header.frame_id = rospy.get_namespace()
        except CvBridgeError as err:
            rospy.logerr('CV bridge failed: %s' % str(err))
            return
        self.img_pub.publish(img_msg)

    def send_command(self, command: str, timeout=1):
        """
        Send a command to the Tello and wait for a response.

        :param command: Command to send.
        :param timeout: Time (in seconds) to wait for a response.
        :return (str): Response from Tello.

        """
        self.command_pub.publish("Sending Command: %s" % command)
        response = super().send_command(command, timeout)
        if response:
            self.command_pub.publish("Response: %s" % response)
        else:
            self.command_pub.publish("Command timed out: %s" % command)
        return response
