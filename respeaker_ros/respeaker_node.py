import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.node import Node
from .util import ignore_stderr
from audio_common_msgs.msg import AudioData
from std_msgs.msg import  Int32, Bool
import pyaudio
import math
import angles

from respeaker_ros.interface import RespeakerInterface

class AudioPublisher(Node):
    def __init__(self, suppress_error=True):
        super().__init__('respeaker_publisher')
        
        #parameters
        self.doa_xy_offset = self.declare_parameter('doa_xy_offset', 0.0)
        self.doa_yaw_offset = self.declare_parameter('doa_yaw_offset', 90.0)

        self.respeaker = RespeakerInterface() #mic-array initialisation

        with ignore_stderr(enable=suppress_error): # ALSA error suppresion
            self.audio_interface = pyaudio.PyAudio()

        self.stream = self.audio_interface.open(format=pyaudio.paInt16, 
                                                channels=1,
                                                rate=16000,
                                                input=True,
                                                frames_per_buffer=1024) #audio stream opening
        
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.prev_is_voice = None
        self.prev_doa = None
        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        self.publisher_ = self.create_publisher(AudioData, 'audio', 10) #audio stream
        self._pub_vad = self.create_publisher(Bool, 'vad', latching_qos) #can be used for taking only when the mic-array takes signal in
        self._pub_doa_raw = self.create_publisher(Int32, 'doa_raw', latching_qos) #degree of audio 

    def timer_callback(self):
        data = self.stream.read(1600)
        msg = AudioData()
        msg.data = data
        self.publisher_.publish(msg)
        is_voice = self.respeaker.is_voice()
        doa_rad = math.radians(self.respeaker.direction - 180.0)
        doa_rad = angles.shortest_angular_distance(
            doa_rad, math.radians(self.doa_yaw_offset.value))
        doa = math.degrees(doa_rad)

        # vad
        if is_voice != self.prev_is_voice:
            self._pub_vad.publish(Bool(data=is_voice == 1))
            self.prev_is_voice = is_voice

        # doa
        if doa != self.prev_doa:
            self._pub_doa_raw.publish(Int32(data=int(doa)))
            self.prev_doa = doa

    def __del__(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio_interface.terminate()

def main(args=None):
    rclpy.init(args=args)

    audio_publisher = AudioPublisher()

    try:
        rclpy.spin(audio_publisher)
    except KeyboardInterrupt:
        pass

    audio_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
