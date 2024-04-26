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
from respeaker_ros.audio import RespeakerAudio

class AudioPublisher(Node):
    def __init__(self, suppress_error=True):
        super().__init__('respeaker_publisher')

        self.sensor_frame_id = self.declare_parameter('sensor_frame_id', 'respeaker_base')
        self.speech_prefetch = self.declare_parameter('speech_prefetch', 0.5)
        self.update_period_s = self.declare_parameter('update_period_s', 0.1)
        self.main_channel = self.declare_parameter('main_channel', 0)
        self.speech_continuation = self.declare_parameter('speech_continuation', 0.5)
        self.speech_max_duration = self.declare_parameter('speech_max_duration', 7.0)
        self.speech_min_duration = self.declare_parameter('speech_min_duration', 0.1)
        self.doa_xy_offset = self.declare_parameter('doa_xy_offset', 0.0)
        self.doa_yaw_offset = self.declare_parameter('doa_yaw_offset', 90.0)

        self.respeaker = RespeakerInterface()

        with ignore_stderr(enable=suppress_error): # ALSA error suppresion
            self.audio_interface = pyaudio.PyAudio()

        self.stream = self.audio_interface.open(format=pyaudio.paInt16,
                                                channels=1,
                                                rate=16000,
                                                input=True,
                                                frames_per_buffer=1024)
        
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.is_speaking = False
        self.speech_stopped = self.get_clock().now()
        self.prev_is_voice = None
        self.prev_doa = None
        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        self.publisher_ = self.create_publisher(AudioData, 'audio', 10)
        self._pub_vad = self.create_publisher(Bool, 'vad', latching_qos)
        self._pub_doa_raw = self.create_publisher(Int32, 'doa_raw', latching_qos)

    def timer_callback(self):
        data = self.stream.read(1024)
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
