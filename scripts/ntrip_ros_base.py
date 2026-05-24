#!/usr/bin/env python

import os
import json
import datetime
import importlib.util

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

from ntrip_client.ntrip_base import NTRIPBase
from ntrip_client.ntrip_client import NTRIPClient
from ntrip_client.nmea_parser import NMEAParser, NMEA_DEFAULT_MAX_LENGTH, NMEA_DEFAULT_MIN_LENGTH

# Try to import a couple different types of RTCM messages
_MAVROS_MSGS_NAME = "mavros_msgs"
_RTCM_MSGS_NAME = "rtcm_msgs"
_PX4_MSGS_NAME = "px4_msgs"
_PX4_GPS_INJECT_DATA_TOPIC = '/fmu/in/gps_inject_data'
_PX4_GPS_INJECT_DATA_MAX_LEN = 300
_PX4_GPS_INJECT_DATA_FRAGMENTED = 1
have_mavros_msgs = False
have_rtcm_msgs = False
have_px4_msgs = False
if importlib.util.find_spec(_MAVROS_MSGS_NAME) is not None:
  have_mavros_msgs = True
  from mavros_msgs.msg import RTCM as mavros_msgs_RTCM
if importlib.util.find_spec(_RTCM_MSGS_NAME) is not None:
  have_rtcm_msgs = True
  from rtcm_msgs.msg import Message as rtcm_msgs_RTCM
if importlib.util.find_spec(_PX4_MSGS_NAME) is not None:
  have_px4_msgs = True
  from px4_msgs.msg import GpsInjectData as px4_msgs_GpsInjectData

class NTRIPRosBase(Node):
  def __init__(self, name):
    # Read a debug flag from the environment that should have been set by the launch file
    try:
      self._debug = json.loads(os.environ["NTRIP_CLIENT_DEBUG"].lower())
    except:
      self._debug = False

    # Init the node and declare params
    super().__init__(name)
    self.declare_parameters(
      namespace='',
      parameters=[
        ('rtcm_frame_id', 'odom'),
        ('nmea_max_length', NMEA_DEFAULT_MAX_LENGTH),
        ('nmea_min_length', NMEA_DEFAULT_MIN_LENGTH),
        ('rtcm_message_package', _MAVROS_MSGS_NAME),
        ('px4_gps_device_id', 0),
        ('reconnect_attempt_max', NTRIPBase.DEFAULT_RECONNECT_ATTEMPT_MAX),
        ('reconnect_attempt_wait_seconds', NTRIPBase.DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS),
      ]
    )

    # Set the log level to debug if debug is true
    if self._debug:
      rclpy.logging.set_logger_level(self.get_logger().name, rclpy.logging.LoggingSeverity.DEBUG)

    # Read an optional Frame ID from the config
    self._rtcm_frame_id = self.get_parameter('rtcm_frame_id').value
    self._px4_gps_device_id = self.get_parameter('px4_gps_device_id').value

    self._rtcm_topic = 'rtcm'
    self._rtcm_qos = 10

    # Determine the type of RTCM message that will be published
    rtcm_message_package = self.get_parameter('rtcm_message_package').value
    if rtcm_message_package == _MAVROS_MSGS_NAME:
      if have_mavros_msgs:
        self._rtcm_message_type = mavros_msgs_RTCM
        self._create_rtcm_messages = self._create_mavros_msgs_rtcm_messages
      else:
        self.get_logger().fatal('The requested RTCM package {} is a valid option, but we were unable to import it. Please make sure you have it installed'.format(rtcm_message_package))
    elif rtcm_message_package == _RTCM_MSGS_NAME:
      if have_rtcm_msgs:
        self._rtcm_message_type = rtcm_msgs_RTCM
        self._create_rtcm_messages = self._create_rtcm_msgs_rtcm_messages
      else:
        self.get_logger().fatal('The requested RTCM package {} is a valid option, but we were unable to import it. Please make sure you have it installed'.format(rtcm_message_package))
    elif rtcm_message_package == _PX4_MSGS_NAME:
      if have_px4_msgs:
        self._rtcm_message_type = px4_msgs_GpsInjectData
        self._create_rtcm_messages = self._create_px4_msgs_rtcm_messages
        self._rtcm_topic = _PX4_GPS_INJECT_DATA_TOPIC
        self._rtcm_qos = QoSProfile(
          history=HistoryPolicy.KEEP_LAST,
          depth=10,
          reliability=ReliabilityPolicy.BEST_EFFORT,
          durability=DurabilityPolicy.VOLATILE,
        )
      else:
        self.get_logger().fatal('The requested RTCM package {} is a valid option, but we were unable to import it. Please make sure you have it installed'.format(rtcm_message_package))
    else:
      self.get_logger().fatal('The RTCM package {} is not a valid option. Please choose between the following packages {}'.format(rtcm_message_package, ','.join([_MAVROS_MSGS_NAME, _RTCM_MSGS_NAME, _PX4_MSGS_NAME])))

    # Setup the RTCM publisher
    self._rtcm_pub = self.create_publisher(self._rtcm_message_type, self._rtcm_topic, self._rtcm_qos)

    # Initialize the client
    self._client = NTRIPBase(
      logerr=self.get_logger().error,
      logwarn=self.get_logger().warning,
      loginfo=self.get_logger().info,
      logdebug=self.get_logger().debug
    )

    # Get some timeout parameters for the NTRIP client
    self._nmea_max_length = self.get_parameter('nmea_max_length').value
    self._nmea_min_length = self.get_parameter('nmea_min_length').value
    self._reconnect_attempt_max = self.get_parameter('reconnect_attempt_max').value
    self._reconnect_attempt_wait_seconds = self.get_parameter('reconnect_attempt_wait_seconds').value

  def run(self):
    # Connect the client
    if not self._client.connect():
      self.get_logger().error('Unable to connect')
      return False
    # Setup our subscribers
    self._nmea_sub = self.create_subscription(Sentence, 'nmea', self.subscribe_nmea, 10)
    self._fix_sub = self.create_subscription(NavSatFix, 'fix', self.subscribe_fix, 10)

    # Start the timer that will check for RTCM data
    self._rtcm_timer = self.create_timer(0.1, self.publish_rtcm)
    return True

  def stop(self):
    self.get_logger().info('Stopping RTCM publisher')
    if self._rtcm_timer:
      self._rtcm_timer.cancel()
      self._rtcm_timer.destroy()
    self.get_logger().info('Disconnecting NTRIP client')
    self._client.disconnect()
    self.get_logger().info('Shutting down node')
    self.destroy_node()

  def subscribe_nmea(self, nmea):
    # Just extract the NMEA from the message, and send it right to the server
    self._client.send_nmea(nmea.sentence)
  
  def subscribe_fix(self, fix: NavSatFix):
    # Calculate the timestamp of the message
    timestamp_secs = fix.header.stamp.sec + fix.header.stamp.nanosec * 1e-9
    timestamp = datetime.datetime.fromtimestamp(timestamp_secs)
    time = timestamp.time()
    hour = time.hour
    minute = time.minute
    second = time.second
    millisecond = int(time.microsecond * 1e-4)
    nmea_utc = f"{hour:02}{minute:02}{second:02}.{millisecond:02}"

    # Figure out the direction of the latitude and longitude
    nmea_lat_direction = "N"
    nmea_lon_direction = "E"
    if fix.latitude < 0:
      nmea_lat_direction = "S"
    if fix.longitude < 0:
      nmea_lon_direction = "W"
    
    # Convert the units of the latitude and longitude
    nmea_lat = NMEAParser.lat_dd_to_dmm(fix.latitude)
    nmea_lon = NMEAParser.lon_dd_to_dmm(fix.longitude)

    # Convert the GPS quality to the right format for the sentence
    status = fix.status.status
    if status == NavSatStatus.STATUS_FIX:
      nmea_status = 1
    elif status == NavSatStatus.STATUS_SBAS_FIX:
      nmea_status = 2
    elif status == NavSatStatus.STATUS_GBAS_FIX:
      nmea_status = 5
    else:
      nmea_status = 0

    # Assemble the sentence
    nmea_sentence_no_checksum = f"$GPGGA,{nmea_utc},{nmea_lat},{nmea_lat_direction},{nmea_lon},{nmea_lon_direction},{nmea_status},05,1.0,100.0,M,-32.0,M,,0000"
    nmea_checksum = NMEAParser.checksum(nmea_sentence_no_checksum)
    nmea_sentence = f"{nmea_sentence_no_checksum}*{nmea_checksum:x}\r\n"

    # Send the sentence to the client
    self._client.send_nmea(nmea_sentence)

  def publish_rtcm(self):
    for raw_rtcm in self._client.recv_rtcm():
      for rtcm_message in self._create_rtcm_messages(raw_rtcm):
        self._rtcm_pub.publish(rtcm_message)

  def _create_mavros_msgs_rtcm_messages(self, rtcm):
    return [mavros_msgs_RTCM(
      header=Header(
        stamp=self.get_clock().now().to_msg(),
        frame_id=self._rtcm_frame_id
      ),
      data=rtcm
    )]

  def _create_rtcm_msgs_rtcm_messages(self, rtcm):
    return [rtcm_msgs_RTCM(
      header=Header(
        stamp=self.get_clock().now().to_msg(),
        frame_id=self._rtcm_frame_id
      ),
      message=rtcm
    )]

  def _create_px4_msgs_rtcm_messages(self, rtcm):
    rtcm_data = bytes(rtcm)
    if not rtcm_data:
      return []

    fragmented = len(rtcm_data) > _PX4_GPS_INJECT_DATA_MAX_LEN
    flags = _PX4_GPS_INJECT_DATA_FRAGMENTED if fragmented else 0
    messages = []

    for offset in range(0, len(rtcm_data), _PX4_GPS_INJECT_DATA_MAX_LEN):
      chunk = list(rtcm_data[offset:offset + _PX4_GPS_INJECT_DATA_MAX_LEN])
      chunk_data = [0] * _PX4_GPS_INJECT_DATA_MAX_LEN
      chunk_data[:len(chunk)] = chunk
      messages.append(
        px4_msgs_GpsInjectData(
          timestamp=int(self.get_clock().now().nanoseconds / 1000),
          device_id=self._px4_gps_device_id,
          len=len(chunk),
          flags=flags,
          data=chunk_data,
        )
      )

    return messages