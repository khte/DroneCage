#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Descriptors: TL = Tobias Lundby (tobiaslundby@gmail.com)
2018-06-04 TL First version
"""

import rospy
from math import sin, cos, pi
import time
import serial
from multiprocessing import Lock
from utm import utmconv
from mavlink_lora.msg import mavlink_lora_pos, mavlink_lora_attitude

# Defines
RX_CENTER_POSITION = {
    'throttle': 50,
    'yaw': 50,
    'pitch': 50,
    'roll': 50,
}
DRONEBUR_LIMITS = {
    'x_min': -1,
    'x_max': 1,
    'y_min': -1,
    'y_max': 1,
    'z_min': 0,
    'z_max': 3
}
FACTOR_INPUT2METRIC_XY  = 0.01
FACTOR_INPUT2METRIC_Z   = 0.1
ARDUINO_BAUD_RATE       = 115200
ARDUINO_PORT            = '/dev/ttyUSB0'
ROS_POSE_TOPIC          = '/mavlink_pos'
ROS_ATTITUDE_TOPIC      = '/mavlink_attitude'
ROS_STATUS_TOPIC        = '/mavlink_status'

DEG2RAD = pi/180
RAD2DEG = 180/pi

class uav_restriction():
    ROS_NODE_NAME = 'uav_restriction'
    mutex_heading = Lock()
    def __init__(self, debug = False):
        self.debug = debug

        rospy.init_node(self.ROS_NODE_NAME)
        rospy.loginfo('Started: '+rospy.get_caller_id())

        rospy.loginfo('Configuring UTM for zone 32')
        self.uc = utmconv()
        self.uc.set_zone_override(32)
        #Center positions
        self.zeroEasting = 590734.045671
        self.zeroNorthing = 6136563.68892

        rospy.loginfo('Configuring arduino serial port')
        self.ser_arduino = serial.Serial()
        self.ser_arduino.baudrate = ARDUINO_BAUD_RATE
        self.ser_arduino.port = ARDUINO_PORT
        #rospy.loginfo(ser)
        rospy.loginfo('Opening arduino serial port')
        try:
            self.ser_arduino.open()
        except serial.SerialException as e:
            rospy.logerr('Could not open arduino serial port')
        else:
            with self.mutex_heading:
                self.uav_heading_rad = 0
                self.uav_heading_deg = self.uav_heading_rad*RAD2DEG

            rospy.Subscriber(ROS_POSE_TOPIC, mavlink_lora_pos, self.callback_pos)
            rospy.Subscriber(ROS_ATTITUDE_TOPIC, mavlink_lora_attitude, self.callback_attitude)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        rospy.loginfo('Closing arduino serial port')
        while self.ser_arduino.is_open:
            self.ser_arduino.close()
        rospy.loginfo('Arduino serial port closed')
        rospy.loginfo('Stopped '+rospy.get_caller_id())

    def send_arduino_message(thr_high, thr_low, pitch_high, pitch_low, roll_high, roll_low):
        if self.ser_arduino != None and self.ser_arduino.is_open:
            values = bytearray([b'P', thr_high, thr_low, pitch_high, pitch_low, roll_high, roll_low])
            try:
                self.ser_arduino.write(values)
            except serial.SerialException as e:
                rospy.logerr('Could not write to arduino')
                return False
            else:
                return True

    def callback_pos(self, msg):
        lat = msg.lat
        lon = msg.lon
        alt = msg.alt
        with self.mutex_heading:
            hdg = self.uav_heading_rad
        if self.debug:
            rospy.loginfo('Received new pose')
            rospy.loginfo('    Lat: %s', lat)
            rospy.loginfo('    Lon: %s', lon)
            rospy.loginfo('Rel alt: %s', alt)
            rospy.loginfo('Heading: %s', hdg)
        (hemisphere, zone, zlet, easting, northing) = self.uc.geodetic_to_utm(lat, lon)
        uav_pose_rel = {
            'y': easting-self.zeroEasting,
            'x': northing-self.zeroNorthing,
            'z': alt,
            'hdg': hdg
        }
        rospy.logdebug('y rel: %f', uav_pose_rel['y'])
        rospy.logdebug('x rel: %f', uav_pose_rel['x'])
        rospy.logdebug('z rel: %f', uav_pose_rel['z'])
        rospy.logdebug('  hdg: %f', uav_pose_rel['hdg'])

        rospy.loginfo('  hdg: %f', uav_pose_rel['hdg'])
        rospy.loginfo('  hdg: %f', self.uav_heading_deg)
        rospy.loginfo('pitch move x: %f', sin(uav_pose_rel['hdg']) )
        rospy.loginfo('pitch move y: %f', cos(uav_pose_rel['hdg']) )


    def callback_attitude(self, msg):
        with self.mutex_heading:
            self.uav_heading_rad = msg.yaw
            self.uav_heading_deg = msg.yaw*RAD2DEG
        if self.debug:
            rospy.loginfo('Received new Attitude')
            rospy.loginfo('Heading: %s', msg.yaw)
            rospy.loginfo('Heading: %s', msg.yaw*RAD2DEG)

def restrict_uav():
    # Variables
    uav_pose = {
        'x': 0,
        'y': 0,
        'z': 0,
        'yaw':90
    }
    # Try changing rx_input below
    rx_input = {
        'throttle': 50,
        'yaw': 50,
        'pitch': 50,
        'roll': 50,
    }
    # rx_output just defined as default values
    rx_output = {
        'throttle': 50,
        'yaw': 50,
        'pitch': 50,
        'roll': 50,
    }

    print 'UAV pose:', uav_pose
    print 'RX input:', rx_input

    move_x_from_pitch = ( rx_input['pitch']-RX_CENTER_POSITION['pitch'] ) * sin( uav_pose['yaw']*pi / 180 )
    move_y_from_pitch = ( rx_input['pitch']-RX_CENTER_POSITION['pitch'] ) * cos( uav_pose['yaw']*pi / 180 )
    # print 'X component from pitch: %.02f' % move_x_from_pitch
    # print 'Y component from pitch: %.02f' % move_y_from_pitch
    if (DRONEBUR_LIMITS['x_min'] < uav_pose['x']+FACTOR_INPUT2METRIC_XY*move_x_from_pitch and uav_pose['x']+FACTOR_INPUT2METRIC_XY*move_x_from_pitch < DRONEBUR_LIMITS['x_max']) and (DRONEBUR_LIMITS['y_min'] < uav_pose['y']+FACTOR_INPUT2METRIC_XY*move_y_from_pitch and uav_pose['y']+FACTOR_INPUT2METRIC_XY*move_y_from_pitch < DRONEBUR_LIMITS['y_max']):
        rx_output['pitch'] = rx_input['pitch']
    else:
        rx_output['pitch'] = RX_CENTER_POSITION['pitch']

    move_x_from_roll = ( rx_input['roll']-RX_CENTER_POSITION['roll'] ) * sin( uav_pose['yaw']*pi / 180 + pi/2 )
    move_y_from_roll = ( rx_input['roll']-RX_CENTER_POSITION['roll'] ) * cos( uav_pose['yaw']*pi / 180 + pi/2 )
    # print 'X component from roll: %.02f' % move_x_from_roll
    # print 'Y component from roll: %.02f' % move_y_from_roll
    if (DRONEBUR_LIMITS['x_min'] < uav_pose['x']+FACTOR_INPUT2METRIC_XY*move_x_from_roll and uav_pose['x']+FACTOR_INPUT2METRIC_XY*move_x_from_roll < DRONEBUR_LIMITS['x_max']) and (DRONEBUR_LIMITS['y_min'] < uav_pose['y']+FACTOR_INPUT2METRIC_XY*move_y_from_roll and uav_pose['y']+FACTOR_INPUT2METRIC_XY*move_y_from_roll < DRONEBUR_LIMITS['y_max']):
        rx_output['roll'] = rx_input['roll']
    else:
        rx_output['roll'] = RX_CENTER_POSITION['roll']

    move_z_from_throttle = rx_input['throttle']-RX_CENTER_POSITION['throttle']
    if DRONEBUR_LIMITS['z_min'] < uav_pose['z']+FACTOR_INPUT2METRIC_Z*move_z_from_throttle and uav_pose['z']+FACTOR_INPUT2METRIC_Z*move_z_from_throttle < DRONEBUR_LIMITS['z_max']:
        rx_output['throttle'] = rx_input['throttle']
    else:
        rx_output['throttle'] = RX_CENTER_POSITION['throttle']

    rx_output['yaw'] = rx_input['yaw']

    UAV_new_x = uav_pose['x']+FACTOR_INPUT2METRIC_XY*(move_x_from_pitch + move_x_from_roll)
    UAV_new_y = uav_pose['y']+FACTOR_INPUT2METRIC_XY*(move_y_from_pitch + move_y_from_roll)
    UAV_new_z = uav_pose['z']+FACTOR_INPUT2METRIC_Z *move_z_from_throttle
    print 'X component combined: %.02f' % UAV_new_y
    print 'Y component combined: %.02f' % UAV_new_x
    print 'Z component combined: %.02f' % UAV_new_z

    if DRONEBUR_LIMITS['x_min'] < UAV_new_x and UAV_new_x < DRONEBUR_LIMITS['x_max']: print 'X: possible move'
    else: print 'X: not possible'

    if DRONEBUR_LIMITS['y_min'] < UAV_new_x and UAV_new_y <  DRONEBUR_LIMITS['y_max']: print 'Y: possible move'
    else: print 'Y: not possible'

    if DRONEBUR_LIMITS['z_min'] < UAV_new_z and UAV_new_z <  DRONEBUR_LIMITS['z_max']: print 'Z: possible move'
    else: print 'Z: not possible'

    print 'RX output:', rx_output

if __name__ == '__main__':
    try:
        uav_restriction_module = uav_restriction()
    except rospy.ROSInterruptException:
        pass
