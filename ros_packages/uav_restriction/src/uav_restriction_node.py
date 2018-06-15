#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Descriptors: TL = Tobias Lundby (tobiaslundby@gmail.com), FMA = Frederik Mazur Andersen (mazurandersen@gmail.com)
2018-06-04 TL First version
2018-06-13 FMA change lowest value sending to transmitter is 1. Can not send 0.
"""

import rospy
from math import sin, cos, pi, sqrt
import time
import serial
from multiprocessing import Lock
import struct
from utm import utmconv
import sys
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
DRONECAGE_LIMITS = {
    'x_min': -1,
    'x_max': 1,
    'y_min': -1,
    'y_max': 1,
    'z_min': 0,
    'z_max': 3
}
FACTOR_INPUT2METRIC_XY  = 0.01
FACTOR_INPUT2METRIC_Z   = 0.1
ARDUINO_BAUD_RATE       = 115200 # NO LONGER USED
ARDUINO_PORT            = '/dev/ttyUSB0' # NO LONGER USED
ROS_POSE_TOPIC          = '/mavlink_pos'
ROS_ATTITUDE_TOPIC      = '/mavlink_attitude'
ROS_STATUS_TOPIC        = '/mavlink_status'

DEG2RAD                 = pi/180
RAD2DEG                 = 180/pi

DIFF_DRONECAGE_DEG      = 5 # measured CW from north
DIFF_DRONECAGE_RAD      = DIFF_DRONECAGE_DEG*DEG2RAD

class uav_restriction():
    ROS_NODE_NAME = 'uav_restriction'
    mutex_heading = Lock()
    def __init__(self, serial_device, serial_baudrate, debug = False):
        rospy.init_node(self.ROS_NODE_NAME, log_level=rospy.DEBUG)
        rospy.loginfo('Started: '+rospy.get_caller_id())
        #serial_device = str(serial_device)
        #serial_baudrate = int(float(serial_baudrate))
        serial_device = rospy.get_param('/uav_restriction_node/serial_device')
        serial_baudrate = rospy.get_param('/uav_restriction_node/serial_baudrate')
        rospy.logdebug('Device %s, data type %s' % (serial_device, type(serial_device)))
        rospy.logdebug('Baud rate %s, data type %s' % (serial_baudrate, type(serial_baudrate)))

        self.debug = debug

        rospy.loginfo('Configuring UTM for zone 32')
        self.uc = utmconv()
        self.uc.set_zone_override(32)
        #Center positions
        self.zeroEasting = 590734.045671
        self.zeroNorthing = 6136563.68892

        rospy.loginfo('Configuring arduino serial port')
        self.ser_arduino = serial.Serial()
        self.ser_arduino.baudrate = serial_baudrate #ARDUINO_BAUD_RATE
        self.ser_arduino.port = serial_device #ARDUINO_PORT
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

            if rospy.get_param('/uav_restriction_node/emulate_arduino_send'):
                r = rospy.Rate(0.5) # 10hz
                while not rospy.is_shutdown():
                    rospy.loginfo('Sending')
                    self.send_arduino_message(0,20,30,40,50,60)
                    r.sleep()

            rospy.Subscriber(ROS_POSE_TOPIC, mavlink_lora_pos, self.callback_pos)
            rospy.Subscriber(ROS_ATTITUDE_TOPIC, mavlink_lora_attitude, self.callback_attitude)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        rospy.loginfo('Closing arduino serial port')
        while self.ser_arduino.is_open:
            self.ser_arduino.close()
        rospy.loginfo('Arduino serial port closed')
        rospy.loginfo('Stopped '+rospy.get_caller_id())

    def send_arduino_message(self, thr_high, thr_low, pitch_high, pitch_low, roll_high, roll_low):
        if self.ser_arduino != None and self.ser_arduino.is_open:
            #values = bytearray([b'P', thr_high, thr_low, pitch_high, pitch_low, roll_high, roll_low])
            try:
                #self.ser_arduino.write(values)
                #self.ser_arduino.write('P %d %d %d %d %d %d' % (thr_high, thr_low, pitch_high, pitch_low, roll_high, roll_low))
                self.ser_arduino.write(struct.pack('>6B',thr_high+1,thr_low+1,pitch_high+1,pitch_low+1,roll_high+1,roll_low+1))
            except serial.SerialException as e:
                rospy.logerr('Could not write to arduino')
                return False
            else:
                return True

    def linearScaler(self, pitchForwardDist, pitchBackwardDist, rollRightDist, rollLeftDist, aboveDist, underDist):
        pitchForwardScaling = 100 * pitchForwardDist
        pitchBackwardScaling = 100 * pitchBackwardDist
        rollRightScaling = 100 * rollRightDist
        rollLeftScaling = 100 * rollLeftDist
        aboveScaling = 100 * aboveDist
        underScaling = 100 * underDist
        if pitchForwardScaling > 100: pitchForwardScaling = 100
        if pitchBackwardScaling > 100: pitchBackwardScaling = 100
        if rollRightScaling > 100: rollRightScaling = 100
        if rollLeftScaling > 100: rollLeftScaling = 100
        if aboveScaling > 100: aboveScaling = 100
        if underScaling > 100: underScaling = 100
        return pitchForwardScaling, pitchBackwardScaling, rollRightScaling, rollLeftScaling, aboveScaling, underScaling

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

        #rospy.loginfo('  hdg: %f, %.01f', uav_pose_rel['hdg'], uav_pose_rel['hdg']*RAD2DEG)
        rospy.loginfo('  hdg corrected: %f, %.01f', uav_pose_rel['hdg']-DIFF_DRONECAGE_RAD, (uav_pose_rel['hdg']-DIFF_DRONECAGE_RAD)*RAD2DEG)

        #uav_pose_rel['hdg'] = -DIFF_DRONECAGE_RAD + 0.1*pi
        pitch_x_cmpt = cos(uav_pose_rel['hdg']+DIFF_DRONECAGE_RAD)
        pitch_y_cmpt = sin(uav_pose_rel['hdg']+DIFF_DRONECAGE_RAD)
        #print pitch_x_cmpt
        if pitch_x_cmpt >= 0:
            dist_uav_pitch_f_x_cmpt = abs(DRONECAGE_LIMITS['x_max'] - uav_pose_rel['x']) # no need for abs but for understanding purposes
            dist_uav_pitch_r_x_cmpt = abs(DRONECAGE_LIMITS['x_min'] - uav_pose_rel['x'])
        else:
            dist_uav_pitch_f_x_cmpt = abs(DRONECAGE_LIMITS['x_min'] - uav_pose_rel['x'])
            dist_uav_pitch_r_x_cmpt = abs(DRONECAGE_LIMITS['x_max'] - uav_pose_rel['x'])
        #print 'Forward:',dist_uav_pitch_f_x_cmpt
        #print 'Reverse:',dist_uav_pitch_r_x_cmpt
        dist_uav_pitch_f = sqrt( pow(dist_uav_pitch_f_x_cmpt,2) + pow(pitch_y_cmpt*dist_uav_pitch_f_x_cmpt,2) )
        dist_uav_pitch_r = sqrt( pow(dist_uav_pitch_r_x_cmpt,2) + pow(pitch_y_cmpt*dist_uav_pitch_r_x_cmpt,2) )
        #print 'Forward 2',dist_uav_pitch_f
        #print 'Reverse 2',dist_uav_pitch_r

        #rospy.loginfo('pitch move x: %f', pitch_x_cmpt )
        #rospy.loginfo('pitch move y: %f', pitch_y_cmpt )

        roll_x_cmpt = cos( uav_pose_rel['hdg']+DIFF_DRONECAGE_RAD + pi/2 )
        roll_y_cmpt = sin( uav_pose_rel['hdg']+DIFF_DRONECAGE_RAD + pi/2 )

        if pitch_x_cmpt >= 0:
            dist_uav_roll_f_y_cmpt = abs(DRONECAGE_LIMITS['y_max'] - uav_pose_rel['y']) # no need for abs but for understanding purposes
            dist_uav_roll_r_y_cmpt = abs(DRONECAGE_LIMITS['y_min'] - uav_pose_rel['y'])
        else:
            dist_uav_roll_f_y_cmpt = abs(DRONECAGE_LIMITS['y_min'] - uav_pose_rel['y'])
            dist_uav_roll_r_y_cmpt = abs(DRONECAGE_LIMITS['y_max'] - uav_pose_rel['y'])
        #print 'Forward:',dist_uav_roll_f_y_cmpt
        #print 'Reverse:',dist_uav_roll_r_y_cmpt
        dist_uav_roll_f = sqrt( pow(dist_uav_roll_f_y_cmpt,2) + pow(roll_x_cmpt*dist_uav_roll_f_y_cmpt,2) )
        dist_uav_roll_r = sqrt( pow(dist_uav_roll_r_y_cmpt,2) + pow(roll_x_cmpt*dist_uav_roll_r_y_cmpt,2) )
        #print 'Forward 2:',dist_uav_roll_f
        #print 'Reverse 2:',dist_uav_roll_r

        dist_uav_throttle_f = DRONECAGE_LIMITS['z_max'] - uav_pose_rel['z']
        dist_uav_throttle_r = abs(DRONECAGE_LIMITS['z_min'] - uav_pose_rel['z'])
        #print dist_uav_throttle_f
        #print dist_uav_throttle_r

        (pitch_f, pitch_r, roll_f, roll_r, throttle_f, throttle_r) = self.linearScaler(dist_uav_pitch_f, dist_uav_pitch_r, dist_uav_roll_f, dist_uav_roll_r, dist_uav_throttle_f, dist_uav_throttle_r)
        throttle_f = 100
        throttle_r = 100
        print 'Throttle limit high:', throttle_f
        print ' Throttle limit low:', throttle_r
        print '    Roll limit high:', roll_f
        print '     Roll limit low:', roll_r
        print '   Pitch limit high:', pitch_f
        print '    Pitch limit low:', pitch_r


        if self.send_arduino_message(throttle_f, throttle_r, pitch_f, pitch_r, roll_f, roll_r): rospy.loginfo('Sent to arduino')
        else: rospy.logerr('Not send to arduino')

        #rospy.loginfo('roll move x: %f', roll_x_cmpt )
        #rospy.loginfo('roll move y: %f', roll_y_cmpt )

    def callback_attitude(self, msg):
        with self.mutex_heading:
            self.uav_heading_rad = msg.yaw
            self.uav_heading_deg = msg.yaw*RAD2DEG
        if self.debug:
            rospy.loginfo('Received new Attitude')
            rospy.loginfo('Heading: %s', msg.yaw)
            rospy.loginfo('Heading: %s', msg.yaw*RAD2DEG)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("usage: uav_restriction_node.py port baud")
    else:
        try:
            uav_restriction_module = uav_restriction(sys.argv[1], sys.argv[2])
        except rospy.ROSInterruptException:
            pass
