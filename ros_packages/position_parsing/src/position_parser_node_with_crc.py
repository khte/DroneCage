#!/usr/bin/env python
#/*****************************************************************************
#* Position Parser Node
#* Copyright (c) 2018, Kristian Husum Terkildsen <khte@mmmi.sdu.dk>
#* All rights reserved.
#*
#* Redistribution and use in source and binary forms, with or without
#* modification, are permitted provided that the following conditions are met:
#*    * Redistributions of source code must retain the above copyright
#*      notice, this list of conditions and the following disclaimer.
#*    * Redistributions in binary form must reproduce the above copyright
#*      notice, this list of conditions and the following disclaimer in the
#*      documentation and/or other materials provided with the distribution.
#*    * Neither the name of the copyright holder nor the names of its
#*      contributors may be used to endorse or promote products derived from
#*      this software without specific prior written permission.
#*
#* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#* DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
#* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#******************************************************************************/

import rospy
import serial
import struct
import sys
from utm import utmconv
from crc16 import crc16

class ROSnode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		
		#Get parameters
		#self.got_dev = rospy.get_param("~got_device", "/dev/drone_cage_got_in")
		#self.aq_dev = rospy.get_param("~aq_device", "/dev/drone_cage_aq_out")
		
		#Serial communications
		serial_error = False
		try:
			self.serialIn = serial.Serial(port = "/dev/ttyUSB0", baudrate = 57600) #(port = self.got_dev, baudrate = 57600)
		except Exception as e:
			rospy.logerr(rospy.get_name() + ": Unable to open serial device")# %s" % self.got_dev)
			serial_error = True
		
		if serial_error == False:
			try:
				self.serialOut = serial.Serial(port = "/dev/ttyUSB1", baudrate = 57600)	#(port = self.aq_dev, baudrate = 57600)
			except Exception as e:
				rospy.logerr(rospy.get_name() + ": Unable to open serial device")# %s" % self.aq_dev)
				serial_error = True
		if serial_error == False:
			#Position conversion class
			self.uc = utmconv()
			
			#Crc calculation class
			self.crc = crc16()
			
			#Center positions
			self.zeroEasting = 590734.045671
			self.zeroNorthing = 6136563.68892
			
			#Update frequency
			self.rate = 200 #5Hz
			
			self.r = rospy.Rate(self.rate)
			self.updater()
	
	def update_position(self):
		try:
			readLine = self.serialIn.readline()
		except:
			readLine = ""
		
		
		
		if "GTPOS" in readLine:
			data = readLine.split(",")
			#print data
			try:
				if len(data) > 5:
					#print data
					eastingDisplacement = float(data[3]) / 1000
					northingDisplacement = float(data[4]) / 1000
					altitude = int(data[5])

					droneEasting = self.zeroEasting + eastingDisplacement
					droneNorthing = self.zeroNorthing + northingDisplacement

					latitude, longitude = self.uc.utm_to_geodetic("N", 32, droneEasting, droneNorthing)
		
					#latitude = 52.1236753
					#longitude = 10.12352352
					#altitude = 1000
		
					latitude *= 10000000
					longitude *= 10000000
					latitude = long(latitude)
					longitude = long(longitude)
		
					#print latitude, longitude, altitude
					#posString = "$GGA" + "," + str(latitude) + "," + str(longitude) + "," + str(altitude)
					#byteVersion = struct.pack('clll', "p", latitude, longitude, altitude)
					#byteVersion = struct.pack('ciii', 'p', latitude, longitude, altitude)
					tx_buf = []
					tx_buf.append (ord('p'))
		
					#tx_buf = tx_buf + [ord(c) for c in struct.pack("!I", latitude)]
					#tx_buf = tx_buf + [ord(c) for c in struct.pack("!I", longitude)]
					#tx_buf = tx_buf + [ord(c) for c in struct.pack("!I", altitude)]

					longval = struct.unpack ("4B", struct.pack ("i", latitude))
					for i in range(4):
						tx_buf.append (longval[i])
					longval = struct.unpack ("4B", struct.pack ("i", longitude))
					for i in range(4):
						tx_buf.append (longval[i])
					longval = struct.unpack ("4B", struct.pack ("i", altitude))
					for i in range(4):
						tx_buf.append (longval[i])
				
					#tmp_val = 1234
					#byteVersion = struct.pack('i', tmp_val)
					#msg = byteVersion
					crcVal = self.crc.calc(tx_buf)
		
					tx_buf.append (crcVal >> 8)
					tx_buf.append (crcVal & 0x00ff)
		
					#print tx_buf
		
					crcOut = chr(crcVal >> 8) + chr(crcVal & 0x00ff)
					#print type(crcOut)
		
					'''
					msgWithCrc = msg + crcOut
		
					#print msg
					tmp_msg = ""
					for element in msg:
						tmp_msg += hex(ord(element))
						tmp_msg += " "
						#print "%s" % hex(ord(element))
					print tmp_msg
					print msgWithCrc
					tmp_msg = ""
					for element in msgWithCrc:
						tmp_msg += hex(ord(element))
						tmp_msg += " "
						#print "%s" % hex(ord(element))
					print tmp_msg
					'''
		
					#msgWithCrc = struct.pack('clllcc', "p", latitude, longitude, altitude, crcOut)
					#print len(msgWithCrc)
					print tx_buf
					self.serialOut.write(tx_buf)
					
			except ValueError:
				print "Value error"
		
		
	def updater(self):
		while not rospy.is_shutdown():
			self.update_position()
			#sys.exit(0)
			self.r.sleep()

if __name__ == '__main__':
	rospy.init_node('position_parsing')
	
	try:
		node_class = ROSnode()
	except rospy.ROSInterruptException:
		pass
