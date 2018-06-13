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
from utm import utmconv

class ROSnode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		
		#Get parameters
		#self.got_dev = rospy.get_param("~got_device", "/dev/drone_cage_got_in")
		#self.aq_dev = rospy.get_param("~aq_device", "/dev/drone_cage_aq_out")
		
		#Serial communications
		serial_error = False
		try:
			self.serialIn = serial.Serial(port = "/dev/ttyUSB1", baudrate = 57600) #(port = self.got_dev, baudrate = 57600)
		except Exception as e:
			rospy.logerr(rospy.get_name() + ": Unable to open serial device")# %s" % self.got_dev)
			serial_error = True
		
		if serial_error == False:
			try:
				self.serialOut = serial.Serial(port = "/dev/ttyUSB0", baudrate = 57600)	#(port = self.aq_dev, baudrate = 57600)
			except Exception as e:
				rospy.logerr(rospy.get_name() + ": Unable to open serial device")# %s" % self.aq_dev)
				serial_error = True
		if serial_error == False:
			#Position conversion class
			self.uc = utmconv()
			
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
			try:
				if len(data) > 5:
					#print data
					eastingDisplacement = float(data[3]) / 1000
					northingDisplacement = float(data[4]) / 1000
					altitude = int(data[5])
		
					droneEasting = self.zeroEasting + eastingDisplacement
					droneNorthing = self.zeroNorthing + northingDisplacement

					latitude, longitude = self.uc.utm_to_geodetic("N", 32, droneEasting, droneNorthing)
					latitude *= 10000000
					longitude *= 10000000
					latitude = int(latitude)
					longitude = int(longitude)
					#print latitude, longitude, altitude
					posString = "$GGA" + "," + str(latitude) + "," + str(longitude) + "," + str(altitude) + "\n"
					self.serialOut.write(posString)
			except ValueError:
				print "Value error"
		
		
	def updater(self):
		while not rospy.is_shutdown():
			self.update_position()
			self.r.sleep()

if __name__ == '__main__':
	rospy.init_node('position_parsing')
	
	try:
		node_class = ROSnode()
	except rospy.ROSInterruptException:
		pass
