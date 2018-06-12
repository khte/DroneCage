#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Descriptors: TL = Tobias Lundby (tobiaslundby@gmail.com), FMA = Frederik Mazur Andersen (mazurandersen@gmail.com)
2018-06-12 TL Copy and clean up from master thesis
2018-06-12 FMA redid into rosnode with extended functionality
"""

"""
Description:
Path UAV animator
License: BSD 3-Clause
"""

""" Import libraries """
from math import floor
from copy import copy, deepcopy
import time # used for sleeping the main loop
from figures import *
from path_animator.srv import SetFigure
import rospy

class ROSnode():
    def __init__(self):
        rospy.loginfo(rospy.get_name() + ": Start")

        # variables
        self.UAV_message_rate = 0.1
        self.waypoint_list = []
        self.runningPath = []
        self.current_pos_idx = 0
        self.goNextPos = False

        # Send desired position 2 times every second. Might send current pos, if current hasn't been reached
        self.desiredTransmitHz = rospy.Rate(2) #2 hz

        # rospy subs and pubs
        # sub_position = rospy.Subscriber("/mavlink_pos/", , self.checkGoNext)
        rospy.Service("path_animator/set_figure", SetFigure, self.newFigure)

        # make path interpolation
        self.interpolatePath(FIGURE_SQUARE)

        # scale and offset if wanted
        self.scaleAxis(x_axis=1, y_axis=1, z_axis=1)
        self.offsetAxis(x_axis=0, y_axis=0, z_axis=0)

        # print list of all positions
        self.saveWaypointsToCSV(waypoints=self.waypoint_list, filename='eights')

        # run pattern, will repeat running pattern aslong as program is running
        self.runPattern()

    # Functions
    def newFigure(self, msg):
        # send drone to mid position
        #TODO

        # load new figure and interpolate
        figure = []
        if msg.figure.lower() == "eight":
            figure = FIGURE_EIGHT
        elif msg.figure.lower() == 'square_vert':
            figure = FIGURE_SQUARE_VERT
        elif msg.figure.lower() == 'square_hori':
            figure = FIGURE_SQUARE_HORI
        else:
            return False, "Couldn't interpert input figure"

        # interpolate
        self.interpolatePath(figure)
        # Scale and offset
        self.scaleAxis(x_axis=msg.scaleX, y_axis=msg.scaleY, z_axis=msg.scaleZ)
        self.offsetAxis(x_axis=msg.offsetX, y_axis=msg.offsetY, z_axis=msg.offsetZ)

        # sleep for 2sec before starting new pattern
        rospy.sleep(2)

        # reset index and load new path over
        self.current_pos_idx = 0
        self.runningPath = deepcopy(self.waypoint_list)

        return True, "New figure set"

    def checkGoNext(self, msg):
        # check if within acceptable margin of desired position TODO
        i = 0
        # if true, go next position

    def scaleAxis(self, x_axis = 1.0, y_axis=1.0, z_axis=1.0):
        for p in self.waypoint_list:
            p['x'] = p['x'] * x_axis
            p['y'] = p['y'] * y_axis
            p['z'] = p['z_rel'] * z_axis


    def offsetAxis(self, x_axis = 0.0, y_axis=0.0, z_axis=0.0):
        for p in self.waypoint_list:
            p['x'] = p['x'] + x_axis
            p['y'] = p['y'] + y_axis
            p['z'] = p['z_rel'] + z_axis

    def saveWaypointsToCSV(self, waypoints, filename):
        # print waypoint_list
        import csv
        with open(filename + '.csv', 'w') as csvfile:
            writer = csv.writer(csvfile, delimiter=',')
            for p in waypoints:
                writer.writerow([p['x'], p['y'], p['z_rel']])


    def interpolatePath(self, path):
        # clear waypoint list, before populating it
        self.waypoint_list = []
        
        if len(path) >= 2: # Check if path with atleast 2 pos
            # Init simulation variables
            uav = {'y': path[0]['y'], 'x': path[0]['x'], 'z_rel': path[0]['z_rel']}
            time_ctr = 0.0
            waypoint_ctr = 0
            waypoint_last = len(path)-1
            i = 0
            while i < waypoint_last:
                print '\nCurrent waypoints %d and %d' % (i, i+1)
                # Calculate the direction vector scaled based on time and the step size
                travel_dir = [
                    (path[i+1]['y']    -path[i]['y'])     / ( (path[i+1]['time_rel']-path[i]['time_rel']) / self.UAV_message_rate ),
                    (path[i+1]['x']    -path[i]['x'])     / ( (path[i+1]['time_rel']-path[i]['time_rel']) / self.UAV_message_rate ),
                    (path[i+1]['z_rel']-path[i]['z_rel']) / ( (path[i+1]['time_rel']-path[i]['time_rel']) / self.UAV_message_rate )
                    ]
                steps_on_current_line = int(floor( ( path[i+1]['time_rel']-path[i]['time_rel'] )/self.UAV_message_rate ))
                # print steps_on_current_line
                for k in range(steps_on_current_line+1):
                    # print 'UAV:'
                    # print uav

                    print 'Time:', time_ctr
                    if k != steps_on_current_line:
                        # Extend the UAV in the direction of the path corresponding to a single time step
                        uav['y'] += travel_dir[0]
                        uav['x'] += travel_dir[1]
                        uav['z_rel'] += travel_dir[2]
                    else:
                        last_time_step = ((path[i+1]['time_rel']-path[i]['time_rel']) / self.UAV_message_rate) - steps_on_current_line
                        if last_time_step != 0.0:
                            uav['y'] += travel_dir[0]*last_time_step
                            uav['x'] += travel_dir[1]*last_time_step
                            uav['z_rel'] += travel_dir[2]*last_time_step
                    if k != steps_on_current_line: # When there is something rest in the path not corresponding to the exact time steps
                        # time.sleep(UAV_message_rate)
                        time_ctr += self.UAV_message_rate
                    else:
                        # time.sleep(last_time_step)
                        time_ctr += last_time_step

                    # save in waypoint list
                    self.waypoint_list.append(deepcopy(uav))

                i += 1 # increment the loop counter

    def sendDesiredPosition(self, position):
        # ... do something  TODO
        i = 0

    def runPattern(self):
        # repeats current flight pattern over and over with correct timing
        self.runningPath = deepcopy(self.waypoint_list)
        
        # Init simulation variables
        self.current_pos_idx = 0

        # Send drone to midpoint before starting and sleep to let it go there
        #TODO
        rospy.sleep(2)

        while not rospy.is_shutdown():
            # output to drone with function....
            current_pos = self.runningPath[self.current_pos_idx]
            self.sendDesiredPosition(current_pos)

            # sleep until at position and timestep is valid
            if self.goNextPos:
                self.current_pos_idx += 1

                if (self.current_pos_idx >= len(self.waypoint_list)):
                    self.current_pos_idx = 0

                self.goNextPos = False
            
            # sleep until next transmit
            self.desiredTransmitHz.sleep()


if __name__ == '__main__':
	rospy.init_node('path_animator')
	
	try:
		node_class = ROSnode()
	except rospy.ROSInterruptException:
		pass