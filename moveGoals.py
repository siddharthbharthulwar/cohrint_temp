#!/usr/bin/env python

import setup_path 
import airsim
import rospy
from geometry_msgs.msg import PoseStamped

import sys
import time

from harps_interface.msg import *
from std_msgs.msg import Int16
from math import pi, hypot, sin, cos, atan2, degrees, radians

import matplotlib.pyplot as plt 


realPathX = []
realPathY = []
drone_x = 0
drone_y = 0 

z = -30
velocity = 25 #Max speed without vertical jerks



#*****************arc functions ******************

def norm_angle(a):
    # Normalize the angle to be between -pi and pi
    return (a+pi)%(2*pi) - pi

def computeArcs(pos1, pos2, theta_0, s, t_0):
    theta_0 = radians(theta_0)
    x_0 = pos1[0]
    y_0 = pos1[1]
    x_1 = pos2[0]
    y_1 = pos2[1]
    r_G = hypot(x_1 - x_0, y_1 - y_0)  
    phi_G = atan2(y_1 - y_0, x_1 - x_0)
    phi = 2*norm_angle(phi_G - theta_0) 
    r_C = r_G/(2*sin(phi_G - theta_0))  
    L = r_C*phi  
    if phi > pi:
        phi -= 2*pi
        L = -r_C*phi
    elif phi < -pi:
        phi += 2*pi
        L = -r_C*phi
    t_1 = L/s + t_0                        # time at which the robot finishes the arc
    omega = phi/(t_1 - t_0)                # angular velocity           
    x_C = x_0 - r_C*sin(theta_0)           # center of rotation
    y_C = y_0 + r_C*cos(theta_0)
    ret = {
        "x_C": x_C,
        "y_C": y_C,
        "r_C": r_C,
        "t_0": t_0,
        "t_1": t_1,
        "theta_0": theta_0,
        "omega": omega

    }
    return ret
def position(t, ret):
    x_C = ret["x_C"]
    y_C = ret["y_C"]
    r_C = ret["r_C"]
    t_0 = ret["t_0"]
    theta_0 = ret["theta_0"]
    omega = ret["omega"]
    x = x_C + r_C*sin(omega*(t - t_0) + theta_0)
    y = y_C - r_C*cos(omega*(t - t_0) + theta_0)
    return x, y
def orientation(t, ret):
    x_C = ret["x_C"]
    y_C = ret["y_C"]
    r_C = ret["r_C"]
    t_0 = ret["t_0"]
    theta_0 = ret["theta_0"]
    omega = ret["omega"]
    return omega * (t - t_0) + theta_0

def makelists(pos0, pos1, t_0, theta_0, s):
    ret = computeArcs(pos0, pos1, theta_0, s, t_0)
    positions = []
    headings = []
    for t in range(t_0, int(ret["t_1"]) + 1):
        positions.append(position(t, ret))
        headings.append((orientation(t, ret)))
    return positions, headings


#*************************************************



#************************************************* TEMP UTIL METHODS

def vecToList(vector):
    tup = (vector.x_val, vector.y_val, vector.z_val)
    return tup

def vectList(vecList):
    l = []
    for i in range(len(vecList)):
        l.append(vecToList(vecList[i]))

#*************************************************

def stop(msg):
    client.moveByVelocityAsync(0, 0, 0, 10, vehicle_name="Drone1")

def state_callback(data):
    drone_x = data.pose.position.x
    drone_y = data.pose.position.y

def movetoGoal(msg):
    global realPathX, realPathY
    print("Callback")
    state_sub = rospy.Subscriber("/Drone1/pose", PoseStamped, state_callback)
    print(drone_x, drone_y)
    path = []
    plotx = []
    ploty = []
    arc = True

    pltDroneX = []
    pltDroneY = []

    if (arc):
        #positions = makelists((drone_x, drone_y), (msg.x[0], msg.y[0]), 0, 15, 10)[0]
        positions = [(0, 0), (100, 0), (100, 150)]
        positions = [(0, 0), (25, 0), (50, 25), (75, 75), (50, 100), (25, 75), (0, 0)]
        print(positions)
        for i in range(0, len(msg.x)): #assumes length of message to be 1
            print(msg.x[i])
            print(msg.y[i])
            print(len(positions), " positions added to path")
            for k in range(0, len(positions)):
                #print(k)
                #print(positions[k][0], positions[k][1])
                #print(type(positions[k][0]))
                plotx.append(positions[k][0])
                ploty.append(positions[k][1])
               # path.insert(0, airsim.Vector3r(positions[k][0], positions[k][0], z))
                path.append(airsim.Vector3r(positions[k][0], positions[k][1], z))
            #print(vectList(path))
            plt.plot(plotx, ploty)
            plt.plot(realPathX, realPathY)
            plt.show()
        for i in range(0, len(path)):
            positionVector = client.getPosition()
            print(positionVector)
            path2 = [path[i]]
            print(path2)
            flag = False
            client.moveOnPathAsync(path2, velocity, 2000, airsim.DrivetrainType.ForwardOnly, 
                        airsim.YawMode(False,0), velocity + (velocity/2), 1, vehicle_name="Drone1")
            time.sleep(15)
            '''
            while not flag:
                currentPosition = client.getPosition()

                pass
            ''' 
            pltDroneX.append(positionVector.x_val)
            pltDroneY.append(positionVector.y_val)
            print(pltDroneY)
            
        '''
            currently the drone doesn't like so many points 
        '''
        plt.plot(plotx,ploty)
        plt.plot(pltDroneX, pltDroneY)
        plt.show()

    else:

        for i in range(0, len(msg.x)):
            print(msg.x[i])
            print(msg.y[i])
            path.append(airsim.Vector3r(msg.x[i], msg.y[i], z))

        client.moveOnPathAsync(path, velocity, 2000, airsim.DrivetrainType.ForwardOnly, 
                    airsim.YawMode(False,0), velocity + (velocity/2), 1, vehicle_name="Drone1")

    # x, y = msg.x, msg.y

    # print("Moving to: ", x,y)

    # path = []

    # path.append(airsim.Vector3r(x, y, z))

    # client.moveToPositionAsync(x, y, z, velocity, 2000,airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), velocity + (velocity/2), 1, vehicle_name="Drone1")




if __name__ == "__main__":
    
    # velocity = 15
    # z = -30

    rospy.Subscriber("Drone1/Goal", path, movetoGoal)
    rospy.Subscriber("Drone1/Stop", Int16, stop)
    rospy.init_node("move_goal")
    rate = rospy.Rate(10)

    client = airsim.MultirotorClient()
    # client = airsim.VehicleClient()
    client.confirmConnection()
    client.enableApiControl(True, vehicle_name="Drone1")
    client.armDisarm(True)

    landed = client.getMultirotorState(vehicle_name="Drone1").landed_state
    if landed == airsim.LandedState.Landed:
        print("taking off...")
        client.takeoffAsync(vehicle_name="Drone1").join()

    landed = client.getMultirotorState(vehicle_name="Drone1").landed_state
    if landed == airsim.LandedState.Landed:
        print("takeoff failed - check Unreal message log for details")

    print("climbing to altitude: 30")
    client.moveToPositionAsync(0, 0, z, velocity, vehicle_name="Drone1").join()
    print("Ready for Goals")



    # # start = client.getMultirotorState(vehicle_name="Drone1")
    # # print(start)

    # print("Going to (10, 0)")
    # # client.moveToPositionAsync(100, 0, z, velocity, vehicle_name="Drone1").join()

    state_sub = rospy.Subscriber("/Drone1/pose", PoseStamped, state_callback)


    while not rospy.is_shutdown():
        print(drone_x, drone_y)

        realPathX.append(drone_x)
        realPathY.append(drone_y)
        
        rospy.spin()
        # x = float(input("x: "))
        # y = float(input("y: "))

        # print("Going to (", x,",", y,")")
        # client.moveToPositionAsync(x, y, z, velocity, 2000,airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), velocity + (velocity/2), 1, vehicle_name="Drone1")

    # print("Going to (0, 0)")
    # client.moveToPositionAsync(100, 0, z, velocity,  2000,airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), velocity + (velocity/2), 1,  vehicle_name="Drone1").join()
    '''
    path = []

    path.append(airsim.Vector3r(100, 0, z))
    path.append(airsim.Vector3r(0, 0, z))
    '''



    # client.moveOnPathAsync(path, velocity, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), vehicle_name="Drone1").join()

    # client.moveOnPathAsync(path, velocity, 2000, airsim.DrivetrainType.ForwardOnly, 
    #             airsim.YawMode(False,0), velocity + (velocity/2), 1, vehicle_name="Drone1").join()
