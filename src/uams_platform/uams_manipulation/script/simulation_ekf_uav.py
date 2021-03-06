#!/usr/bin/env python3
from __future__ import print_function
from __future__ import division
import rospy
import numpy as np
from scipy.linalg import block_diag
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, Twist
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from mav_msgs.msg import Actuators

"""
NOTE: The simulation IMU does not take the body kinematics into account (control input acceleration will not affect the simulated IMU readings)

    Subscribed to
    ----------
    Topic: /uams/imu
           /uams/ground_truth/pose (will be replaced with gps reading in real quad)
           /uams/ground_truth/odometry 
           /uams/motor_speed 

    Publishes
    ---------
    Topic: /uav_localization/odom

"""

class SimulationEkfStateEstimation(object):
    """ Takes IMU, position data, and motor speed data and generates a state estimate based off an Extended Kalman Filter with 15 states
        x-pos, y-pos, z-pos, x-vel, y-vel, z-vel, x-acc, y-acc, z-acc, roll, pitch, yaw, roll rate, pitch rate, yaw rate"""

    def __init__(self):
        self.ekfPublisher = rospy.Publisher("/uav_localization/odom", Odometry, queue_size=1)
        # initialize initial state covariance matrix
        self.previousPm = np.identity(15) * 0.1

        # initialize the previous X State
        self.previousXm = np.zeros((15, 1))

        # x-pos, y-pos, z-pos, x-vel, y-vel, z-vel, x-acc, y-acc, z-acc, roll, pitch, yaw, roll rate, pitch rate, yaw rate
        self.processVariance = block_diag(0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05,
                                          0.05, 0.05)
        self.measurementVariance = block_diag(0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.05, 0.05, 0.01, 0.001, 0.001, 0.001,
                                              0.01, 0.01, 0.01)
        self.firstMeasurementPassed = False
        self.initTimeDelta = 0.01

        # constants
        self.g = 9.8  # [m/s^2]
        self.m = 3.42  # [kg]
        self.Ixx = 6.08870e-02  # [kg*m^2]
        self.Iyy = 6.87913e-02  # [kg*m^2]
        self.Izz = 1.48916e-01  # [kg*m^2]
        self.L = 0.2895  # [m]
        self.thrustConstant = 1.269e-05  #
        self.momentConstant = 0.016754

        # matrix mapping squared motor angular velocity to force/torque control input
        self.speedAllocationMatrix = np.array([[self.thrustConstant, self.thrustConstant, self.thrustConstant,
                                                self.thrustConstant, self.thrustConstant, self.thrustConstant],
                                               [(0.5) * self.L * self.thrustConstant,
                                                self.L * self.thrustConstant,
                                                (0.5) * self.L * self.thrustConstant,
                                                (-0.5) * self.L * self.thrustConstant,
                                                (-1) * self.L * self.thrustConstant,
                                                (-0.5) * self.L * self.thrustConstant],
                                               [-0.5 * (3 ** 0.5) * self.L * self.thrustConstant, 0,
                                                0.5 * (3 ** 0.5) * self.L * self.thrustConstant,
                                                0.5 * (3 ** 0.5) * self.L * self.thrustConstant,
                                                0, -0.5 * (3 ** 0.5) * self.L * self.thrustConstant],
                                               [self.momentConstant, (-1) * self.momentConstant, self.momentConstant,
                                                (-1) * self.momentConstant, self.momentConstant,
                                                (-1) * self.momentConstant]])
        # self.speedAllocationMatrix = np.array([[self.thrustConstant, self.thrustConstant, self.thrustConstant,
        #                                         self.thrustConstant, self.thrustConstant, self.thrustConstant],
        #                                        [0,
        #                                         self.L * self.thrustConstant,
        #                                         self.L * self.thrustConstant,
        #                                         0,
        #                                         (-1) * self.L * self.thrustConstant,
        #                                         (-1) * self.L * self.thrustConstant],
        #                                        [self.L * self.thrustConstant,
        #                                         0,
        #                                         0,
        #                                         (-1) * self.L * self.thrustConstant,
        #                                         0,
        #                                         0],
        #                                        [(-1) * self.momentConstant, self.momentConstant, (-1) * self.momentConstant,
        #                                         self.momentConstant, (-1) * self.momentConstant,
        #                                         self.momentConstant]])
        self.controlInput = np.zeros((4, 1))

        # logic to reduce position measurement rate to 10 Hz
        self.positionCallbackRateCount = 0
        self.positionCallbackRate = 10  # every 10th measurement
        self.imuNoise = np.array(([0.001, 0.001, 0.001]))
        self.positionStdDev = np.array(([0.01, 0.01, 0.01]))
        self.velStdDev = np.array(([0.01, 0.01, 0.01]))

    def euler_to_quaternion(self, r, p, y):
        """ Function to convert euler angles to quaternion"""
        # assemble direction cosine matrix corresponding to 1-2-3 rotation
        a_DCM_b = np.array(([np.cos(y) * np.cos(p), np.cos(y) * np.sin(p) * np.sin(r) - np.sin(y) * np.cos(r),
                             np.cos(y) * np.sin(p) * np.cos(r) + np.sin(r) * np.sin(y)],
                            [np.sin(y) * np.cos(p), np.sin(y) * np.sin(p) * np.sin(r) + np.cos(r) * np.cos(y),
                             np.sin(y) * np.sin(p) * np.cos(r) - np.cos(y) * np.sin(r)],
                            [(-1) * np.sin(p), np.cos(p) * np.sin(r), np.cos(p) * np.cos(r)]))
        qw = 0.5 * pow(1 + a_DCM_b[0, 0] + a_DCM_b[1, 1] + a_DCM_b[2, 2], 0.5)
        qx = (a_DCM_b[2, 1] - a_DCM_b[1, 2]) / (4 * qw)
        qy = (a_DCM_b[0, 2] - a_DCM_b[2, 0]) / (4 * qw)
        qz = (a_DCM_b[1, 0] - a_DCM_b[0, 1]) / (4 * qw)
        a_q_b = np.array(([qx],
                          [qy],
                          [qz],
                          [qw]))
        return a_q_b

    def imu_callback(self, imuMsg):
        """ Callback for the imu input"""
        # convert quaternion into euler angles
        (imuRoll, imuPitch, imuYaw) = euler_from_quaternion(
            [imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w])
        imuRoll = imuRoll + np.random.normal(0, self.imuNoise[0])
        imuPitch = imuPitch + np.random.normal(0, self.imuNoise[1])
        imuYaw = imuYaw + np.random.normal(0, self.imuNoise[2])
        # convert back to quaternion
        quat = self.euler_to_quaternion(imuRoll, imuPitch, imuYaw)
        imuMsg.orientation.x = quat[0, 0]
        imuMsg.orientation.y = quat[1, 0]
        imuMsg.orientation.z = quat[2, 0]
        imuMsg.orientation.w = quat[3, 0]

        imuEstimate = self.imu_ekf_estimation(imuMsg)

        self.ekfPublisher.publish(imuEstimate)

    def pose_callback(self, poseMsg):
        """ Callback for the pose input"""
        # ekf on every 10th measurement 
        self.positionCallbackRateCount = self.positionCallbackRateCount + 1
        if not (self.positionCallbackRateCount % self.positionCallbackRate):
            poseMsg.position.x = poseMsg.position.x + np.random.normal(0, self.positionStdDev[0])
            poseMsg.position.y = poseMsg.position.y + np.random.normal(0, self.positionStdDev[1])
            poseMsg.position.z = poseMsg.position.z + np.random.normal(0, self.positionStdDev[2])
            # update the position estimate but don't publish
            self.pose_ekf_estimation(poseMsg)

    def vel_callback(self, velMsg):
        """ Callback for the velocity input"""
        # TODO: complete the velocity message feedback
        velLinMsg = Twist()
        velLinMsg.linear.x = velMsg.twist.twist.linear.x + np.random.normal(0, self.velStdDev[0])
        velLinMsg.linear.y = velMsg.twist.twist.linear.y + np.random.normal(0, self.velStdDev[1])
        velLinMsg.linear.z = velMsg.twist.twist.linear.z + np.random.normal(0, self.velStdDev[2])
        # update the velocity estimate but don't publish
        self.vel_ekf_estimation(velLinMsg)

    def motor_speed_callback(self, motorSpeeds):
        """ Callback for the motor speeds"""
        motorSpeedSquaredArray = np.array(([pow(motorSpeeds.angular_velocities[0], 2)],
                                           [pow(motorSpeeds.angular_velocities[1], 2)],
                                           [pow(motorSpeeds.angular_velocities[2], 2)],
                                           [pow(motorSpeeds.angular_velocities[3], 2)],
                                           [pow(motorSpeeds.angular_velocities[4], 2)],
                                           [pow(motorSpeeds.angular_velocities[5], 2)]))
        # map the squared motor speeds to forces and torques
        self.controlInput = np.dot(self.speedAllocationMatrix, motorSpeedSquaredArray)

    def quad_nonlinear_eom(self, state, input, dt):
        """ Function for nonlinear equations of motion of quadcopter """
        # RPY position and rate update
        prevAngPos = np.array(([state[9],
                                state[10],
                                state[11]]))
        prevAngVel = np.array(([state[12],
                                state[13],
                                state[14]]))
        # print(prevAngPos)
        # angAccel = np.array(([(input[1,0] + self.Iyy*prevAngVel[1,0]*prevAngVel[2,0] - self.Izz*prevAngVel[1,0]*prevAngVel[2,0])/self.Ixx],
        #                      [(input[2,0] - self.Ixx*prevAngVel[0,0]*prevAngVel[2,0] + self.Izz*prevAngVel[0,0]*prevAngVel[2,0])/self.Iyy],
        #                      [(input[3,0] + self.Ixx*prevAngVel[0,0]*prevAngVel[1,0] - self.Iyy*prevAngVel[0,0]*prevAngVel[1,0])/self.Izz]))
        angAccel = np.array(([(input[1, 0] + self.Iyy * prevAngVel[1, 0] * prevAngVel[0, 0] - self.Izz * prevAngVel[
            1, 0] * prevAngVel[2, 0]) / self.Ixx],
                             [(input[2, 0] - self.Ixx * prevAngVel[0, 0] * prevAngVel[2, 0] + self.Izz * prevAngVel[
                                 0, 0] * prevAngVel[2, 0]) / self.Iyy],
                             [(self.Ixx * prevAngVel[0, 0] * prevAngVel[1, 0] - self.Iyy * prevAngVel[0, 0] *
                               prevAngVel[1, 0]) / self.Izz]))
        angVel = prevAngVel + angAccel * dt
        angPos = prevAngPos + angVel * dt + 0.5 * angAccel * pow(dt, 2)

        # XYZ position and rate update
        prevLinPos = np.array(([state[0],
                                state[1],
                                state[2]]))
        prevLinVel = np.array(([state[3],
                                state[4],
                                state[5]]))

        gravityComponent = np.array(([[0],
                                      [0],
                                      [-self.g]]))

        # 1-2-3 rotation matrix of inertial in body frame
        rotMatThirdCol = np.array(([np.cos(prevAngPos[2, 0]) * np.sin(prevAngPos[1, 0]) * np.cos(
            prevAngPos[0, 0]) + np.sin(prevAngPos[2, 0]) * np.sin(prevAngPos[0, 0])],
                                   [np.sin(prevAngPos[2, 0]) * np.sin(prevAngPos[1, 0]) * np.cos(
                                       prevAngPos[0, 0]) - np.cos(prevAngPos[2, 0]) * np.sin(prevAngPos[0, 0])],
                                   [np.cos(prevAngPos[1, 0]) * np.cos(prevAngPos[0, 0])]))

        if self.controlInput[0, 0] <= 0.05:
            gravityComponent[2, 0] = 0

        linAccel = gravityComponent + (input[0, 0] / self.m) * rotMatThirdCol
        linVel = prevLinVel + linAccel * dt
        linPos = prevLinPos + linVel * dt + 0.5 * linAccel * pow(dt, 2)
        nonLinState = np.vstack((linPos, linVel, linAccel, angPos, angVel))

        return nonLinState

    def pose_ekf_estimation(self, poseMsg):
        """ Use ekf to create a state estimate when given pose and control input data"""
        # get the current time
        currentTime = rospy.get_time()

        # calculate the time delta b/t previous measurement and current measurement
        if self.firstMeasurementPassed:
            dt = currentTime - self.previousTime
        else:
            dt = self.initTimeDelta
        if dt == 0.0:
            dt = 0.01

        # system measurements, stuff linear acceleration into linear velocity state
        z = np.array(([poseMsg.position.x],
                      [poseMsg.position.y],
                      [poseMsg.position.z]))

        # state update matrix
        A = np.array([[1, 0, 0, dt, 0, 0, 0.5 * pow(dt, 2), 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, dt, 0, 0, 0.5 * pow(dt, 2), 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, dt, 0, 0, 0.5 * pow(dt, 2), 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, self.g, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 1, 0, -self.g, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

        # measurement matrix
        H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

        # measurement noise partial derivative matrix
        M = np.identity(3)
        # process noise partial derivative matrix
        L = np.identity(15)

        # process variance matrix
        V = self.processVariance

        # measurement Variance
        W = self.measurementVariance[0:3, 0:3]

        xm = self.ekf_calc(A, V, L, M, W, H, z, dt)

        # first pass has been completed, time delta can be updated
        self.firstMeasurementPassed = True

        # update the previous time
        self.previousTime = currentTime

        return self.odom_msg_creation(xm)

    def vel_ekf_estimation(self, velLinMsg):
        """ Use ekf to create a state estimate when given linear velocity and control input data"""
        # get the current time
        currentTime = rospy.get_time()

        # calculate the time delta b/t previous measurement and current measurement
        if self.firstMeasurementPassed:
            dt = currentTime - self.previousTime
        else:
            dt = self.initTimeDelta
        if dt == 0.0:
            dt = 0.01

        # system measurements, stuff linear acceleration into linear velocity state
        z = np.array(([velLinMsg.linear.x],
                      [velLinMsg.linear.y],
                      [velLinMsg.linear.z]))

        # state update matrix
        A = np.array([[1, 0, 0, dt, 0, 0, 0.5 * pow(dt, 2), 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, dt, 0, 0, 0.5 * pow(dt, 2), 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, dt, 0, 0, 0.5 * pow(dt, 2), 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, self.g, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 1, 0, -self.g, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

        # measurement matrix
        H = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

        # measurement noise partial derivative matrix
        M = np.identity(3)
        # process noise partial derivative matrix
        L = np.identity(15)

        # process variance matrix
        V = self.processVariance

        # measurement Variance
        W = self.measurementVariance[3:6, 3:6]

        xm = self.ekf_calc(A, V, L, M, W, H, z, dt)

        # first pass has been completed, time delta can be updated
        self.firstMeasurementPassed = True

        # update the previous time
        self.previousTime = currentTime

        return self.odom_msg_creation(xm)

    def imu_ekf_estimation(self, imuMsg):
        """ Use ekf to create a state estimate when given imu and control input data"""
        # convert quat to rpy angles
        (imuRoll, imuPitch, imuYaw) = euler_from_quaternion(
            [imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w])
        # print(imuYaw)
        # get the current time
        currentTime = rospy.get_time()

        # calculate the time delta b/t previous measurement and current measurement
        if self.firstMeasurementPassed:
            dt = currentTime - self.previousTime
        else:
            dt = self.initTimeDelta
        if dt == 0.0:
            dt = 0.01
        # rotate the acceleration measurements to inertial frame 
        prevRoll = self.previousXm[9, 0]
        prevPitch = self.previousXm[10, 0]
        prevYaw = self.previousXm[11, 0]

        # body in inertial frame 1-2-3 rotation
        N_R_b = np.array(([np.cos(prevYaw) * np.cos(prevPitch),
                           np.cos(prevYaw) * np.sin(prevPitch) * np.sin(prevRoll) - np.sin(prevYaw) * np.cos(prevRoll),
                           np.cos(prevYaw) * np.sin(prevPitch) * np.cos(prevRoll) + np.sin(prevRoll) * np.sin(prevYaw)],
                          [np.sin(prevYaw) * np.cos(prevPitch),
                           np.sin(prevYaw) * np.sin(prevPitch) * np.sin(prevRoll) + np.cos(prevRoll) * np.cos(prevYaw),
                           np.sin(prevYaw) * np.sin(prevPitch) * np.cos(prevRoll) - np.cos(prevYaw) * np.sin(prevRoll)],
                          [(-1) * np.sin(prevPitch), np.cos(prevPitch) * np.sin(prevRoll),
                           np.cos(prevPitch) * np.cos(prevRoll)]))

        linAccelBody = np.array(([imuMsg.linear_acceleration.x],
                                 [imuMsg.linear_acceleration.y],
                                 [(-1) * imuMsg.linear_acceleration.z]))
        # rotate linear acceleration in body frame into linear acceleration in inertial frame
        linAccelInertial = np.dot(N_R_b, linAccelBody)
        linAccelInertial[2, 0] = linAccelInertial[2, 0] + self.g

        if self.controlInput[0, 0] <= 0.05:
            linAccelInertial[2, 0] = 0
        # system measurements, stuff linear acceleration into linear velocity state
        z = np.array(([linAccelInertial[0, 0]],
                      [linAccelInertial[1, 0]],
                      [linAccelInertial[2, 0]],
                      [imuRoll],
                      [imuPitch],
                      [imuYaw],
                      [imuMsg.angular_velocity.x],
                      [imuMsg.angular_velocity.y],
                      [imuMsg.angular_velocity.z]))

        # state update matrix
        A = np.array([[1, 0, 0, dt, 0, 0, 0.5 * pow(dt, 2), 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, dt, 0, 0, 0.5 * pow(dt, 2), 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, dt, 0, 0, 0.5 * pow(dt, 2), 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, self.g, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 1, 0, -self.g, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
        # measurement matrix
        H = np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
        # measurement noise partial derivative matrix
        M = np.identity(9)

        # process noise partial derivative matrix
        L = np.identity(15)

        # process variance matrix
        V = self.processVariance

        # measurement Variance
        W = self.measurementVariance[6:15, 6:15]

        # ekf calculations
        xm = self.ekf_calc(A, V, L, M, W, H, z, dt)

        # first pass has been completed, time delta can be updated
        self.firstMeasurementPassed = True

        # update the previous time
        self.previousTime = currentTime

        return self.odom_msg_creation(xm)

    def ekf_calc(self, A, V, L, M, W, H, z, dt):
        """ Function to calculate the EKF output given the input matrices"""
        # prior state 
        xp = self.quad_nonlinear_eom(self.previousXm, self.controlInput, dt)

        # prior covariance
        Pp = np.dot(A, np.dot(self.previousPm, np.transpose(A))) + np.dot(L, np.dot(V, np.transpose(L)))

        # kalman gain
        K = np.dot(Pp, np.dot(np.transpose(H), np.linalg.inv(
            np.dot(H, np.dot(Pp, np.transpose(H))) + np.dot(M, np.dot(W, np.transpose(M))))))
        # state matrix update
        xm = xp + np.dot(K, (z - np.dot(H, xp)))
        self.previousXm = xm

        # posterior state update
        Pm = np.dot(np.identity(15) - np.dot(K, H), Pp)
        self.previousPm = Pm

        return xm

    def odom_msg_creation(self, xm):
        """ Temporary function to create an odom message given state data"""
        createdOdomMsg = Odometry()

        # position
        createdOdomMsg.pose.pose.position.x = xm[0]
        createdOdomMsg.pose.pose.position.y = xm[1]
        createdOdomMsg.pose.pose.position.z = xm[2]

        # linear velocity
        createdOdomMsg.twist.twist.linear.x = xm[3]
        createdOdomMsg.twist.twist.linear.y = xm[4]
        createdOdomMsg.twist.twist.linear.z = xm[5]

        # orientation
        (createdOdomMsg.pose.pose.orientation.x,
         createdOdomMsg.pose.pose.orientation.y,
         createdOdomMsg.pose.pose.orientation.z,
         createdOdomMsg.pose.pose.orientation.w) = quaternion_from_euler(xm[9], xm[10], xm[11])

        # angular velocity
        createdOdomMsg.twist.twist.angular.x = xm[12]
        createdOdomMsg.twist.twist.angular.y = xm[13]
        createdOdomMsg.twist.twist.angular.z = xm[14]

        return createdOdomMsg

    def data_converter(self):
        """ Subscribe to the IMU, pose, and motor speed data"""

        rospy.Subscriber("/uams/imu", Imu, self.imu_callback, queue_size = 1)
        rospy.Subscriber("/uams/ground_truth/pose", Pose, self.pose_callback, queue_size = 1)
        # NOTE: will need to change to a twist message callback when actually doing optical flow, but this will serve as the velocity sensor feedback
        rospy.Subscriber("/uams/ground_truth/odometry", Odometry, self.vel_callback, queue_size = 1)
        rospy.Subscriber("/uams/motor_speed", Actuators, self.motor_speed_callback, queue_size = 1)

        # rospy.Subscriber("/neo11/imu", Imu, self.imu_callback, queue_size=1)
        # rospy.Subscriber("/neo11/ground_truth/pose", Pose, self.pose_callback, queue_size=1)
        # # NOTE: will need to change to a twist message callback when actually doing optical flow, but this will serve as the velocity sensor feedback
        # rospy.Subscriber("/neo11/ground_truth/odometry", Odometry, self.vel_callback, queue_size=1)
        # rospy.Subscriber("/neo11/motor_speed", Actuators, self.motor_speed_callback, queue_size=1)
        rospy.spin()


def main():
    rospy.init_node("uav_ekf_node", anonymous=False)
    ekfEstimator = SimulationEkfStateEstimation()

    try:
        ekfEstimator.data_converter()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
