#!/usr/bin/env python3

''' Get a raw frame from camera and display in OpenCV
By press space, save the image from 001.bmp to ...
'''

import cv2
import cozmo
import numpy as np
from numpy.linalg import inv
import threading

from ar_markers.hamming.detect import detect_markers

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *
from cozmo.util import *

# camera params
camK = np.matrix([[295, 0, 160], [0, 295, 120], [0, 0, 1]], dtype='float32')

# marker size in inches
marker_size = 3.5

# tmp cache
last_pose = cozmo.util.Pose(0, 0, 0, angle_z=cozmo.util.Angle(degrees=0))
flag_odom_init = False

# goal location for the robot to drive to, (x, y, theta)
goal = (6, 10, 0)

# map
Map_filename = "map_arena.json"
grid = CozGrid(Map_filename)
gui = GUIWindow(grid)


async def image_processing(robot):
    global camK, marker_size

    event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # convert camera image to opencv format
    opencv_image = np.asarray(event.image)

    # detect markers
    markers = detect_markers(opencv_image, marker_size, camK)

    # show markers
    for marker in markers:
        marker.highlite_marker(opencv_image, draw_frame=True, camK=camK)
        # print("ID =", marker.id);
        # print(marker.contours);
    cv2.imshow("Markers", opencv_image)

    return markers


# calculate marker pose
def cvt_2Dmarker_measurements(ar_markers):
    marker2d_list = []

    for m in ar_markers:
        R_1_2, J = cv2.Rodrigues(m.rvec)
        R_1_1p = np.matrix([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
        R_2_2p = np.matrix([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
        R_2p_1p = np.matmul(np.matmul(inv(R_2_2p), inv(R_1_2)), R_1_1p)
        # print('\n', R_2p_1p)
        yaw = -math.atan2(R_2p_1p[2, 0], R_2p_1p[0, 0])

        x, y = m.tvec[2][0] + 0.5, -m.tvec[0][0]
        print('x =', x, 'y =', y, 'theta =', yaw)

        # remove any duplate markers
        dup_thresh = 2.0
        find_dup = False
        for m2d in marker2d_list:
            if grid_distance(m2d[0], m2d[1], x, y) < dup_thresh:
                find_dup = True
                break
        if not find_dup:
            marker2d_list.append((x, y, math.degrees(yaw)))

    return marker2d_list


# compute robot odometry based on past and current pose
def compute_odometry(curr_pose, cvt_inch=True):
    global last_pose, flag_odom_init
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
                             last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
                             curr_pose.rotation.angle_z.degrees

    dx, dy = rotate_point(curr_x - last_x, curr_y - last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / 25.6, dy / 25.6

    return dx, dy, diff_heading_deg(curr_h, last_h)


# particle filter functionality
class ParticleFilter:
    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):
        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return m_x, m_y, m_h, m_confident


async def run(robot: cozmo.robot.Robot):
    global flag_odom_init, last_pose
    global grid, gui

    # start streaming
    robot.camera.image_stream_enabled = True

    # start particle filter
    pf = ParticleFilter(grid)

    while True:
        currPose = robot.pose
        poseDiff = compute_odometry(currPose)

        markers = await image_processing(robot)
        markers = cvt_2Dmarker_measurements(markers)

        meanEstimate = pf.update(poseDiff, markers)

        gui.show_particles(pf.particles)
        gui.show_mean(meanEstimate[0], meanEstimate[1], meanEstimate[2], meanEstimate[3])
        gui.updated.set()

        if meanEstimate[3]:
            # PF thinks converged
            # Go from where you think you are to goal
            print("CONVERGED")
            estimatedX = meanEstimate[0]
            estimatedY = meanEstimate[1]
            estimatedTh = meanEstimate[2]

            face90 = -estimatedTh + 90
            print(face90)
            robot.stop_all_motors()
            await robot.turn_in_place(degrees(face90), in_parallel=False, num_retries=0).wait_for_completed()


            turnAngle = math.atan2(math.fabs(estimatedY - goal[1]), math.fabs(estimatedX - goal[0]))

            currentToGoalX = goal[0] - estimatedX
            currentToGoalY = goal[1] - estimatedY
            turn = 0
            faceGoal = 0

            if currentToGoalX > 0 and currentToGoalY > 0:
                turn = 0 - turnAngle
                faceGoal = -((math.pi/2) - turnAngle)
            elif currentToGoalX > 0 >= currentToGoalY:
                turn = -(math.pi - turnAngle)
                faceGoal = ((math.pi / 2) - turnAngle)
            elif currentToGoalX <= 0 < currentToGoalY:
                turn = -(turnAngle + math.pi / 2)
                faceGoal = -(turnAngle + (math.pi/2))
            elif currentToGoalX <= 0 and currentToGoalY <= 0:
                turn = math.pi - turnAngle
                faceGoal = turnAngle + math.pi/2

            await robot.turn_in_place(cozmo.util.Angle(turn), in_parallel=False).wait_for_completed()
            robot.stop_all_motors()

            await robot.drive_straight(cozmo.util.Distance
                                       (grid_distance(currentToGoalX, goal[0], currentToGoalY, goal[1])),
                                       cozmo.util.Speed(25)).wait_for_completed()
            await robot.turn_in_place(cozmo.util.Angle(faceGoal)).wait_for_completed()

            await robot.say_text("Bitches I have arrive").wait_for_completed()

        else:
            # look around, turn in place and update filter
            await robot.drive_wheels(10, -10, duration=0)


class CozmoThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on his charger
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':
    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # init
    grid = CozGrid(Map_filename)
    gui = GUIWindow(grid)
    gui.start()
