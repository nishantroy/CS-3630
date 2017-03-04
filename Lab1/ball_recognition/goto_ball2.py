# Madelyn Juby, Nishant Roy

# !/usr/bin/env python3

import asyncio
import sys

import cv2
import numpy as np
import find_ball

import cozmo
from cozmo.util import degrees, distance_mm

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')


# Define a decorator as a subclass of Annotator; displays battery voltage
class BatteryAnnotator(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        batt = self.world.robot.battery_voltage
        text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
        text.render(d, bounds)


# Define a decorator as a subclass of Annotator; displays the ball
class BallAnnotator(cozmo.annotate.Annotator):
    ball = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BallAnnotator.ball is not None:
            # double size of bounding box to match size of rendered image
            BallAnnotator.ball = np.multiply(BallAnnotator.ball, 2)

            # define and display bounding box with params:
            # msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BallAnnotator.ball[0] - BallAnnotator.ball[2],
                                      BallAnnotator.ball[1] - BallAnnotator.ball[2],
                                      BallAnnotator.ball[2] * 2, BallAnnotator.ball[2] * 2)
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            BallAnnotator.ball = None


async def cozmo_program(robot: cozmo.robot.Robot):
    await robot.say_text("yo what the heck").wait_for_completed()


def isCentered(ball, width):
    xCenter = width / 2
    offset = ball[2] * 1.2 + 5

    is_x_centered = (ball[0] >= xCenter - offset) and (ball[0] <= xCenter + offset)


    return is_x_centered


async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    # add annotators for battery level and ball bounding box
    robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
    robot.world.image_annotator.add_annotator('ball', BallAnnotator)

    try:
        await robot.set_lift_height(0.0, in_parallel=True).wait_for_completed()
        await robot.set_head_angle(degrees(-17), in_parallel=True).wait_for_completed()

        focalLength = 132.7
        actualBallHeight = 40
        ballDetected = 0
        ballCentered = 0
        isNotFinished = 1
        state = "initial"
        prevBall = [0, 0, 0]
        distanceCheck = 3
        prevDistance = 0
        while isNotFinished:
            print(state)
            # get camera image
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

            # convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
            height, width = opencv_image.shape[:2]

            # look for ball
            ball = find_ball.find_ball(opencv_image)


            if ball[0] != 0 or ball[1] != 0 or ball[2] != 0:

                ballDetected = 1
                ballCentered = isCentered(ball, width)
                prevBall = ball
            else:
                ball = prevBall
                # ballDetected = 0
                # ballCentered = 0
            BallAnnotator.ball = ball

            if state == "initial":
                if ballDetected == 1:
                    await robot.say_text("I found the ball. ", in_parallel=True).wait_for_completed()
                    if ballCentered:
                        state = "drive"
                    else:
                        state = "center"
                else:
                    state = "search"

            elif state == "search":
                robot.stop_all_motors()

                if ballDetected == 1:
                    await robot.say_text("I found the ball. ", in_parallel=True).wait_for_completed()
                    if ballCentered:
                        state = "drive"
                    else:
                        state = "center"
                else:
                    await robot.turn_in_place(degrees(-35), in_parallel=False, num_retries=0).wait_for_completed()
            elif state == "center":
                robot.stop_all_motors()
                xCenter = width / 2
                ballX = ball[0]

                if ballDetected == 0:
                    state = "search"
                else:
                    if ballCentered:
                        state = "drive"
                    else:
                        dist = xCenter - ballX
                        angle = (67.0/320.0 * dist) - 2
                        if ballX < xCenter:
                            await robot.turn_in_place(degrees(angle), in_parallel=False, num_retries=0).wait_for_completed()
                        elif ballX > xCenter:
                            await robot.turn_in_place(degrees(angle), in_parallel=False,
                                                      num_retries=0).wait_for_completed()

            elif state == "drive":
                if ballDetected:
                    distance = actualBallHeight * focalLength / ball[2]
                    if abs(prevDistance - distance) < 2 and distance < 90:
                        print("didn't move")
                        if distanceCheck == 0:
                            state = "hit ball"
                        else:
                            distanceCheck -= 1
                    else:
                        distanceCheck = 3

                    prevDistance = distance

                    print("distance: ", distance, "radius: ", ball[2])

                    if distance < 56 or ball[2] > 94:
                        state = "hit ball"
                    else:
                        if ballDetected == 0:
                            robot.stop_all_motors()
                            state = "search"
                        else:
                            if ballCentered == 0:
                                robot.stop_all_motors()
                                state = "center"
                            else:
                                await robot.drive_wheels(85, 85)
                else:
                    state = "center"

            elif state == "hit ball":
                robot.stop_all_motors()
                await robot.say_text("I hit the ball. ", in_parallel=True).wait_for_completed()
                await robot.set_lift_height(0.95, in_parallel=True).wait_for_completed()

                isNotFinished = 0


    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)


if __name__ == '__main__':
    cozmo.run_program(run, use_viewer=True, force_viewer_on_top=True)
