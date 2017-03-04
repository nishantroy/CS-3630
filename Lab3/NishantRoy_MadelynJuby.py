# Madelyn Juby, Nishant Roy

# !/usr/bin/env python3

import sys

import cv2
import numpy as np
import find_ball


from cozmoFSM import cozmoFSM


import cozmo
from cozmo.util import degrees

try:
    from PIL import Image, ImageDraw, ImageFont
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

def make_text_image(text_to_draw, x, y, font=None):
    '''Make a PIL.Image with the given text printed on it

    Args:
        text_to_draw (string): the text to draw to the image
        x (int): x pixel location
        y (int): y pixel location
        font (PIL.ImageFont): the font to use

    Returns:
        :class:(`PIL.Image.Image`): a PIL image with the text drawn on it
    '''

    # make a blank image for the text, initialized to opaque black
    text_image = Image.new('RGBA', cozmo.oled_face.dimensions(), (0, 0, 0, 255))

    # get a drawing context
    dc = ImageDraw.Draw(text_image)

    # draw the text
    dc.text((x, y), text_to_draw, fill=(255, 255, 255, 255), font=font)

    return text_image


_clock_font = None
try:
    _clock_font = ImageFont.truetype("arial.ttf", 20)
except IOError:
    try:
        _clock_font = ImageFont.truetype("/Library/Fonts/Arial.ttf", 20)
    except IOError:
        pass


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
        isNotFinished = 1
        state = "initial"
        prevBall = [0, 0, 0]
        robot.set_robot_volume(0.3)
        myCozmo = cozmoFSM("myCozmo")

        while isNotFinished:
            oled_face_data = cozmo.oled_face.convert_image_to_screen_data(
                make_text_image(myCozmo.state, 8, 6, _clock_font))
            robot.display_oled_face_image(oled_face_data, 1000.0, in_parallel=True)
            # get camera image
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

            # convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
            height, width = opencv_image.shape[:2]

            # look for ball
            ball = find_ball.find_ball(opencv_image)

            if ball[0] != 0 or ball[1] != 0 or ball[2] != 0:

                ballDetected = 1
                prevBall = ball
            else:
                ball = None
                ballDetected = 0
            BallAnnotator.ball = ball

            if myCozmo.state == "initial":
                print("Starting state: initial")
                # print('\a')

                if ballDetected == 1:
                    print("Previous state: initial, new state: drive")
                    myCozmo.ballDetected()

                else:
                    print("Previous state: initial, new state: search")
                    myCozmo.ballLost()


                # print('\a')

            elif myCozmo.state == "search":

                # robot.stop_all_motors()

                if ballDetected == 1:
                    robot.stop_all_motors()
                    print("Previous state: search, new state: drive")
                    myCozmo.ballDetected()
                    # print('\a')
                    continue
                else:
                    if prevBall[0] != 0 or prevBall[1] != 0 or prevBall[2] != 0:
                        offset = width / 2 - prevBall[0]
                        if offset < 0:
                            await robot.drive_wheels(35, -35)
                        else:
                            await robot.drive_wheels(-35, 35)
                    else:
                        await robot.drive_wheels(-35, 35)

            elif myCozmo.state == "drive":

                if not ballDetected:
                    print("Previous state: drive, new state: search")
                    myCozmo.ballLost()
                    # print('\a')
                    continue
                if ballDetected:
                    distance = actualBallHeight * focalLength / ball[2]

                    leftConst = 70
                    rightConst = 70

                    offset = width / 2 - ball[0]

                    leftSpeed = leftConst - 0.333 * offset
                    rightSpeed = rightConst + 0.333 * offset

                    if distance < 54 or ball[2] > 98:

                        print("Previous state: drive, new state: hit ball")
                        myCozmo.ballInReach()
                        # print(state)
                        # print('\a')
                    else:
                        await robot.drive_wheels(leftSpeed, rightSpeed)

            elif myCozmo.state == "hitball":
                await robot.set_lift_height(0.5, in_parallel=True).wait_for_completed()
                await robot.set_lift_height(0.1, in_parallel=True).wait_for_completed()
                print("Previous state: hit ball, new state: drive")
                myCozmo.ballDetected()
                # print('\a')

                # isNotFinished = 0

    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)


if __name__ == '__main__':
    cozmo.run_program(run, use_viewer=True, force_viewer_on_top=True)
