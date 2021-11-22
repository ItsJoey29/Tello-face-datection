from typing import DefaultDict
from djitellopy import Tello
import cv2
import numpy as np
import time
import datetime
import os
import argparse
#import Key_Control_Module as kp

# Argparse setup
parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter, add_help=False)
parser.add_argument('-h', '--help', action='help',
                    default=argparse.SUPPRESS, help='** = required')
parser.add_argument('-d', '--distance', type=int, default=3,
                    help='use -d to change the distance of the drone. Range 0-6')
parser.add_argument('-sx', '--safety_x', type=int, default=100,
                    help='use -sx to change the safety bound on the x axis. Range 0-480.')
parser.add_argument('-sy', '--safety_y', type=int, default=55,
                    help='use -sy to change the safety bound on the y-axis. Range 0-360')
parser.add_argument('-os', '--override_speed', type=int, default=1,
                    help='use -os to change override speed. Range 0-3.')
parser.add_argument('-ss', '--save_session', action='store_true',
                    help='add the -ss flag to save your session as an image sequence in the sessions folder.')
parser.add_argument('-D', '--debug', action='store_true',
                    help='add the -D flag to enable debug mode. Everything works the same, but no commands will be sent to the drone.')

args = parser.parse_args()


# Speed of drone
S = 20
S2 = 5
UDOffset = 150

# Bound box sizes
faceSizes = [1026, 684, 456, 304, 202, 136, 90]

# Speed mode values
acc = [500, 250, 250, 150, 110, 70, 50]

# CV2 display setup
FPS = 25
dimentions = (960, 720)

# Make safe directory
if args.save_session:
    ddir = "Sessions"

    if not os.path.isdir(ddir):
        os.mkdir(ddir)

    ddir = 'Sessions/Session {}'.format(str(datetime.datetime.now())
                                        ).replace(':', '-').replace('.', '_')

# Cascade setup
face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')


class AI(object):
    def __init__(self):
        # make tello object
        self.tello = Tello()

        # Drone velocities
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

        self.send_rc_control = False

    def run(self):
        if not self.tello.connect():
            self.tello.connect()
            print('Tello Connected')

        if not self.tello.set_speed(self.speed):
            print('Not set speed to lowest as possible')

        should_stop = False
        img_count = 0
        OVERRIDE = False
        oSpeed = args.override_speed
        tDistance = args.distance
        print(self.tello.get_battery())
        self.tello.streamon()

        # Safety zone X
        szx = args.safety_x

        # safety zone Y
        szy = args.safety_y

        if args.debug:
            print('DEBUG MODE ENABLED!!!')

        while should_stop == False:
            self.update()

            frame = self.tello.get_frame_read().frame

            imgGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            faces = face_cascade.detectMultiScale(
                imgGray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30),
                flags=cv2.CASCADE_SCALE_IMAGE
            )
            i = 0

            the_time = str(datetime.datetime.now()).replace(
                ':', '-').replace('.', '_')

            # Key listener
            # if kp.getKey('t'):
            #    print('TAKEOFF')
            #    self.tello.takeoff()

            # elif kp.getKey('l'):
            #    print('LANDING')
            #    self.tello.land()

            # Target Size
            tSize = faceSizes[tDistance]

            # Center dimentions
            cWidth = int(dimentions[0] / 2)
            cHeight = int(dimentions[1] / 2)

            noFaces = len(faces) == 0

            for (x, y, w, h) in faces:

                # Face box properties
                fbCol = (255, 0, 0)
                fbStroke = 2

                # End coords are end of bounding box
                end_cord_x = x + w
                end_cord_y = y + h
                end_size = w * 2

                # target coorinates
                targ_cord_x = int((end_cord_x + x) / 2)
                targ_cord_y = int((end_cord_y + y) / 2) + UDOffset

                # calculate vectors
                vTrue = np.array((cWidth, cHeight, tSize))
                vTarget = np.array((targ_cord_x, targ_cord_y, end_size))
                vDistance = vTrue - vTarget

                # Draw bounding box
                cv2.rectangle(frame, (x, y), (end_cord_x,
                                              end_cord_y), fbCol, fbStroke)

                # Draw target as circle
                cv2.circle(frame, (targ_cord_x, targ_cord_y),
                           10, (0, 255, 0), 2)

                safety_zone_p1 = str(targ_cord_x - szx)
                safety_zone_p2 = str(targ_cord_y - szy)

                # Draw safey zone
                safety_zone_p1_end = str(targ_cord_x + szx)
                safety_zone_p2_end = str(targ_cord_y + szy)

                cv2.rectangle(frame, (safety_zone_p1, safety_zone_p2),
                              (safety_zone_p1_end, safety_zone_p2_end), (0, 255, 0), fbStroke)


                # Draw estimated drone vector position in relation to bound box
                cv2.putText(frameRet, str(vDistance), (0, 64),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                # Draw center screen circle
                cv2.circle(frame, (cWidth, cHeight), 10, (0, 0, 255), 2)

                if not args.debug:
                    # for turning
                    if vDistance[0] < -szx:
                        self.yaw_velocity = 5
                    elif vDistance[0] > szx:
                        self.yaw_velocity = -5
                    else:
                        self.yaw_velocity = 0

                # Up & Down
                    if vDistance[1] > szy:
                        self.up_down_velocity = 5
                    elif vDistance[0] < -szy:
                        self.up_down_velocity = -5
                    else:
                        self.up_down_velocity = 0

                    F = 0
                    if abs(vDistance[2]) > acc[tDistance]:
                        F = S

                    # Forwards & Backwards
                    if vDistance[2] > 0:
                        self.for_back_velocity = S + F
                    elif vDistance[2] < 0:
                        self.for_back_velocity = -S - F
                    else:
                        self.for_back_velocity = 0

                # If no faces detected
                if noFaces:
                    self.yaw_velocity = 0
                    self.up_down_velocity = 0
                    self.for_back_velocity = 0
                    print('NO TARGET')

            dCol = lerp(np.array((0, 0, 255)), np.array(
                (255, 255, 255)), tDistance + 1 / 7)

            if OVERRIDE:
                show = 'OVERRIDE: {}'.format(oSpeed)
                dCol = (255, 255, 255)
            else:
                show = 'AI: {}'.format(str(tDistance))

            # Draw distance choosen
            cv2.putText(frame, show, (32, 664),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, dCol, 2)

            # resize cv2 window
            master_window = cv2.resize(frame, (360, 240))

            # Display resulting frame
            cv2.imshow(f'Tello tracking...', master_window)

        # Print battery on exit
        self.tello.get_battery()

        # When done, release capture
        cv2.destroyAllWindows()

        self.tello.end()

    def battery(self):
        return self.tello.get_battery()[:2]

    def update(self):
        if self.send_rc_control:
            self.tello.send_rc_control(
                self.left_right_velocity, self.for_back_velocity, self.up_down_velocity, self.yaw_velocity)


def lerp(a, b, c):
    return a + c * (b-a)


def main():
    master = AI()

    master.run()


if __name__ == '__main__':
    main()
