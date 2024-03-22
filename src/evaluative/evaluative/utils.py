import math
from anki_vector.util import degrees, distance_mm, speed_mmps, radians
import time
import os


def get_initial_position(robot):
    pose = robot.pose
    print("intial pose:", pose)
    return pose.position.x, pose.position.y
def get_current_position(robot, start_position):
    pose = robot.pose
    return pose.position.x-start_position[0], pose.position.y-start_position[1]


def get_state(robot, start_position, width=5):
    x, y = get_current_position(robot, start_position)

    if x < 0:
        x = 0
    if y < 0:
        y = 0

    i = int(x / 115)
    j = int(y / 115)
    if abs(x % 115) > 60:
        i += 1
    if y % 115 > 60:
        j += 1

    s = i + j * width

    if robot.lift_height_mm > 50:
        s += 25
    return s


def move_up(robot, state):
    target_angle = math.pi/2
    angle = robot.pose_angle_rad
    robot.behavior.turn_in_place(radians(target_angle-angle ))
    if state in [20, 21, 22, 23, 24, 45, 46, 47, 48, 49]:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        robot.audio.stream_wav_file(os.path.join(dir_path, 'vector_bell_whistle.wav'), volume=100)


    else:
        robot.behavior.drive_straight(distance_mm(115), speed_mmps(50))


def move_down(robot, state):
    target_angle = - math.pi/2
    angle = robot.pose_angle_rad
    robot.behavior.turn_in_place(radians(target_angle-angle))

    if state in [0, 1, 2, 3, 4,9, 25, 26, 27, 28, 29]:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        robot.audio.stream_wav_file(os.path.join(dir_path, 'vector_bell_whistle.wav'), volume=100)


    else:
        robot.behavior.drive_straight(distance_mm(115), speed_mmps(50))


def move_right(robot, state):
    target_angle = 0
    angle = robot.pose_angle_rad
    robot.behavior.turn_in_place(radians(target_angle-angle))


    if state in [3,  4, 9, 14, 19, 24, 29, 34, 39, 44, 49]:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        robot.audio.stream_wav_file(os.path.join(dir_path, 'vector_bell_whistle.wav'), volume=100)

    else:
        robot.behavior.drive_straight(distance_mm(115), speed_mmps(50))


def move_left(robot, state):
    target_angle = math.pi
    angle = robot.pose_angle_rad
    robot.behavior.turn_in_place(radians(target_angle-angle))
    if state in [0, 5, 10, 15, 20, 25, 30, 35, 40, 45]:
        dir_path = os.path.dirname(os.path.realpath(__file__))
        robot.audio.stream_wav_file(os.path.join(dir_path, 'vector_bell_whistle.wav'), volume=100)
    else:
        robot.behavior.drive_straight(distance_mm(115), speed_mmps(50))

def pick(robot, state):


    if state == 3 :
        robot.behavior.say_text("Connecting to the cube")
        print("is cube connected? ", robot.world.connected_light_cube)

        while not robot.world.connected_light_cube:
            robot.world.connect_cube()
            print("Cube connected")
            robot.behavior.say_text("Successfully connected to the cube")
            time.sleep(3.0)
        print("Attempting to pick up the Light Cube...")
        robot.behavior.say_text("Attempting to pick up the cube")
        #turn right
        target_angle = 0
        angle = robot.pose_angle_rad
        robot.behavior.turn_in_place(radians(target_angle - angle))
        if robot.world.connected_light_cube:
            robot.behavior.pickup_object(robot.world.connected_light_cube)


    elif state == 9:
        robot.behavior.say_text("Connecting to the cube")
        print("is cube connected? ", robot.world.connected_light_cube)

        while not robot.world.connected_light_cube:
            robot.world.connect_cube()
            print("Cube connected")
            robot.behavior.say_text("Successfully connected to the cube")
            time.sleep(3.0)
        print("Attempting to pick up the Light Cube...")
        robot.behavior.say_text("Attempting to pick up the cube")
        # turn down
        target_angle = -math.pi / 2
        angle = robot.pose_angle_rad
        robot.behavior.turn_in_place(radians(target_angle - angle))
        if robot.world.connected_light_cube:
            robot.behavior.pickup_object(robot.world.connected_light_cube)


    elif state < 25:
        print("nothing to pick")
        robot.behavior.set_lift_height(1.0)
        time.sleep(0.5)
        robot.behavior.set_lift_height(0)
        robot.behavior.say_text("Nothing to pick")

    else:
        print("nothing to pick")
        robot.behavior.say_text("Cube already picked up")

def put_down(robot, state):
    if state > 24:
        robot.behavior.say_text("Putting down the cube")
        robot.behavior.place_object_on_ground_here()
    else:
        print("nothing to pick")
        robot.behavior.set_lift_height(1.0)
        time.sleep(0.5)
        robot.behavior.set_lift_height(0)
        robot.behavior.say_text("Nothing to put down")

