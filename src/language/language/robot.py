#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
import os
import threading
import anki_vector
from anki_vector.util import degrees, distance_mm, speed_mmps, radians
from .utils import move_right, move_up, move_left, move_down, get_state, get_current_position, get_initial_position, put_down, pick


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        self.publisher_start = self.create_publisher(Bool, 'connected', 10)
        self.publisher_state = self.create_publisher(Int32, 'state', 10)
        self.publisher_end = self.create_publisher(Bool, 'end', 10)
        self.publisher_done = self.create_publisher(Bool, 'done', 10)
        self.publisher_action_reset = self.create_publisher(Bool, 'action_reset', 10)
        self.connected = False
        self.state = 0
        self.action = -1
        self.action_done = False
        self.done = False
        self.interrupt = False
        self.action_reset = False

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription_action = self.create_subscription(
            Int32,
            'action',
            self.get_action,
            10)
        self.subscription_action  # prevent unused variable warning

        self.subscription_interrupt = self.create_subscription(
            Bool,
            'interrupt',
            self.get_interrupt,
            10)
        self.subscription_interrupt



        self.subscription_state = self.create_subscription(
            Int32,
            'state',
            self.get_state,
            10)
        self.subscription_state

    def timer_callback(self):
        # publishing start
        msg_start = Bool()
        msg_start.data = self.connected
        self.publisher_start.publish(msg_start)

        # publishing state
        msg_state = Int32()
        msg_state.data = self.state
        self.publisher_state.publish(msg_state)

        # publishing end of action
        msg_end = Bool()
        msg_end.data = self.action_done
        self.publisher_end.publish(msg_end)

        # publishing done
        msg_done = Bool()
        msg_done.data = self.done
        self.publisher_done.publish(msg_done)

        #publish action_reset
        msg_action_reset = Bool()
        msg_action_reset.data = self.action_reset
        self.publisher_action_reset.publish(msg_action_reset)

    def get_action(self, msg):
        self.action = msg.data
        if self.action != -1:
            self.action_done = False


    def get_interrupt(self, msg):
        self.interrupt = msg.data
        if self.interrupt:
            self.connected = False
            self.state = 0
            self.done = False


    def get_state(self, msg):
        self.state = msg.data

def step(robot, state, action):
    if action == 0: #up
        move_up(robot, state)
    elif action == 1: #down
        move_down(robot, state)
    elif action == 2: #StoppableThreadleft
        move_left(robot, state)
    elif action== 3: #right
        move_right(robot, state)
    elif action == 4: #pick
        pick(robot, state)
    elif action == 5: #put down
        put_down(robot, state)
    done = False
    if action == 5 and state > 24:
        done = True
    if state > 24 and robot.lift_height_mm < 50:
        done = True
    return done

def control_robot(robot_node):
    args = anki_vector.util.parse_command_args()
    with anki_vector.Robot(args.serial, ip="192.168.8.107") as robot: #192.168.8.107
        robot.behavior.set_lift_height(0)
        while True:
            if not robot_node.connected:
                print("is robot connected:", robot_node.connected)
                input_user = input("Type start to begin the study \n")
                if input_user == "start":
                    robot_node.connected = True
                    robot_node.action_done = False
                    print(robot_node.connected)
                    print("stat sleep")
                    time.sleep(4)
                    print("end sleep")


            if robot_node.connected == True:

                start_position = get_initial_position(robot)
                print("getting position")
                while robot_node.connected:
                    robot_node.state = get_state(robot, start_position)
                    time.sleep(0.5)
                    action = robot_node.action
                    if action != -1:
                        time.sleep(0.5)
                        robot_node.action_reset = True
                        print("State is:", robot_node.state)
                        print("Action is: ", robot_node.action)
                        print("Action done: ", robot_node.action_done)
                        robot_node.done = step(robot, robot_node.state, action)
                        time.sleep(3)
                        robot_node.action_done = True
                        time.sleep(1)
                        robot_node.action_reset = False
                        print("Action done: ", robot_node.action_done)




def main(args=None):

    rclpy.init(args=args)
    robot_node = Robot()

    input_thread = threading.Thread(target=control_robot, args=(robot_node,))
    input_thread.start()

    # input_user = input("Type start to begin the study \n")
    # if input_user == "start":
    #     robot_node.connected = True


    try:
        rclpy.spin(robot_node)
    except KeyboardInterrupt:
        robot_node.connected = False
        robot_node.action_done = True
        robot_node.action = -1
        robot_node.state = 0
        robot_node.done = False
        robot_node.timer_callback()

    finally:
        robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
