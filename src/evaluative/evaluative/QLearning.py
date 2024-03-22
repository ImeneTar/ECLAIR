#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
import numpy as np
import random
import os
import pickle


import threading
from .qlearning import Q_learning

class Agent(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_action = self.create_publisher(Int32, 'action', 10)
        self.publisher_feedback_reset = self.create_publisher(Bool, 'feedback_reset', 10)
        self.publisher_interrupt = self.create_publisher(Bool, 'interrupt', 10)
        #self.publisher_state = self.create_publisher(Int32, 'state', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.feedback = 0
        self.feedback_reset = False
        self.stop = "False"
        self.connected = False
        self.state = 1
        self.action = -1
        self.action_end = False
        self.done = False
        self.interrupt = False
        self.action_reset = False


        self.subscription_feedback = self.create_subscription(
            Int32,
            'feedback',
            self.get_feedback,
            10)
        self.subscription_feedback  # prevent unused variable warning

        self.subscription_stop = self.create_subscription(
            String,
            'stop',
            self.get_stop,
            10)
        self.subscription_stop  # prevent unused variable warning

        self.subscription_start = self.create_subscription(
            Bool,
            'connected',
            self.get_connection,
            10)
        self.subscription_start  # prevent unused variable warning

        self.subscription_state = self.create_subscription(
            Int32,
            'state',
            self.get_state,
            10)
        self.subscription_state  # prevent unused variable warning

        self.subscription_end = self.create_subscription(
            Bool,
            'end',
            self.get_end,
            10)
        self.subscription_end  # prevent unused variable warning

        self.subscription_done = self.create_subscription(
            Bool,
            'done',
            self.get_done,
            10)
        self.subscription_done  # prevent unused variable warning

        self.subscription_action_reset = self.create_subscription(
            Bool,
            'action_reset',
            self.get_action_reset,
            10)
        self.subscription_action_reset  # prevent unused variable warning





    def timer_callback(self):
        # publishing action
        msg_action= Int32()
        msg_action.data = self.action
        self.publisher_action.publish(msg_action)


        # publishing feedback_reset
        msg_feedback_reset = Bool()
        msg_feedback_reset.data = self.feedback_reset
        self.publisher_feedback_reset.publish(msg_feedback_reset)


        #publish connected
        msg_interrupt = Bool()
        msg_interrupt.data = self.interrupt
        self.publisher_interrupt.publish(msg_interrupt)



    def get_feedback(self, msg):
        self.feedback = int(msg.data)
    def get_stop(self, msg):
        self.stop = msg.data
    def get_connection(self, msg):
        self.connected = msg.data
        if self.connected:
            self.interrupt = False

    def get_state(self, msg):
        self.state = msg.data
    def get_end(self, msg):
        self.action_end= msg.data
        if self.action_end:
            self.action = -1
    def get_done(self, msg):
        self.done = msg.data

    def get_action_reset(self, msg):
        self.action_reset = msg.data
        if self.action_reset:
            self.action = -1



def send_reset_feedback(agent_node):
    agent_node.feedback_reset = True

def start_learning(agent_node, agent, participant_id):
    performance = []
    feedback_total = []
    success = []
    states = np.zeros((50,))
    done_past = False
    s = 0
    t = 0
    end_ep = False
    ep = 0
    while ep < 8:
        print("Waiting fot connection")
        while not agent_node.connected:
            pass
        time.sleep(2)
        print("Episode:", ep)
        t = 0
        feedback_ep = []
        states_ep = np.zeros((50,))
        while agent_node.connected and not agent_node.done and t < 15:
            time.sleep(1)
            print("t: ", t)
            end_ep = False
            agent_node.feedback_reset = True
            s =  agent_node.state
            states_ep [s] += 1
            print("State is:", s)
            time.sleep(0.5)

            agent_node.feedback_reset = False
            action = agent.select_action(s)
            agent_node.action = int(action)
            print("Action:", action )
            time.sleep(0.5)
            print("Is action done: ", agent_node.action_end)
            while not agent_node.action_end:
                #print("waiting for action")
                pass
            print("Feedback:", agent_node.feedback)
            agent.update(s, action, agent_node.feedback)
            feedback_ep.append(agent_node.feedback)
            print("Done:", agent_node.done)
            done_past = agent_node.done
            t += 1

            if agent_node.done or t == 15:
                print("end of episode")
                agent_node.interrupt = True
                end_ep = True
                time.sleep(1)
                agent_node.interrupt = False

        if end_ep:
            ep += 1
            if not done_past:
                t += 15
                success.append(0)
            elif done_past and s in [48]:
                success.append(1)
            else:
                success.append(0)
                t += 10
            print("Success: ", success[-1])
            print("Performance: ", -t)
            performance.append(-t)
            feedback_total.append(feedback_ep)
            states += states_ep

    save_path = os.path.join("Evaluative", f"participant_{participant_id}")
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    # saving_performance
    f = open(save_path + "/performance", "wb")
    pickle.dump(performance, f)
    f.close()

    # saving_evaluative
    f = open(save_path + "/feedback", "wb")
    pickle.dump(feedback_total, f)
    f.close()

    # saving_success
    f = open(save_path + "/success", "wb")
    pickle.dump(success, f)
    f.close()

    # saving_states
    f = open(save_path + "/states_heatmap", "wb")
    pickle.dump(states, f)
    f.close()

    print("End of training.")





def main(args=None):
    rclpy.init(args=args)

    participant_id = 12

    seed = 0
    random.seed(seed)
    np.random.seed(seed)

    agent_node = Agent()
    agent = Q_learning()

    input_thread = threading.Thread(target=start_learning, args=(agent_node,agent, participant_id))
    input_thread.start()


    try:
        rclpy.spin(agent_node)
    except KeyboardInterrupt:
        agent_node.interrupt = True
        agent_node.timer_callback()

    finally:
        agent_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
