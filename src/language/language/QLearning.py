#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
import numpy as np
import random

import threading
from .qlearning import Q_learning
import pickle
import os
class Agent(Node):

    def __init__(self):
        super().__init__('agent')
        self.publisher_action = self.create_publisher(Int32, 'action', 10)
        self.publisher_reset = self.create_publisher(Bool, 'reset', 10)
        self.publisher_past_action = self.create_publisher(Int32, 'past_action', 10)
        self.publisher_get_feedback = self.create_publisher(Bool, 'get_feedback', 10)
        self.publisher_interrupt = self.create_publisher(Bool, 'interrupt', 10)
        #self.publisher_state = self.create_publisher(Int32, 'state', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.evaluative = 'None'
        self.corrective = 'None'
        self.advice = 'None'
        self.reset = False
        self.get_feedback = False
        self.stop = "False"
        self.connected = False
        self.state = 1
        self.action = -1
        self.action_end = False
        self.done = False
        self.interrupt = False
        self.action_reset = False
        self.past_action = -1


        self.subscription_evaluative = self.create_subscription(
            String,
            'evaluative',
            self.get_evaluative,
            10)
        self.subscription_evaluative  # prevent unused variable warning

        self.subscription_corrective = self.create_subscription(
            String,
            'corrective',
            self.get_corrective,
            10)
        self.subscription_corrective  # prevent unused variable warning

        self.subscription_advice = self.create_subscription(
            String,
            'advice',
            self.get_advice,
            10)
        self.subscription_advice  # prevent unused variable warning



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


        # publishing reset
        msg_reset = Bool()
        msg_reset.data = self.reset
        self.publisher_reset.publish(msg_reset)


        #publish connected
        msg_interrupt = Bool()
        msg_interrupt.data = self.interrupt
        self.publisher_interrupt.publish(msg_interrupt)

        #publish get_feedback
        msg_get_feedback = Bool()
        msg_get_feedback.data = self.get_feedback
        self.publisher_get_feedback.publish(msg_get_feedback)



    def get_evaluative(self, msg):
        self.evaluative= msg.data


    def get_corrective(self, msg):
        self.corrective = msg.data


    def get_advice(self, msg):
        self.advice = msg.data


    def get_stop(self, msg):
        self.stop = msg.data

    def get_connection(self, msg):
        self.connected = msg.data
        if self.connected:
            self.interrupt = False
            self.get_feedback = False

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
    evaluative_total = []
    corrective_total = []
    advice_total = []
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
            #print("waiting for connection with robot")
            pass
        time.sleep(1)
        print("Episode:", ep)
        t = 0
        evalutive_ep = []
        corrective_ep = []
        advice_ep = []
        states_ep = np.zeros((50,))
        while agent_node.connected and not agent_node.done and t < 15:
            time.sleep(1)
            print("t: ", t)
            end_ep = False
            agent_node.past_action = -1
            s =  agent_node.state
            states_ep [s] += 1
            previous_step = s
            print("State is:", s)
            time.sleep(0.5)

            action = agent.select_action(s, agent_node.advice)
            agent_node.reset = True
            time.sleep(1) #was 2
            agent_node.reset = False
            agent_node.action = int(action)
            agent_node.past_action = int(action)
            print("Action:", action )
            time.sleep(0.5)
            print("Is action done: ", agent_node.action_end)
            while not agent_node.action_end:
                #print("waiting for action")
                pass
            print("getting feedback")
            time.sleep(4) #was 3
            print("Evaluative:", agent_node.evaluative)
            print("Corrective:", agent_node.corrective)
            print("Advice:", agent_node.advice)
            time.sleep(0.5)
            agent.update(s, action, agent_node.evaluative, agent_node.corrective)
            evalutive_ep.append(agent_node.evaluative)
            corrective_ep.append(agent_node.corrective)
            advice_ep.append(agent_node.advice)
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
                t +=15
                success.append(0)
            elif done_past and s in [48]:
                success.append(1)
            else:
                success.append(0)
                t+= 10
            states += states_ep
            print("Success: ", success[-1])
            print("Performance: ", -t)
            performance.append(-t)
            evaluative_total.append(evalutive_ep)
            corrective_total.append(corrective_ep)
            advice_total.append(advice_ep)
            #end_ep = False


    save_path = os.path.join("GPT", f"participant_{participant_id}")
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    #saving_performance
    f = open(save_path + "/performance", "wb")
    pickle.dump(performance, f)
    f.close()

    # saving_evaluative
    f = open(save_path + "/evaluative", "wb")
    pickle.dump(evaluative_total, f)
    f.close()

    # saving_corrective
    f = open(save_path + "/corrective", "wb")
    pickle.dump(corrective_total, f)
    f.close()

    # saving_advice
    f = open(save_path + "/advice", "wb")
    pickle.dump(advice_total, f)
    f.close()

    #saving_success
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
        agent_node.get_feedback = False
        agent_node.timer_callback()

    finally:
        agent_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
