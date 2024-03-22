#!/usr/bin/env python3


from flask import Flask, request, jsonify, render_template
from flask_cors import CORS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
import os
import threading



feedback = 0
stop = "False"

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_feedback = self.create_publisher(Int32, 'feedback', 10)
        self.publisher_stop = self.create_publisher(String, 'stop', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.feedback_reset = True

        self.subscription_feedback_reset = self.create_subscription(
            Bool,
            'feedback_reset',
            self.get_feedback_reset,
            10)
        self.subscription_feedback_reset  # prevent unused variable warning



    def get_feedback_reset(self, msg):
        global feedback
        self.feedback_reset = msg.data
        if self.feedback_reset:
            feedback = 0

    def timer_callback(self):
        #publishing feedback
        msg_feedback = Int32()
        msg_feedback.data = int(feedback)
        self.publisher_feedback.publish(msg_feedback)

        #publishing stop
        msg_stop = String()
        msg_stop.data = stop
        self.publisher_stop.publish(msg_stop)


app = Flask(__name__, template_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates'))
CORS(app, resources={r"/transcribe": {"origins": "http://192.168.8.108:5000"}})

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/send_feedback/<new_message>', methods=['POST'])
def get_feedback(new_message):
    global feedback
    feedback = new_message
    print("Received :", feedback)
    return jsonify({'feedback': feedback})



@app.route('/send_stop/<new_message>', methods=['POST'])
def get_stop(new_message):
    global stop
    stop = new_message
    print("Recieved :", stop)
    return jsonify({'feedback': feedback })


def main(args=None):
    flask_thread = threading.Thread(target=app.run,
                                    kwargs={'debug': True, 'host': '0.0.0.0', 'port': 5000, 'use_reloader': False})
    flask_thread.daemon = True  # This ensures the thread exits when the main program finishes
    flask_thread.start()
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




