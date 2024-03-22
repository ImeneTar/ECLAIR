#!/usr/bin/env python3


from flask import Flask, request, jsonify, render_template
from flask_cors import CORS
from openai import OpenAI
from werkzeug.utils import secure_filename
import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
import os
import threading
import re


prompt = ("You are working with a discrete Markov Decision Process (MDP) environment that simulates a gridworld. The agent navigates this grid to pick a cube and place it in a specific location. The agent has a set of actions available: \\"
          "Action Space: The agent can move up (action 0), down (action 1), left (action 2), or right (action 3), pick up the cube (action 4), and put down the cube (action 5).)"
          "Your task is to interpret natural language feedback given in response to the agent's past actions, converting this feedback into a structured format for each variable: evaluative feedback, corrective feedback, and advice for future actions. It's crucial to consider both the current feedback and any relevant past feedback when making your interpretations."
          "Here’s how to structure your responses:"
          "Evaluative: Assign a numerical value based on the sentiment of the feedback:) +1 for positive feedback (indicating the action was good or correct). -1 for negative feedback (indicating the action was bad or incorrect). 0 if the feedback is neutral or if no explicit evaluative feedback is given."
          "Corrective: If the past action was incorrect, specify the action that should have been taken instead, choosing from the defined action space (up, down, left, right, pick up, put down). This feedback is focused on the past action only. If the sentiment of the feedback is correct or neutral, or no alternative action is implied, corrective feedback should be None. "
          "Advice: Provide a suggestion for the next action the agent should take. If the feedback does not imply a specific next action or is focused solely on correcting a past mistake without suggesting a next move, Advice should be None."
          "Example:"\
          "Given Feedback: ""Action=0, No, turn left  """\
          "Evaluative=-1 \n Corrective=None (no correction for past action) \n Advice=2" \
          "Given Feedback: ""Action=1, Now go up """\
          "Evaluative=0 \n Corrective=None \n Advice=0"\
          "Given Feedback: ""Action=0, yes ,turn left"""\
          "Evaluative=1 \n Corrective=None \n Advice=2"\
          "Given Feedback: ""Action=0, No, you should have turned left"""\
          "Evaluative=-1 \n Corrective=2 \n Advice=None (no advice for next action is given)"\
          "Given Feedback: ""Action=4, Yeah!"""\
          "Evaluative=1 \n Corrective=0 \n Advice=None (no advice for next action is given)"\
          "Given Feedback: ""Action=1, Continue like this!"""\
          "Evaluative=1 \n Corrective=0 \n Advice=1 (same as previous action)")


transcription = None
evaluative = "None"
corrective = "None"
advice = "None"
action = -1

def convert_audio(input_path, output_format='mp3'):
    output_path = f"{input_path.rsplit('.', 1)[0]}.{output_format}"
    command = ['ffmpeg', '-i', input_path, output_path]
    subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return output_path

def get_interpretation(client, feedback):
    completion = client.chat.completions.create(
    model="gpt-3.5-turbo",
    temperature=0,
    messages=[
    {"role": "system", "content": prompt},
    {"role": "user", "content": feedback}
    ]
    )
    return completion.choices[0].message

def get_feedback(text):
    lines = [line.strip().strip('"') for line in text.strip().split('\n')]
    evaluative = None
    corrective = None
    advice = None

    delimiters = "=", " =","= ", ":"
    regex_pattern = '|'.join(map(re.escape, delimiters))


    for line in lines:
        key, value = re.split(regex_pattern, line)
        #key, value = line.split('=', ':')
        if key == "Evaluative":
            evaluative = value.replace(" ", "")
            matches = re.findall(r'-?\d+', evaluative)
            if matches:
                # Return the first match as an integer
                evaluative = matches[0]
            else:
                # Return None if no integer is found
                evaluative = 'None'
        elif key == "Corrective":
            corrective = value.replace(" ", "")
            matches = re.findall(r'-?\d+', corrective)
            if matches:
                # Return the first match as an integer
                corrective = matches[0]
            else:
                corrective = 'None'

        elif key == "Advice":
            advice = value.replace(" ", "")
            matches = re.findall(r'-?\d+', advice)
            if matches:
                # Return the first match as an integer
                advice = matches[0]
            else:
                advice = 'None'

    return evaluative, corrective, advice

feedback = 0


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_evaluative = self.create_publisher(String, 'evaluative', 10)
        self.publisher_corrective = self.create_publisher(String, 'corrective', 10)
        self.publisher_advice = self.create_publisher(String, 'advice', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.evaluative = 'None'
        self.corrective = 'None'
        self.advice = 'None'
        self.reset = False
        self.get_feedback = False
        self.action = -1


        self.subscription_reset = self.create_subscription(
            Bool,
            'reset',
            self.get_reset,
            10)
        self.subscription_reset  # prevent unused variable warning

        self.subscription_get_feedback = self.create_subscription(
            Bool,
            'get_feedback',
            self.get_get_feedback,
            10)
        self.subscription_get_feedback  # prevent unused variable warning

        self.subscription_past_action= self.create_subscription(
            Int32,
            'past_action',
            self.get_action,
            10)
        self.subscription_past_action  # prevent unused variable warning


    def get_reset(self, msg):
        global transcription
        global evaluative, corrective, advice
        self.reset = msg.data
        if self.reset:
            transcription = 'Tap on audio to provide your feedback'
            evaluative = 'None'
            corrective = 'None'
            advice = 'None'

    def get_get_feedback(self, msg):
        self.get_feedback = msg.data
        if self.get_feedback:
            print("Transcription in get:", transcription)
            if transcription != None:
                print("getting interpereation")
                prompting = "action=" + str(self.action) + ", " + transcription
                interpretation = get_interpretation(client, prompting)
                print(interpretation.content)
                self.evaluative, self.corrective, self.advice = get_feedback(interpretation.content)
    def get_action(self,msg):
        global action
        self.action = msg.data
        action = self.action





    def timer_callback(self):
        #publishing evaluative
        global evaluative, corrective, advice
        msg_evaluative = String()
        msg_evaluative.data = evaluative
        self.publisher_evaluative.publish(msg_evaluative)

        #publishing corrective
        msg_corrective = String()
        msg_corrective.data = corrective
        self.publisher_corrective.publish(msg_corrective)

        #publishing advice
        msg_advice= String()
        msg_advice.data = advice
        self.publisher_advice.publish(msg_advice)




app = Flask(__name__, template_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates'))
CORS(app, resources={r"/transcribe": {"origins": "http://192.168.8.108:5000"}}) #192.168.8.108

# Ensure your OpenAI API key is set as an environment variable
client = OpenAI()  #insert openai key

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/transcribe', methods=['POST'])
def transcribe_audio():
    global transcription, evaluative, corrective, advice
    global action
    if 'audio' not in request.files:
        return jsonify({'error': 'No audio file provided'}), 400

    audio_file = request.files['audio']
    filename = secure_filename(audio_file.filename)

    audio_file.save(filename)
    converted_file_path = convert_audio(filename, 'mp3')
    audio_file = open(converted_file_path, "rb")
    transcript = client.audio.transcriptions.create(
        model="whisper-1",
        file=audio_file
    )
    print("Transcript:", transcript)
    os.remove(filename)
    os.remove(converted_file_path)
    transcription = transcript.text
    print("getting interpereation")
    prompting = "action=" + str(action) + ", " + transcription
    interpretation = get_interpretation(client, prompting)
    print(interpretation.content)
    evaluative, corrective, advice = get_feedback(interpretation.content)

    # except Exception as e:
    #     return jsonify({'error': str(e)}), 500

    return jsonify({'transcription': transcription})


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




