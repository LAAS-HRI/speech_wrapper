#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import time
import rospy
import underworlds
from underworlds.types import Situation
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from naoqi import ALProxy
from speech_wrapper.srv import Speak, SpeakTo
from head_manager.msg import TargetWithPriority

SPEAKING_ATTENTION_POINT_PUBLISHER = "/head_manager/head_speaking_target"

SPEAK_ALONE_PRIORITY = 50

VOICE_SPEED = 80


class SpeechWrapperNode(object):
    def __init__(self, ctx, world, nao_ip, nao_port):

        self.tts = ALProxy("ALTextToSpeech", nao_ip, nao_port)
        self.tts.setParameter("speed", VOICE_SPEED)
        self.nao_ip = nao_ip
        self.nao_port = nao_port
        self.world = ctx.worlds[world]

        self.ros_services = {"speak": rospy.Service("speech_wrapper/speak", Speak, self.handle_speak),
                             "speak_to": rospy.Service("speech_wrapper/speak_to", SpeakTo, self.handle_speak_to)}
        self.ros_pub = {"speaking_attention_point": rospy.Publisher(SPEAKING_ATTENTION_POINT_PUBLISHER,
                                                                    TargetWithPriority, queue_size=5),
                        "vizu": rospy.Publisher("speech_wrapper/speaking_attention_point", PointStamped, queue_size=5),
                        "speech": rospy.Publisher("robot_dialogue", String, queue_size=5)}
        self.log_pub = {"isSpeakingTo": rospy.Publisher("predicates_log/speakingto", String, queue_size=5),
                        "isSpeaking": rospy.Publisher("predicates_log/speaking", String, queue_size=5)}
        self.current_situations_map = {}
        self.attention_point = None

    def start_predicate(self, timeline, predicate, subject_name, object_name=None, isevent=False):
        if object_name is None:
            description = predicate + "(" + subject_name + ")"
        else:
            description = predicate + "(" + subject_name + "," + object_name + ")"
        sit = Situation(desc=description)
        sit.starttime = time.time()
        if isevent:
            sit.endtime = sit.starttime
        self.current_situations_map[description] = sit
        self.log_pub[predicate].publish("START " + description)
        timeline.update(sit)
        return sit.id

    def end_predicate(self, timeline, predicate, subject_name, object_name=None):
        if object_name is None:
            description = predicate + "(" + subject_name + ")"
        else:
            description = predicate + "(" + subject_name + "," + object_name + ")"
        try:
            sit = self.current_situations_map[description]
            self.log_pub[predicate].publish("END " + description)
            timeline.end(sit)
        except Exception as e:
            rospy.logwarn("[speech_wrapper] Exception occurred : " + str(e))

    def handle_speak(self, req):
        self.start_predicate(self.world.timeline, "isSpeaking", "robot")
        target_with_priority = TargetWithPriority()
        target_with_priority.target.header.frame_id = "base_link"
        target_with_priority.target.header.point = Point()
        target_with_priority.target.header.point.x = 2.0
        target_with_priority.target.header.point.y = 1.0
        target_with_priority.target.header.point.z = 0.0
        target_with_priority.priority = SPEAK_ALONE_PRIORITY
        self.attention_point = target_with_priority
        self.ros_pub["speech"].publish(req.text)
        try:
            self.tts.say(req.text)
        except Exception:
            time.sleep(0.2)
            self.tts = ALProxy("ALTextToSpeech", self.nao_ip, self.nao_port)
            self.tts.setParameter("speed", VOICE_SPEED)
            self.tts.say(req.text)
        self.end_predicate(self.world.timeline, "isSpeaking", "robot")
        return True

    def handle_speak_to(self, req):
        target_with_priority = TargetWithPriority()
        target_with_priority.target = req.look_at
        target_with_priority.priority = req.priority
        self.attention_point = target_with_priority
        self.start_predicate(self.world.timeline, "isSpeaking", "robot")
        self.start_predicate(self.world.timeline, "isSpeakingTo", "robot", object_name=req.look_at.header.frame_id)
        self.ros_pub["speech"].publish(req.text)
        try:
            self.tts.say(req.text)
        except Exception:
            time.sleep(0.2)
            self.tts = ALProxy("ALTextToSpeech", self.nao_ip, self.nao_port)
            self.tts.setParameter("speed", VOICE_SPEED)
            self.tts.say(req.text)
        self.end_predicate(self.world.timeline, "isSpeakingTo", "robot", object_name=req.look_at.header.frame_id)
        self.end_predicate(self.world.timeline, "isSpeaking", "robot")
        return True

    def publish_attention_point(self):
        self.ros_pub["speaking_attention_point"].publish(self.attention_point)
        self.ros_pub["vizu"].publish(self.attention_point.target)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.attention_point:
                self.publish_attention_point()
            rate.sleep()

if __name__ == "__main__":
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)

    # Manage command line options
    import argparse

    parser = argparse.ArgumentParser(description="Handle speech synthesis")
    parser.add_argument("world", help="The world where to write the situation associated to speech")
    parser.add_argument("--nao_ip", default="mummer-eth0.laas.fr", help="The robot IP")
    parser.add_argument("--nao_port", default="9559", help="The robot port")
    args = parser.parse_args()

    rospy.init_node("speech_wrapper", anonymous=True)

    with underworlds.Context("speech_wrapper") as ctx:  # Here we connect to the server
        SpeechWrapperNode(ctx, args.world, args.nao_ip, int(args.nao_port)).run()
