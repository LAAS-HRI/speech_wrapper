#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import time
import rospy
import underworlds
from underworlds.types import Situation
from geometry_msgs.msg import Point
from std_msgs.msg import String
from nao_interaction_msgs.srv import Say
from speech_wrapper.srv import Speak, SpeakTo
from head_manager.msg import TargetWithPriority

SPEAKING_ATTENTION_POINT_PUBLISHER = "/head_speaking_target"

class SpeechWrapperNode(object):
    def __init__(self, ctx, world):
        rospy.loginfo("[speech_wrapper] Waiting for service /naoqi_driver/tts/say")
        rospy.wait_for_service("/naoqi_driver/tts/say")
        rospy.loginfo("[speech_wrapper] Ready to use !")
        self.world = ctx.worlds[world]
        self.ros_services_proxy = {"say": rospy.ServiceProxy('/naoqi_driver/tts/say', Say)}
        self.ros_services = {"speak": rospy.Service("speak", Speak, self.handle_speak),
                             "speakTo": rospy.Service("speakTo", SpeakTo, self.handle_speak_to)}
        self.ros_pub = {"speaking_attention_point": rospy.Publisher(SPEAKING_ATTENTION_POINT_PUBLISHER,
                                                                    TargetWithPriority, queue_size=5)}
        self.log_pub = {"situation_log": rospy.Publisher("speech_wrapper/log", String, queue_size=5)}
        self.current_situations_map = {}
        self.attention_point = TargetWithPriority()

    def start_n2_situation(self, timeline, predicate, subject_name, object_name, isevent=False):
        description = predicate + "(" + subject_name + "," + object_name + ")"
        sit = Situation(desc=description)
        sit.starttime = time.time()
        if isevent:
            sit.endtime = sit.starttime
        self.current_situations_map[description] = sit
        self.log_pub["situation_log"].publish("START " + description)
        timeline.update(sit)
        return sit.id

    def start_n1_situation(self, timeline, predicate, subject_name, isevent=False):
        description = predicate + "(" + subject_name + ")"
        sit = Situation(desc=description)
        sit.starttime = time.time()
        if isevent:
            sit.endtime = sit.starttime
        self.current_situations_map[description] = sit
        self.log_pub["situation_log"].publish("START " + description)
        timeline.update(sit)
        return sit.id

    def end_n1_situation(self, timeline, predicate, subject_name):
        description = predicate + "(" + subject_name + ")"
        sit = self.current_situations_map[description]
        self.log_pub["situation_log"].publish("END " + description)
        try:
            timeline.end(sit)
        except Exception as e:
            rospy.logwarn("[speech_wrapper] Exception occurred : " + str(e))

    def end_n2_situation(self, timeline, predicate, subject_name, object_name):
        description = predicate + "(" + subject_name + "," + object_name + ")"
        sit = self.current_situations_map[description]
        self.log_pub["situation_log"].publish("END " + description)
        try:
            timeline.end(sit)
        except Exception as e:
            rospy.logwarn("[speech_wrapper] Exception occurred : " + str(e))

    def handle_speak(self, req):
        self.start_n1_situation(self.world.timeline, "speaking", "robot")
        self.ros_services_proxy["say"](req.text)
        self.end_n1_situation(self.world.timeline, "speaking", "robot")
        return True

    def handle_speak_to(self, req):
        self.start_n2_situation(self.world.timeline, "speakingTo", "robot", req.human_frame_id)
        self.ros_services_proxy["say"](req.text)
        target_with_priority = TargetWithPriority()
        target_with_priority.target = req.point_to_look
        target_with_priority.priority = req.priority
        self.attention_point = target_with_priority
        self.end_n2_situation(self.world.timeline, "speakingTo", "robot", req.human_frame_id)
        return True

    def publish_attention_point(self):
        self.ros_pub["speaking_attention_point"].publish(self.attention_point)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish_attention_point()
            rate.sleep()

if __name__ == "__main__":
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)

    # Manage command line options
    import argparse

    parser = argparse.ArgumentParser(description="Handle speech synthesis")
    parser.add_argument("world", help="The world where to write the situation associated to speech")
    args = parser.parse_args()

    rospy.init_node("speech_wrapper", anonymous=True)

    with underworlds.Context("speech_wrapper") as ctx:  # Here we connect to the server
        SpeechWrapperNode(ctx, args.world).run()