#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
import time
import rospy
import underworlds
from underworlds.types import Situation
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from nao_interaction_msgs.srv import Say
from speech_wrapper.srv import Speak, SpeakTo
from head_manager.msg import TargetWithPriority

SPEAKING_ATTENTION_POINT_PUBLISHER = "/head_speaking_target"

SPEAK_ALONE_PRIORITY = 50


class SpeechWrapperNode(object):
    def __init__(self, ctx, world):
        rospy.wait_for_service("/naoqi_driver/tts/say")
        self.world = ctx.worlds[world]
        self.ros_services_proxy = {"say": rospy.ServiceProxy('/naoqi_driver/tts/say', Say)}
        self.ros_services = {"speak": rospy.Service("speech_wrapper/speak", Speak, self.handle_speak),
                             "speak_to": rospy.Service("speech_wrapper/speak_to", SpeakTo, self.handle_speak_to)}
        self.ros_pub = {"speaking_attention_point": rospy.Publisher(SPEAKING_ATTENTION_POINT_PUBLISHER,
                                                                    TargetWithPriority, queue_size=5),
                        "vizu": rospy.Publisher("speech_wrapper/speaking_attention_point", PointStamped, queue_size=5)}
        self.log_pub = {"situation_log": rospy.Publisher("speech_wrapper/log", String, queue_size=5)}
        self.current_situations_map = {}
        self.attention_point = TargetWithPriority()

    def start_predicate(self, timeline, predicate, subject_name, object_name, isevent=False):
        description = predicate + "(" + subject_name + "," + object_name + ")"
        sit = Situation(desc=description)
        sit.starttime = time.time()
        if isevent:
            sit.endtime = sit.starttime
        self.current_situations_map[description] = sit
        self.log_pub["situation_log"].publish("START " + description)
        timeline.update(sit)
        return sit.id

    def end_predicate(self, timeline, predicate, subject_name, object_name):
        description = predicate + "(" + subject_name + "," + object_name + ")"
        sit = self.current_situations_map[description]
        self.log_pub["situation_log"].publish("END " + description)
        try:
            timeline.end(sit)
        except Exception as e:
            rospy.logwarn("[speech_wrapper] Exception occurred : " + str(e))

    def handle_speak(self, req):
        self.start_predicate(self.world.timeline, "isSpeakingTo", "robot", "robot")
        target_with_priority = TargetWithPriority()
        target_with_priority.target.header.frame_id = "base_link"
        target_with_priority.target.header.point = Point()
        target_with_priority.target.header.point.x = 2.0
        target_with_priority.target.header.point.y = 1.0
        target_with_priority.target.header.point.z = 0.0
        target_with_priority.priority = SPEAK_ALONE_PRIORITY
        self.attention_point = target_with_priority
        self.ros_services_proxy["say"](req.text)
        self.end_predicate(self.world.timeline, "isSpeakingTo", "robot", "robot")
        return True

    def handle_speak_to(self, req):
        target_with_priority = TargetWithPriority()
        target_with_priority.target = req.look_at
        target_with_priority.priority = req.priority
        self.attention_point = target_with_priority
        self.start_predicate(self.world.timeline, "isSpeakingTo", "robot", req.look_at.header.frame_id)
        self.ros_services_proxy["say"](req.text)
        self.end_predicate(self.world.timeline, "isSpeakingTo", "robot", req.look_at.header.frame_id)
        return True

    def publish_attention_point(self):
        self.ros_pub["speaking_attention_point"].publish(self.attention_point)
        self.ros_pub["vizu"].publish(self.attention_point.target)

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