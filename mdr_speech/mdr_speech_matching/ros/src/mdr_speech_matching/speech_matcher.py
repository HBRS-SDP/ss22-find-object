#!/usr/bin/env python
from __future__ import print_function
import rospy
from speech_matching.speech_matching import SpeechMatching
from std_msgs.msg import String
from mdr_speech_matching.msg import MatchedSentence

class SpeechMatcher(object):

    def __init__(self):
        rospy.init_node("speech_matcher")
        self.sm = SpeechMatching()
        self.sub = rospy.Subscriber("speech_recognizer", String, self.match)
        self.pub = rospy.Publisher("speech_matcher", MatchedSentence, latch=True, queue_size=1)
        self.result = MatchedSentence()

    def match(self, data):
        input_sentence = data.data.strip()
        matching_result = self.sm.match_sentence(input_sentence)

        sentence_type = matching_result[0]
        if sentence_type == "nothing":
            rospy.logerr("No match found.")
            return
        else:
            self.result.type = 0

        # TODO change sentence type to string in order to allow a variety of sentence types.
        # elif sentence_type == "question":
        #     self.result.type = MatchedSentence.TYPE_QUESTION
        # elif sentence_type == "command":
        #     self.result.type = MatchedSentence.TYPE_COMMAND

        self.result.matched_sentence = matching_result[1][0]
        self.result.similarity = matching_result[1][1]

        rospy.loginfo("Received string: '" + self.result.matched_sentence  + "' with a distance of: " + str(self.result.similarity) + ".")
        self.pub.publish(self.result)


def main():
    speech_matcher = SpeechMatcher()
    rospy.spin()
