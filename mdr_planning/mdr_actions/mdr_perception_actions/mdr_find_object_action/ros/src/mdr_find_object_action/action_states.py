#!/usr/bin/env python
from curses.ascii import isxdigit

from numpy import append
import rospy

import actionlib
from geometry_msgs.msg import PoseStamped
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mas_knowledge_utils.domestic_ontology_interface import DomesticOntologyInterface
from mas_knowledge_base.domestic_kb_interface import DomesticKBInterface
from mdr_find_object_action.msg import FindObjectGoal, FindObjectResult

class FindObjectSM(ActionSMBase):
    def __init__(self, ontology_url,
                 ontology_base_url,
                 ontology_entity_delimiter,
                 ontology_class_prefix,
                 number_of_retries=0,
                 
                 move_base_server='move_base_server',
                 
                 timeout=120.,
                 max_recovery_attempts=1):
        super(FindObjectSM, self).__init__('FindObject', [], max_recovery_attempts)
        self.ontology_url = ontology_url
        self.ontology_base_url = ontology_base_url
        self.ontology_entity_delimiter = ontology_entity_delimiter
        self.ontology_class_prefix = ontology_class_prefix
        self.number_of_retries = number_of_retries
        self.timeout = timeout
        self.ontology_interface = None
        self.kb_interface = None
        
        self.move_base_server = move_base_server
        self.move_base_client = None
        
    def init(self):
        try:
            rospy.loginfo('[find_object] Creating an interface client for ontology %s', self.ontology_url)
            self.ontology_interface = DomesticOntologyInterface(ontology_file=self.ontology_url,
                                                                base_url=self.ontology_base_url,
                                                                entity_delimiter=self.ontology_entity_delimiter,
                                                                class_prefix=self.ontology_class_prefix)
        except Exception as exc:
            rospy.logerr('[find_object] Could not create an ontology interface client: %s', exc)
        

        try:
            rospy.loginfo('[find_object] Creating a knowledge base interface client')
            self.kb_interface = DomesticKBInterface()
        except Exception as exc:
            rospy.logerr('[find_object] Could not create a knowledge base interface client: %s', exc)
        
        
        try:
            self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
            rospy.loginfo('[pickup] Waiting for %s server', self.move_base_server)
            self.move_base_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[pickup] %s server does not seem to respond: %s',
                         self.move_base_server, str(exc))
        
        return FTSMTransitions.INITIALISED


    def running(self):
        obj_name = None
        obj_location = None
        relation = None
        goal = MoveBaseGoal()
        if self.goal.goal_type == FindObjectGoal.NAMED_OBJECT:
            obj_name = self.goal.object_name
            location, predicate = self.kb_interface.get_object_location(obj_name)
            if location:
                rospy.loginfo('[find_object] Found %s %s %s', obj_name, predicate, location)
                # TODO: verify that the object is still at that location
                obj_location = location
                relation = predicate
                self.result = self.set_result(True, obj_location, relation)
                return FTSMTransitions.DONE

            rospy.loginfo('[find_object] %s not found in the knowledge base; querying the ontology', obj_name)
            location = self.ontology_interface.get_default_storing_location(obj_name=obj_name)
            all_possible_locations = self.ontology_interface.get_all_subjects_and_objects('possibleLocations')
            if location:
                predicate = 'in'
                rospy.loginfo('[find_object] %s is usually %s: %s', obj_name, predicate, ', '.join(x[1] for x in all_possible_locations))
                natural_location = []
                possible_locations = []
                for name in all_possible_locations:
                    natural_location.append(self.ontology_interface.get_obj_location(name[1]))
                    possible_locations.append(name[1])
                # TODO: check if the object is at the default location
                obj_location = location
                relation = predicate
                self.result = self.set_result(True, obj_location, relation,natural_location,possible_locations)

                ### Navigation from move_base_action_client_test
                goal.goal_type = MoveBaseGoal.NAMED_TARGET
                goal.destination_location = str(natural_location[0])
                print("Lucy destination: ", natural_location[0])
                timeout = 15.0
                rospy.loginfo('Sending action lib goal to move_base_server, ' +
                          'destination : ' + goal.destination_location)
            
                self.move_base_client.send_goal(goal)
            
                self.move_base_client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
                return FTSMTransitions.DONE

            rospy.loginfo('[find_object] %s not found', obj_name)
            obj_location = None
            relation = None
            self.result = self.set_result(True, obj_location, relation)
            return FTSMTransitions.DONE
        elif self.goal.goal_type == FindObjectGoal.OBJECT_CATEGORY:
            obj_category = self.goal.object_name
            category_objects = self.kb_interface.get_category_objects(obj_category)
            if category_objects:
                for obj_name in category_objects:
                    location, predicate = self.kb_interface.get_object_location(obj_name)
                    if location:
                        rospy.loginfo('[find_object] Found %s %s %s', obj_name, predicate, location)
                        # TODO: verify that the object is still at that location
                        obj_location = location
                        relation = predicate
                        self.result = self.set_result(True, obj_location, relation)
                        return FTSMTransitions.DONE
                    else:
                        rospy.loginfo('[find_object] The location of %s is unknown', obj_name)
            else:
                rospy.loginfo('[find_object] No object of category %s was found in the knowledge base; querying the ontology', obj_category)

            location = self.ontology_interface.get_default_storing_location(obj_category=obj_category)
            if location:
                predicate = 'in'
                rospy.loginfo('[find_object] Objects of %s are usually %s %s', obj_category, predicate, location)
                # TODO: check if an object of the desired category is at the default location
                obj_location = location
                relation = predicate
                self.result = self.set_result(True, obj_location, relation)
                return FTSMTransitions.DONE

            rospy.logerr('[find_object] Object of category %s could not be found', obj_category)
            self.result = self.set_result(False, obj_location, relation)

            return FTSMTransitions.DONE

    def set_result(self, success, obj_location, relation,natural_location,possible_locations):
        result = FindObjectResult()
        result.success = success
        result.object_location = obj_location
        result.relation = relation
        result.natural_location = natural_location
        result.possible_locations = possible_locations
        return result
