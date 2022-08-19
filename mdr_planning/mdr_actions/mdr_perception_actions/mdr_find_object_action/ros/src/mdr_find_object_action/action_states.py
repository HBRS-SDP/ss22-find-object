#!/usr/bin/env python
from curses.ascii import isxdigit

import numpy as np
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

from mdr_perceive_plane_action.msg import PerceivePlaneAction,PerceivePlaneGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

# from mdr_move_base_action.action_states import MoveBaseSM
class FindObjectSM(ActionSMBase):
    def __init__(self, ontology_url,
                 ontology_base_url,
                 ontology_entity_delimiter,
                 ontology_class_prefix,
                 number_of_retries=0,
                #  pose_description_file='',
                 move_base_server='move_base_server',
                 perceive_plane_server = '/mdr_actions/perceive_plane_server',
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
        
        # self.pose_description_file = pose_description_file

        self.move_base_server = move_base_server
        self.move_base_client = None

        
        self.perceive_plane_server = perceive_plane_server
        
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

        try:
            # spanch2s Perception
            self.perceive_plane_client = actionlib.SimpleActionClient(self.perceive_plane_server,PerceivePlaneAction)
            rospy.loginfo('[pickup] Waiting for %s server', self.perceive_plane_server)
            self.perceive_plane_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[pickup] %s server does not seem to respond: %s',
                         self.perceive_plane_server, str(exc))
        
        return FTSMTransitions.INITIALISED


    def running(self):
        obj_name = None
        obj_location = None
        relation = None
        goal = MoveBaseGoal()
        
        # Perception spanch2s
        perceive_plane_goal = PerceivePlaneGoal()
        perceive_plane_goal.plane_config = 'table'
        perceive_plane_goal.plane_frame_prefix = 'frame_table'

        robot_current_location = rospy.Subscriber('/laser_2d_correct_pose',PoseWithCovarianceStamped,callback=self.location_callback)
        print("Robot_current_location : ",robot_current_location)

        # Navigation
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

                robot_name = self.kb_interface.robot_name
                # print("Robot Name: ", robot_name)
                robot_location = self.kb_interface.get_robot_location(robot_name)
                # print("Robot location in the map",robot_location)
                # natural_locations = []
                # for location in natural_location:
                #     locatation_coordinates = MoveBaseSM.convert_pose_name_to_coordinates(self,location)
                #     print("Location coordinates",locatation_coordinates)
                #     distance = np.linalg.norm(np.array(robot_location) - np.array(locatation_coordinates))
                #     print("Distance :",distance)

                
                # Comment the for loop for testing
                # goal.destination_location = 'observation_table'
                # timeout = 15.0
                # premt_timeout = 45.0
                # rospy.loginfo('Sending action lib goal to move_base_server, ' +
                #           'destination : ' + goal.destination_location)
                # self.move_base_client.send_goal(goal)
                # self.move_base_client.wait_for_result(rospy.Duration.from_sec(int(timeout)))

                # while self.move_base_client.get_state() ==3:
                #     self.move_base_client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
                #     if self.move_base_client.get_state() == 3:
                #         print('get_result',self.move_base_client.get_result())
                #         print('get_state',self.move_base_client.get_state())
                #         try:
                #             timeout = 45.0
                #             rospy.loginfo('Sending action lib goal to perceive_plane_server')
                #             self.perceive_plane_client.send_goal(perceive_plane_goal)
                #             self.perceive_plane_client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
                #             preception_results = self.perceive_plane_client.get_result()
                #             # print("perception_results",preception_results)
                #             objects_list = preception_results.objects_list
                #             print("objects results ", objects_list)
                #             if 'cup' in objects_list:
                #                 print("Approach is correct and proceed with manipulation part")
                #             else:
                #                 print("Goto to next location and perceive again")
                #         except:
                #             pass
                #     else:
                #         print("Code failed ------------------")
                #     break

                # Loop through all locations ------ working code for perception + navigation
                # for location in natural_location:
                for location in ['observation_table']:
                    goal.destination_location = location
                    timeout = 15.0
                    rospy.loginfo('Sending action lib goal to move_base_server, ' +
                          'destination : ' + goal.destination_location)

                    self.move_base_client.send_goal(goal)
                    self.move_base_client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
                    while self.move_base_client.get_state() !=3:
                        self.move_base_client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
                        if self.move_base_client.get_state() == 3:
                            print('get_result',self.move_base_client.get_result())
                            print('get_state',self.move_base_client.get_state())
                            try:
                                timeout = 45.0
                                rospy.loginfo('Sending action lib goal to perceive_plane_server')
                                self.perceive_plane_client.send_goal(perceive_plane_goal)
                                self.perceive_plane_client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
                                preception_results = self.perceive_plane_client.get_result()
                                # print("perception_results",preception_results)
                                objects_list = preception_results.objects_list
                                print("objects results ", objects_list)
                                if 'cup' in objects_list:
                                    print("Approach is correct and proceed with manipulation part")
                                    # we need something to stop the loop and manipulation should work
                                    break
                                else:
                                    print("Goto to next location and perceive again")
                            except:
                                pass
                        else:
                            print("Code failed ------------------") # comment this line later
                            # pass 
                                   
                return FTSMTransitions.DONE


    def set_result(self, success, obj_location, relation,natural_location,possible_locations):
        result = FindObjectResult()
        result.success = success
        result.object_location = obj_location
        result.relation = relation
        result.natural_location = natural_location
        result.possible_locations = possible_locations
        return result

    def location_callback(self,msg):
        print("location callbacck message",msg.data)
        return msg.data
    
    # def get_distance(self,robot_coord,location_coord):
    #     '''
    #     robot_coord: [x,y,z]
    #     location_coord:[x,y,z]
    #     '''
    #     distance = np.linalg.norm(np.array(robot_coord) - np.array(location_coord))
    #     return distance

## We need a dictionary of locations along with their locations.
# example = {'location1':[x,y,z],'location2':[x,y,z],'location3':[x,y,z]}
# distances = {'rob_loc-loc1':distance}
# for location in example:
#     distance = get_distance(robot_location,location)
#     distances.append(distance)

