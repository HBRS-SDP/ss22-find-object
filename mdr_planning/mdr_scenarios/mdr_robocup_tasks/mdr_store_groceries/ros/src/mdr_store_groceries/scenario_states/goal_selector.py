from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_rosplan_interface.planner_interface import PlannerInterface

class StoreGroceriesGoalSelector(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'store_groceries_goal_selector',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'store_groceries_goal_selector')
        self.number_of_shelves = kwargs.get('number_of_shelves', -1)
        self.planner_interface = PlannerInterface()

    def execute(self, userdata):
        # we check if the table and all shelves have been explored.
        # if they have been, we call the planner with the goal of storing
        # the groceries; otherwise, the goal is to scan them
        if not self.exploration_done():
            self.say('I need to check out the table and shelves')

            storing_goals = self.get_storing_groceries_goals()
            self.planner_interface.remove_plan_goals(storing_goals)

            scanning_goals = self.get_scanning_goals()
            self.planner_interface.add_plan_goals(scanning_goals)
        else:
            self.say('I will now store the groceries')

            scanning_goals = self.get_scanning_goals()
            self.planner_interface.remove_plan_goals(scanning_goals)

            storing_goals = self.get_storing_groceries_goals()
            self.planner_interface.add_plan_goals(storing_goals)

        plan_generated = self.planner_interface.plan()
        if plan_generated:
            plan_executed = self.planner_interface.start_plan_dispatch()

        if plan_executed:
            return 'succeeded'
        return 'failed'

    def exploration_done(self):
        '''Returns True if the table and all shelves (as many as expected based on
        self.number_of_shelves) have been explored; returns False otherwise.
        '''
        explored_table = False
        explored_shelf_count = 0

        explored_attributes = self.planner_interface.kb_interface.get_all_attributes('explored')
        for attribute in explored_attributes:
            for val in attribute.values:
                if val.key == 'plane':
                    if val.value.find('table') != -1:
                        explored_table = True
                    elif val.value.find('shelf') != -1:
                        explored_shelf_count += 1

        return explored_table and explored_shelf_count == self.number_of_shelves

    def get_scanning_goals(self):
        '''Returns a list of planning goals for scanning the table and shelves.
        '''
        goals = [('explored', [('plane', 'table')])]
        for i in range(self.number_of_shelves):
            goals.append(('explored', [('plane', 'shelf{0}'.format(i+1))]))
        return goals

    def get_storing_groceries_goals(self):
        '''Returns a list containing a single planning goal -
        a "groceries_stored" predicate.
        '''
        goals = [('groceries_stored')]
        return goals
