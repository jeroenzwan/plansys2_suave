#!/usr/bin/python3

import rclpy
from rclpy.node import Node as ROS2Node
from plansys2_msgs.msg import Param, Node, Tree
from plansys2_msgs.srv import AffectParam, AffectNode, AddProblemGoal

class ProblemGenerator(ROS2Node):

    def __init__(self):
        super().__init__('problem_generator')
        self.instance_cli = self.create_client(AffectParam,
                '/problem_expert/add_problem_instance')
        while not self.instance_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('instance service not available, waiting again...')

        self.predicate_cli = self.create_client(AffectNode,
                '/problem_expert/add_problem_predicate')
        while not self.predicate_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('predicate service not available, waiting again...')
            
        self.function_cli = self.create_client(AffectNode,
                '/problem_expert/add_problem_function')
        while not self.function_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('function service not available, waiting again...')

        self.goal_cli = self.create_client(AddProblemGoal,
                '/problem_expert/add_problem_goal')
        while not self.goal_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('goal service not available, waiting again...')

        self.remove_predicate_cli = self.create_client(AffectNode,
                '/problem_expert/remove_problem_predicate')
        while not self.remove_predicate_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('remove predicate service not available, waiting again...')

    def call_instance_service(self, _name, _type):
        msg = Param()
        msg.name = _name
        msg.type = _type

        req = AffectParam.Request()
        req.param = msg

        future = self.instance_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future

    def call_predicate_service(self, _name, _params):
        msg = Node()
        msg.node_type = 5
        msg.name = _name

        param_list = []
        for _param in _params:
            param = Param()
            param.name = _param[0]
            param.type = _param[1]
            param_list.append(param)

        msg.parameters = param_list

        req = AffectNode.Request()
        req.node = msg

        future = self.predicate_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future

    def call_function_service(self, _name, _params, _value):
        msg = Node()
        msg.node_type = 6
        msg.name = _name
        msg.value = _value

        param_list = []
        for _param in _params:
            param = Param()
            param.name = _param[0]
            param.type = _param[1]
            param_list.append(param)

        msg.parameters = param_list

        req = AffectNode.Request()
        req.node = msg

        future = self.function_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future

    def call_goal_service(self, _pred, _param, _type):
        msg = Tree()
        node = Node()
        node.node_type = _type
        node.name = _pred

        param = Param()
        param.name = _param[0]
        param.type = _param[1]

        node.parameters = [param]

        msg.nodes = [node]

        req = AddProblemGoal.Request()
        req.tree = msg

        future = self.goal_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future

    def call_remove_predicate_service(self, _name, _params):
        msg = Node()
        msg.node_type = 5
        msg.name = _name

        param_list = []
        for _param in _params:
            param = Param()
            param.name = _param[0]
            param.type = _param[1]
            param_list.append(param)

        msg.parameters = param_list

        req = AffectNode.Request()
        req.node = msg

        future = self.remove_predicate_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future

    def populate_instances(self):
        self.call_instance_service('bluerov', 'uuv')

        self.call_instance_service('pl1', 'pipeline')

        self.call_instance_service('f_action', 'function')

        self.call_instance_service('fd_recharge', 'functiondesign')

        self.call_instance_service('fd_search_pipeline', 'functiondesign')

        self.call_instance_service('fd_follow_pipeline', 'functiondesign')

        self.call_instance_service('f_maintain_motion', 'function')

        self.call_instance_service('f_go_to_recharge_waypoints', 'function')

        self.call_instance_service('f_search_pipeline_waypoints', 'function')

        self.call_instance_service('f_follow_pipeline_waypoints', 'function')

        self.call_instance_service('fd_all_thrusters', 'functiondesign')

        self.call_instance_service('fd_recover', 'functiondesign')

        self.call_instance_service('fd_generate_recharge_wp', 'functiondesign')

        self.call_instance_service('fd_spiral_low', 'functiondesign')

        self.call_instance_service('fd_spiral_medium', 'functiondesign')

        self.call_instance_service('fd_spiral_high', 'functiondesign')

        self.call_instance_service('fd_generate_follow_wp', 'functiondesign')

        self.call_predicate_service('pipeline_not_found', [['pl1','pipeline']])

        self.call_predicate_service('pipeline_not_inspected', [['pl1','pipeline']])

        self.call_predicate_service('recharge_available',
                [['fd_recharge','functiondesign']])

        self.call_predicate_service('search_available',
                [['fd_search_pipeline','functiondesign']])

        self.call_predicate_service('follow_available',
                [['fd_follow_pipeline','functiondesign']])

        self.call_predicate_service('fd_available',
                [['fd_all_thrusters','functiondesign'],
                    ['f_maintain_motion','function']])

        self.call_predicate_service('fd_available',
                [['fd_recover','functiondesign'],
                    ['f_maintain_motion','function']])

        self.call_predicate_service('fd_available',
                [['fd_generate_recharge_wp','functiondesign'],
                    ['f_go_to_recharge_waypoints','function']])

        self.call_predicate_service('fd_available',
                [['fd_spiral_low','functiondesign'],
                    ['f_search_pipeline_waypoints','function']])

        self.call_predicate_service('fd_available',
                [['fd_spiral_medium','functiondesign'],
                    ['f_search_pipeline_waypoints','function']])

        self.call_predicate_service('fd_available',
                [['fd_spiral_high','functiondesign'],
                    ['f_search_pipeline_waypoints','function']])

        self.call_predicate_service('fd_available',
                [['fd_generate_follow_wp','functiondesign'],
                    ['f_follow_pipeline_waypoints','function']])

        self.call_predicate_service('requires_f',
                [['fd_recharge','functiondesign'],
                    ['f_go_to_recharge_waypoints','function'],
                    ['f_maintain_motion','function']])

        self.call_predicate_service('requires_f',
                [['fd_search_pipeline','functiondesign'],
                    ['f_search_pipeline_waypoints','function'],
                    ['f_maintain_motion','function']])

        self.call_predicate_service('requires_f',
                [['fd_follow_pipeline','functiondesign'],
                    ['f_follow_pipeline_waypoints','function'],
                    ['f_maintain_motion','function']])
        
        # self.call_predicate_service('function_not_grounded',
                # [['f_maintain_motion','function']])

        # self.call_predicate_service('function_not_grounded',
                # [['f_go_to_recharge_waypoints','function']])

        # self.call_predicate_service('function_not_grounded',
                # [['f_search_pipeline_waypoints','function']])

        # self.call_predicate_service('function_not_grounded',
                # [['f_follow_pipeline_waypoints','function']])

        self.call_predicate_service('not_occupied',
                [['bluerov','uuv']])

        self.call_function_service('speed',
                [['fd_all_thrusters','functiondesign']], 1.)

        self.call_function_service('speed',
                [['fd_recover','functiondesign']], 100.)

        self.call_function_service('speed',
                [['fd_generate_recharge_wp','functiondesign']], 5.)

        self.call_function_service('speed',
                [['fd_spiral_low','functiondesign']], 20.)

        self.call_function_service('speed',
                [['fd_spiral_medium','functiondesign']], 10.)

        self.call_function_service('speed',
                [['fd_spiral_high','functiondesign']], 5.)

        self.call_function_service('speed',
                [['fd_generate_follow_wp','functiondesign']], 5.)

        self.call_function_service('efficiency',
                [['fd_all_thrusters','functiondesign']], 1.)

        self.call_function_service('efficiency',
                [['fd_generate_follow_wp','functiondesign']], 5.)

        self.call_function_service('efficiency',
                [['fd_recover','functiondesign']], 5.)

        self.call_function_service('efficiency',
                [['fd_spiral_low','functiondesign']], 30.)

        self.call_function_service('efficiency',
                [['fd_spiral_medium','functiondesign']], 35.)

        self.call_function_service('efficiency',
                [['fd_spiral_high','functiondesign']], 40.)

        self.call_function_service('efficiency',
                [['fd_generate_follow_wp','functiondesign']], 30.)

        self.call_function_service('battery_level',
                [['bluerov','uuv']], 40.)

        self.call_goal_service('pipeline_inspected', ['pl1', 'pipeline'], 5)

    def remove_predicates(self):
        self.call_remove_predicate_service('available_fd', [['fd2','functiondesign']])

def main(args=None):
    rclpy.init(args=args)

    problem_generator = ProblemGenerator()
    problem_generator.populate_instances()
    problem_generator.remove_predicates()

    rclpy.spin(problem_generator)

    problem_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    # (:init
        # (deployed bluerov)
        # (localized bluerov)
        # (fd_selected fd2)
        # (pipeline_found pl1)
        # (pipeline_not_inspected pl1)
        # (available_fd fd2)
        # (available_fd fd3)
        # (= (speed fd2) 5)
        # (= (speed fd3) 20)
        # (= (accuracy fd2) 20)
        # (= (accuracy fd3) 5)
    # )

    # (:goal (and
        # (pipeline_inspected pl1))
