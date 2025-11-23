import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup

from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.srv import AddProblemPredicate
from plansys2_msgs.msg import Node as PDDLNode

from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

class PDDLActionServer(Node):

    def __init__(self):
        super().__init__('pddl_action_server')
        self.declare_parameter('action_name', 'default_action')
        self._action_name = self.get_parameter('action_name').get_parameter_value().string_value
        
        # This node's name must match the PDDL action name + '_node'
        # But the action server it provides must match the PDDL action name
        self.get_logger().info(f"Initializing action server for '{self._action_name}'")

        # --- MoveIt Client ---
        self.moveit_check_validity_client = self.create_client(
            GetStateValidity,
            '/check_state_validity')
        while not self.moveit_check_validity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MoveIt service not available, waiting...')

        # --- Problem Expert Client ---
        self.add_pred_client = self.create_client(
            AddProblemPredicate,
            '/problem_expert/add_problem_predicate')
        while not self.add_pred_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Problem Expert service not available, waiting...')
            
        # Simplified robot state
        self.current_joint_state = JointState()
        self.current_joint_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        self.current_joint_state.position = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.78]

        # The action server
        self._action_server = ActionServer(
            self,
            ExecutePlan,
            self._action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info(f"Action server '{self._action_name}' ready.")


    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received goal request for '{self._action_name}'")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info('Goal accepted')
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal cancelled')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing action: {self._action_name}")
        
        # --- This is the core TAMP logic ---
        action_succeeded = True
        
        # 1. If the action is 'pick', perform the MoveIt check
        if self._action_name == 'pick':
            # Get the object to pick from the PDDL arguments
            # goal_handle.request.arguments will be ['panda', 'obj_a', 'loc_a']
            target_object = goal_handle.request.arguments[1] 
            
            if target_object == 'obj_b':
                self.get_logger().warn(f"Checking geometric feasibility for {target_object}...")
                
                # --- Call MoveIt ---
                robot_state = RobotState()
                robot_state.joint_state = self.current_joint_state
                # Simulate a colliding pose for obj_b
                robot_state.joint_state.position = [0.1, 0.2, 0.1, -1.0, 0.0, 1.0, 0.0] 
                
                req = GetStateValidity.Request()
                req.robot_state = robot_state
                req.group_name = 'panda_arm'
                
                future = self.moveit_check_validity_client.call_async(req)
                await future
                
                response = future.result()
                is_valid = response.valid
                # ---------------------
                
                if not is_valid:
                    self.get_logger().error(f"MoveIt: Path for {target_object} is INFEASIBLE (Collision)!")
                    action_succeeded = False
                    
                    # --- This is the refinement! ---
                    self.get_logger().info("UPDATING KNOWLEDGE BASE...")
                    
                    # 1. Add (not (path_clear obj_b))
                    req_not_clear = AddProblemPredicate.Request()
                    pred_not_clear = PDDLNode()
                    pred_not_clear.name = 'path_clear'
                    pred_not_clear.parameters.append(target_object)
                    pred_not_clear.negate = True # <-- This makes it (not ...)
                    req_not_clear.predicate = pred_not_clear
                    await self.add_pred_client.call_async(req_not_clear)

                    # 2. Add (blocker obj_a obj_b)
                    req_blocker = AddProblemPredicate.Request()
                    pred_blocker = PDDLNode()
                    pred_blocker.name = 'blocker'
                    pred_blocker.parameters.append('obj_a') # Assumed blocker
                    pred_blocker.parameters.append(target_object)
                    req_blocker.predicate = pred_blocker
                    await self.add_pred_client.call_async(req_blocker)
                    
                    self.get_logger().info("KB updated.")
            
        # 2. For all other actions (or valid picks), just succeed
        if action_succeeded:
            self.get_logger().info(f"Simulating execution for '{self._action_name}'...")
            # Send feedback (optional)
            feedback_msg = ExecutePlan.Feedback()
            feedback_msg.action_execution_status.status = 1 # RUNNING
            feedback_msg.action_execution_status.message_status = "Executing..."
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(2) # Simulate work

        # 3. Send the final result
        result = ExecutePlan.Result()
        if action_succeeded:
            self.get_logger().info(f"Action '{self._action_name}' SUCCEEDED")
            goal_handle.succeed()
            result.action_execution_status.status = 2 # SUCCEEDED
            result.action_execution_status.message_status = "Action Succeeded"
        else:
            self.get_logger().error(f"Action '{self._action_name}' FAILED")
            goal_handle.abort()
            result.action_execution_status.status = 4 # FAILED
            result.action_execution_status.message_status = "Action Failed: MoveIt check infeasible"
            
        return result

    # --- Lifecycle Methods ---
    # PlanSys2 requires action servers to be lifecycle nodes.
    # We are skipping the full implementation for simplicity,
    # but a real node would use these.
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring node")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating node")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating node")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up node")
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    
    # We need a MultiThreadedExecutor to handle async service/action calls
    executor = rclpy.executors.MultiThreadedExecutor()
    action_server = PDDLActionServer()
    
    # We're not using the full lifecycle manager, just spinning the node
    # A full implementation would transition the node to 'active'
    executor.add_node(action_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()