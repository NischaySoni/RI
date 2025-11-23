import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from plansys2_msgs.srv import GetPlan
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import Node as PDDLNode

class Replanner(Node):
    def __init__(self):
        super().__init__('replanner_node')
        self.get_logger().info('Replanner node is active.')
        
        # This node will orchestrate the whole process
        self.problem_expert_client = self.create_client(
            # We don't need a client, we just call the services directly
            # This is simpler for Python
            'plansys2_msgs/srv/AddProblemGoal',
            '/problem_expert/add_problem_goal')
        
        self.planner_client = self.create_client(
            GetPlan,
            '/planner/get_plan')
        
        self.executor_client = ActionClient(
            self,
            ExecutePlan,
            '/execute_plan')

        # Wait for all services
        while not self.problem_expert_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Problem Expert service not available, waiting...')
        while not self.planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Planner service not available, waiting...')
        while not self.executor_client.wait_for_action_server(timeout_sec=1.0):
            self.get_logger().info('Executor action server not available, waiting...')
        
        self.get_logger().info('All PlanSys2 services are ready.')

    def set_goal(self):
        self.get_logger().info("Setting goal: (obj_at obj_b loc_target)")
        
        # PDDL goal is a tree structure
        goal_pred = PDDLNode()
        goal_pred.name = 'obj_at'
        goal_pred.parameters.append('obj_b')
        goal_pred.parameters.append('loc_target')
        
        request = self.problem_expert_client.srv_type.Request()
        request.goal.nodes.append(goal_pred)
        
        future = self.problem_expert_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info("Goal set successfully.")
        else:
            self.get_logger().error(f"Failed to set goal: {future.result().error_info}")

    async def get_and_run_plan(self):
        self.get_logger().info("Requesting a plan...")
        
        # 1. Get Domain and Problem from experts (not strictly needed, but good)
        # For this example, we'll just call the planner
        
        plan_req = GetPlan.Request()
        # We don't need to specify domain/problem path, PlanSys2 knows
        
        plan_future = self.planner_client.call_async(plan_req)
        rclpy.spin_until_future_complete(self, plan_future)
        
        plan_response = plan_future.result()
        
        if not plan_response.success or not plan_response.plan:
            self.get_logger().error(f"Could not find a plan: {plan_response.error_info}")
            return False

        self.get_logger().info("Plan found! Executing...")
        
        # 2. Execute the plan
        exec_goal = ExecutePlan.Goal()
        exec_goal.plan = plan_response.plan
        
        exec_future = self.executor_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, exec_future)
        goal_handle = exec_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Plan execution was rejected.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        
        if result.status == 2: # SUCCEEDED
            self.get_logger().info("PLAN SUCCEEDED!")
            return True
        else:
            self.get_logger().error("PLAN FAILED!")
            self.get_logger().error(f"Failure details: {result.result.action_execution_status}")
            return False

async def main(args=None):
    rclpy.init(args=args)
    replanner_node = Replanner()
    
    # 1. Set the initial goal
    replanner_node.set_goal()
    
    # 2. Run the first attempt
    # This will fail because monitor_motion.py will find the collision
    success = await replanner_node.get_and_run_plan()
    
    if not success:
        replanner_node.get_logger().info("Replanning... (KB was updated by action server)")
        
        # 3. Run the second attempt
        # The 'pick' action server already updated the KB.
        # This new plan will be the correct, longer one.
        success = await replanner_node.get_and_run_plan()
    
    if success:
        replanner_node.get_logger().info("Mission Complete!")
    else:
        replanner_node.get_logger().error("Mission Failed.")
        
    replanner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # rclpy.spin() is not needed, we're using async main
    import asyncio
    asyncio.run(main())