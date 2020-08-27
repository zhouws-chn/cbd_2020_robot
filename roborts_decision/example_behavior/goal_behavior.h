#ifndef ROBORTS_DECISION_GOAL_BEHAVIOR_H
#define ROBORTS_DECISION_GOAL_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

namespace roborts_decision
{
class GoalBehavior
{
public:
  GoalBehavior(ChassisExecutor *&chassis_executor,
               GimbalExecutor *&gimbal_executor,
               Blackboard *&blackboard,
               const std::string &proto_file_path) : chassis_executor_(chassis_executor),
                                                     gimbal_executor_(gimbal_executor),
                                                     blackboard_(blackboard) {}

  void Run()
  {

    auto chassis_executor_state = ChassisUpdate();
    auto gimbal_executor_state = GimbalUpdate();
    static int car_flag = 0;
    car_flag++;
    if (car_flag % 20 == 0)
    {
      blackboard_->CarTurn(0);
      //gimbal mode change
      blackboard_->SetGimbalMode(1); //云台跟随底盘
    }
    int execution_mode_ = chassis_executor_->GetExcutionMode();
    std::cout << "goal ExcutionMode:" << execution_mode_ << std::endl;
    std::cout << "chassis_executor_state:" << (int)chassis_executor_state << std::endl;
    if (chassis_executor_state == BehaviorState::SUCCESS)
    {
      //   roborts_msgs::GimbalAngle gimbal_angle;
      // gimbal_angle.yaw_mode = false;
      // gimbal_angle.pitch_mode = false;
      // gimbal_angle.yaw_angle = 0.5;
      // gimbal_angle.pitch_angle = 0;
      // gimbal_executor_->Execute(gimbal_angle);
      bool arrive_goal_flag = true;
      blackboard_->SetSearchFlag(arrive_goal_flag);
    }
    if (execution_mode_ == 3)
    {

      chassis_executor_state = BehaviorState::IDLE;
      //static int flag=1;

      // cmd_vel_acc_pub_.publish(zero_twist_accel_);
    }
   
    auto robot_map_pose = blackboard_->GetRobotMapPose();
    last_position_ = blackboard_->GetEnemy();
    double x_diff = last_position_.pose.position.x - robot_map_pose.pose.position.x;
    double y_diff = last_position_.pose.position.y - robot_map_pose.pose.position.y;
    double yaw = std::atan2(y_diff, x_diff);
    auto orientation = tf::createQuaternionMsgFromYaw(yaw);

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position = last_position_.pose.position;
    goal.pose.orientation = orientation;
    chassis_executor_->Execute(goal);
  }

  void Cancel()
  {
    chassis_executor_->Cancel();
  }

  BehaviorState ChassisUpdate()
  {
    return chassis_executor_->Update();
  }

  BehaviorState GimbalUpdate()
  {
    return gimbal_executor_->Update();
  }

  ~GoalBehavior() = default;

private:
  //! executor
  ChassisExecutor *const chassis_executor_;
  GimbalExecutor *const gimbal_executor_;

  //! perception information
  Blackboard *const blackboard_;

  geometry_msgs::PoseStamped last_position_;

  //! planning goal
  geometry_msgs::PoseStamped planning_goal_;

  //! execution state of the executor (same with behavior state)
  BehaviorState execution_state_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_GOAL_BEHAVIOR_H
