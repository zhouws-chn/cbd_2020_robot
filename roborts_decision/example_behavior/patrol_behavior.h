#ifndef ROBORTS_DECISION_PATROL_BEHAVIOR_H
#define ROBORTS_DECISION_PATROL_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

//巡逻思路,四个角落各一个点.四个点循环移动,就是巡逻
namespace roborts_decision
{
class PatrolBehavior
{
public:
  PatrolBehavior(ChassisExecutor *&chassis_executor,
                 GimbalExecutor *&gimbal_executor,
                 Blackboard *&blackboard,
                 const std::string &proto_file_path) : chassis_executor_(chassis_executor),
                                                       gimbal_executor_(gimbal_executor),
                                                       blackboard_(blackboard)
  {
    
    patrol_count_ = 0;
    point_size_ = 0;

    if (!LoadParam(proto_file_path))
    {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
  }

  void Run()
  {
    //  blackboard_->CarTurn(0);
    // ROS_INFO(" blackbord CarTurn false");
    static int number_patrol = 0;
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
    std::cout << "patrol ExcutionMode:" << execution_mode_ << std::endl;

    if (execution_mode_ == 3)
    {
      chassis_executor_state = BehaviorState::IDLE;
     
    }
    number_patrol = (number_patrol + 1) % 10;
    //gimbal mode change
    if (number_patrol == 0)
    {
      roborts_msgs::GimbalAngle gimbal_angle;
      gimbal_angle.yaw_mode = false;
      gimbal_angle.pitch_mode = false;
      gimbal_angle.yaw_angle = 0;
      gimbal_angle.pitch_angle = 0;

    
      gimbal_executor_->Execute(gimbal_angle);
    }

    if (chassis_executor_state != BehaviorState::RUNNING)
    {

      if (patrol_goals_.empty())
      {
        ROS_ERROR("patrol goal is empty");
        return;
      }
      std::cout << "send goal" << std::endl;
      chassis_executor_->Execute(patrol_goals_[patrol_count_]);
      patrol_count_ = ++patrol_count_ % point_size_;
      ROS_INFO("patrol_executor_ Execute");
    }
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
  bool LoadParam(const std::string &proto_file_path)
  {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config))
    {
      return false;
    }

    point_size_ = (unsigned int)(decision_config.point().size());
    patrol_goals_.resize(point_size_);
    for (int i = 0; i != point_size_; i++)
    {
      patrol_goals_[i].header.frame_id = "map";
      patrol_goals_[i].pose.position.x = decision_config.point(i).x();
      patrol_goals_[i].pose.position.y = decision_config.point(i).y();
      patrol_goals_[i].pose.position.z = decision_config.point(i).z();

      tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.point(i).roll(),
                                                              decision_config.point(i).pitch(),
                                                              decision_config.point(i).yaw());
      patrol_goals_[i].pose.orientation.x = quaternion.x();
      patrol_goals_[i].pose.orientation.y = quaternion.y();
      patrol_goals_[i].pose.orientation.z = quaternion.z();
      patrol_goals_[i].pose.orientation.w = quaternion.w();
    }

    return true;
  }

  ~PatrolBehavior() = default;

private:
  //! executor
  ChassisExecutor *const chassis_executor_;
  GimbalExecutor *const gimbal_executor_;

  //! perception information
  Blackboard *const blackboard_;

  //! patrol buffer
  std::vector<geometry_msgs::PoseStamped> patrol_goals_;
  unsigned int patrol_count_;
  unsigned int point_size_;

  // roborts_msgs::TwistAccel zero_twist_accel_;

  // //! velocity with accel publisher in ROS
  // ros::Publisher cmd_vel_acc_pub_;

  // //  geometry_msgs::Twist whirl_vel_;
  // roborts_msgs::TwistAccel whirl_vel_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_PATROL_BEHAVIOR_H
