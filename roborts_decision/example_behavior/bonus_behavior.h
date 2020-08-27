#ifndef ROBORTS_DECISION_BONUSBEHAVIOR_H
#define ROBORTS_DECISION_BONUSBEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision
{
class BonusBehavior
{
public:
  BonusBehavior(ChassisExecutor *&chassis_executor,
                GimbalExecutor *&gimbal_executor,
                Blackboard *&blackboard,
                const std::string &proto_file_path) : chassis_executor_(chassis_executor),
                                                      gimbal_executor_(gimbal_executor),
                                                      blackboard_(blackboard)
  {
    ros::NodeHandle nh;
    cmd_vel_acc_pub_ = nh.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 100);

    boot_position_.header.frame_id = "map";
    boot_position_.pose.orientation.x = 0;
    boot_position_.pose.orientation.y = 0;
    boot_position_.pose.orientation.z = 0;
    boot_position_.pose.orientation.w = 1;

    boot_position_.pose.position.x = 0;
    boot_position_.pose.position.y = 0;
    boot_position_.pose.position.z = 0;

    if (!LoadParam(proto_file_path))
    {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
  }

  void Run()
  {
    //  blackboard_->CarTurn(0);
    // ROS_INFO(" blackbord CarTurn false");
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
    std::cout << "bonus ExcutionMode:" << execution_mode_ << std::endl;

    if (execution_mode_ == 3)
    {
      chassis_executor_state = BehaviorState::IDLE;
      //static int flag=1;

      cmd_vel_acc_pub_.publish(zero_twist_accel_);
    }

   

    if (chassis_executor_state != BehaviorState::RUNNING)
    {
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      auto dx = boot_position_.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = boot_position_.pose.position.y - robot_map_pose.pose.position.y;

      auto boot_yaw = tf::getYaw(boot_position_.pose.orientation); //Helper function for getting yaw from a Quaternion.
      auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

      tf::Quaternion rot1, rot2;                                    //The Quaternion implements quaternion to perform linear algebra rotations in combination with Matrix3x3, Vector3 and Transform.
                                                                    //四元数与矩阵x3x3、向量3和变换相结合，实现四元数的线性代数旋转 tf::quaternionMsgToTF函数取/tf中四元数消息转换为四元数
      tf::quaternionMsgToTF(boot_position_.pose.orientation, rot1); //使用geometry_msgs::PoseStampedPtr pose构造一个对象  tf::quaternionMsgToTF(pose->pose.orientation,q);   //把geomsg形式的四元数转化为tf形式，得到q
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2); //
      auto d_yaw = rot1.angleShortestPath(rot2);                    //返回这个四元数与另一个四元数之间沿最短路径的夹角

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5)
      {                                             //pow(dx, 2)计算数 dx 的 2 次幂，
                                                    //boot_position_与robot_map_pose的距离>0.2.d_yaw > 0.5? 0.2和0.5意思?
        chassis_executor_->Execute(boot_position_); //chassis_executor_没有定义，如何使用的。 boot_position_.header.frame_id = "map"的一系列定义
                                                    //boot_position_是PoseStamped类型
      }
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

    boot_position_.header.frame_id = "map";
    boot_position_.pose.position.x = decision_config.bonus_bot().blue().x();
    boot_position_.pose.position.z = decision_config.bonus_bot().blue().z();
    boot_position_.pose.position.y = decision_config.bonus_bot().blue().y();

    auto master_quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.bonus_bot().blue().roll(),
                                                                     decision_config.bonus_bot().blue().pitch(),
                                                                     decision_config.bonus_bot().blue().yaw());
    boot_position_.pose.orientation = master_quaternion;

    //222
    return true;
  }

  ~BonusBehavior() = default;

private:
  //! executor
  ChassisExecutor *const chassis_executor_;
  GimbalExecutor *const gimbal_executor_;

  //! perception information
  Blackboard *const blackboard_;

  //! boot position
  geometry_msgs::PoseStamped boot_position_;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

  roborts_msgs::TwistAccel zero_twist_accel_;

  //! velocity with accel publisher in ROS
  ros::Publisher cmd_vel_acc_pub_;

  //  geometry_msgs::Twist whirl_vel_;
  roborts_msgs::TwistAccel whirl_vel_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_BONUSBEHAVIOR_H
