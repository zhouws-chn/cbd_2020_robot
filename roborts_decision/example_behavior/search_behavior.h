#ifndef ROBORTS_DECISION_SEARCH_BEHAVIOR_H
#define ROBORTS_DECISION_SEARCH_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision
{
class SearchBehavior
{
public:
  SearchBehavior(ChassisExecutor *&chassis_executor,
                 GimbalExecutor *&gimbal_executor,
                 Blackboard *&blackboard,
                 const std::string &proto_file_path) : chassis_executor_(chassis_executor),
                                                       gimbal_executor_(gimbal_executor),
                                                       blackboard_(blackboard)
  {

    ros::NodeHandle nh;
    cmd_vel_acc_pub_ = nh.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 100);

    last_position_.header.frame_id = "map";
    last_position_.pose.orientation.x = 0;
    last_position_.pose.orientation.y = 0;
    last_position_.pose.orientation.z = 0;
    last_position_.pose.orientation.w = 1;

    last_position_.pose.position.x = 0;
    last_position_.pose.position.y = 0;
    last_position_.pose.position.z = 0;

    search_index_ = 0;
    search_count_ = 0;

    if (!LoadParam(proto_file_path))
    {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
  }

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
    double yaw;
    double x_diff;
    double y_diff;

    enemy_position_ = blackboard_->GetEnemy();

    last_position_.pose.position.x = enemy_position_.pose.position.x;
    last_position_.pose.position.y = enemy_position_.pose.position.y;
    std::cout << "search_enemy_x:" << last_position_.pose.position.x << std::endl;
    std::cout << "search_enemy_y:" << last_position_.pose.position.y << std::endl;
    // blackboard_->CarTurn(0);
    // ROS_INFO(" blackbord CarTurn false");
    // int execution_mode_ = chassis_executor_->GetExcutionMode();
    // std::cout << "search ExcutionMode:" << execution_mode_ << std::endl;

    // if (execution_mode_ == 3)
    // {
    //   chassis_executor_state = BehaviorState::IDLE;
    //   //static int flag=1;

    //   cmd_vel_acc_pub_.publish(zero_twist_accel_);
    // }

    //gimbal mode change
   
    // gimbal_angle.yaw_mode = false;
    // gimbal_angle.pitch_mode = false;
    // gimbal_angle.yaw_angle = 0.5;
    // gimbal_angle.pitch_angle = 0;
    // gimbal_executor_->Execute(gimbal_angle);
    auto robot_map_pose = blackboard_->GetRobotMapPose();
    if (search_count_ == 5)
    {
      std::cout << "search_count_ == 5" << std::endl;
      x_diff = last_position_.pose.position.x - robot_map_pose.pose.position.x;
      y_diff = last_position_.pose.position.y - robot_map_pose.pose.position.y;

      auto last_x = last_position_.pose.position.x;
      auto last_y = last_position_.pose.position.y;

      if (last_x < 4.2 && last_y < 2.75)
      {
        search_region_ = search_region_1_;
      }
      else if (last_x > 4.2 && last_y < 2.75)
      {
        search_region_ = search_region_2_;
      }
      else if (last_x < 4.2 && last_y > 2.75)
      {
        search_region_ = search_region_3_;
      }
      else
      {
        search_region_ = search_region_4_;
      }

      double search_min_dist = 99999;
      for (unsigned int i = 0; i < search_region_.size(); ++i)
      {
        auto dist_sq = std::pow(search_region_[i].pose.position.x - last_x, 2) + std::pow(search_region_[i].pose.position.y - last_y, 2);

        if (dist_sq < search_min_dist)
        {
          search_min_dist = dist_sq;
          search_index_ = i;
        }
      }

      // yaw = std::atan2(y_diff, x_diff);

      // auto orientation = tf::createQuaternionMsgFromYaw(yaw);

      // geometry_msgs::PoseStamped goal;
      // goal.header.frame_id = "map";
      // goal.header.stamp = ros::Time::now();
      // goal.pose.position = last_position_.pose.position;
      // goal.pose.orientation = orientation;
      // chassis_executor_->Execute(goal);
      // std::cout << "search_count_" << search_count_ << std::endl;
      // search_count_--;
      auto search_goal = search_region_[(search_index_++)];
      chassis_executor_->Execute(search_goal);
      search_index_ = (unsigned int)(search_index_ % search_region_.size());
      search_count_--;
      std::cout << "search_count_" << search_count_ << std::endl;
    }
    else if (search_count_ > 0)
    {
      auto search_goal = search_region_[(search_index_++)];
      chassis_executor_->Execute(search_goal);
      search_index_ = (unsigned int)(search_index_ % search_region_.size());
      search_count_--;
      std::cout << "search_count_" << search_count_ << std::endl;
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

    // may have more efficient way to search a region(enemy where disappear)
    search_region_.resize((unsigned int)(decision_config.search_region_1().size()));
    for (int i = 0; i != decision_config.search_region_1().size(); i++)
    {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_1(i).x();
      search_point.pose.position.y = decision_config.search_region_1(i).y();
      search_point.pose.position.z = decision_config.search_region_1(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_1(i).roll(),
                                                                decision_config.search_region_1(i).pitch(),
                                                                decision_config.search_region_1(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_1_.push_back(search_point);
    }

    for (int i = 0; i != decision_config.search_region_2().size(); i++)
    {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_2(i).x();
      search_point.pose.position.y = decision_config.search_region_2(i).y();
      search_point.pose.position.z = decision_config.search_region_2(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_2(i).roll(),
                                                                decision_config.search_region_2(i).pitch(),
                                                                decision_config.search_region_2(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_2_.push_back(search_point);
    }

    for (int i = 0; i != decision_config.search_region_3().size(); i++)
    {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_3(i).x();
      search_point.pose.position.y = decision_config.search_region_3(i).y();
      search_point.pose.position.z = decision_config.search_region_3(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_3(i).roll(),
                                                                decision_config.search_region_3(i).pitch(),
                                                                decision_config.search_region_3(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_3_.push_back(search_point);
    }

    for (int i = 0; i != decision_config.search_region_4().size(); i++)
    {
      geometry_msgs::PoseStamped search_point;
      search_point.header.frame_id = "map";
      search_point.pose.position.x = decision_config.search_region_4(i).x();
      search_point.pose.position.y = decision_config.search_region_4(i).y();
      search_point.pose.position.z = decision_config.search_region_4(i).z();

      auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_4(i).roll(),
                                                                decision_config.search_region_4(i).pitch(),
                                                                decision_config.search_region_4(i).yaw());
      search_point.pose.orientation = quaternion;
      search_region_4_.push_back(search_point);
    }
    return true;
  }

  void SetLastPosition(geometry_msgs::PoseStamped last_position)
  {
    last_position_ = last_position;
    search_count_ = 5;
  }

  ~SearchBehavior() = default;

private:
  //! executor
  ChassisExecutor *const chassis_executor_;
  GimbalExecutor *const gimbal_executor_;

  //! perception information
  Blackboard *const blackboard_;

  //! chase goal
  geometry_msgs::PoseStamped last_position_;

  geometry_msgs::PoseStamped enemy_position_;

  //! search buffer
  std::vector<geometry_msgs::PoseStamped> search_region_1_;
  std::vector<geometry_msgs::PoseStamped> search_region_2_;
  std::vector<geometry_msgs::PoseStamped> search_region_3_;
  std::vector<geometry_msgs::PoseStamped> search_region_4_;
  std::vector<geometry_msgs::PoseStamped> search_region_;
  unsigned int search_count_;
  unsigned int search_index_;

  roborts_msgs::TwistAccel zero_twist_accel_;

  //! velocity with accel publisher in ROS
  ros::Publisher cmd_vel_acc_pub_;

  //  geometry_msgs::Twist whirl_vel_;
  roborts_msgs::TwistAccel whirl_vel_;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_SEARCH_BEHAVIOR_H
