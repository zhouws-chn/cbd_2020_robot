/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
//
#include <tf/transform_broadcaster.h>

#include "roborts_msgs/ArmorDetectionAction.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"
#include "roborts_msgs/MucInterface.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/SupplierStatus.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/BonusStatus.h"
#include "roborts_msgs/RobotBonus.h"
#include "roborts_msgs/GimbalMode.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/GimbalAngle.h"
//camke中需要添加这个Teammate.h
#include "roborts_msgs/Teammate.h"
#include "roborts_msgs/GameSurvivor.h"

using namespace std;

namespace roborts_decision
{

class Blackboard
{
public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path) : remain_hp_(2000), chase_enemy_(false), can_supply(true), go_bonus(false),
                                                            bonus_(false), blue_bonus_(0), red_bonus_(0), supplier_can_status(2), supply_bullet_(true),

                                                            armor_detection_actionlib_client_("armor_detection_node_action", true)
  {
    armor_hurt_time = ros::Time::now();
    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    std::string map_path = ros::package::getPath("roborts_costmap") +
                           "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    // Enemy fake pose
    ros::NodeHandle rviz_nh("/move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);

    //supply
    ros::NodeHandle nh;
    supply_sub_ = nh.subscribe("muc_car_status", 20, &Blackboard::SupplyCallback, this);

    ros_robot_damage_sub_ = nh.subscribe("robot_damage", 30, &Blackboard::RobotDamageCallback, this);

    ros_robot_status_sub_ = nh.subscribe("robot_status", 30, &Blackboard::RobotStatusCallback, this);

    ros_supplier_status_sub_ = nh.subscribe("field_supplier_status", 30, &Blackboard::SupplierStatusCallback, this);

    ros_game_status_sub_ = nh.subscribe("game_status", 30, &Blackboard::GameStatusCallback, this);

    ros_bonus_status_sub_ = nh.subscribe("field_bonus_status", 30, &Blackboard::BonusStatusCallback, this);

    ros_robot_bonus_sub_ = nh.subscribe("robot_bonus", 30, &Blackboard::RobotBonusCallback, this);

    ros_game_survivor_sub_ = nh.subscribe("game_survivor", 30, &Blackboard::GameSurvivorCallback, this);

    gimbal_client_ = nh.serviceClient<roborts_msgs::GimbalMode>("set_gimbal_mode");

    shoot_client_ = nh.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");

    ros_projectile_supply_pub_ = nh.advertise<roborts_msgs::ProjectileSupply>("projectile_supply", 30);
    cat_walk_ = nh.advertise<std_msgs::Int8>("cmd_cat_step", 30);

    ROS_INFO(" blackbord CarTurn flase");
    //用于多机通信
    ros_teammate_msg_sub_ = nh.subscribe("team_mate_no1", 30, &Blackboard::TeammateCallback, this);
    ros_teammate_msg_pub_ = nh.advertise<roborts_msgs::Teammate>("team_mate_no2", 30);

    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

    ROS_INFO("Simulate status:%d", decision_config.simulate());
    if (!decision_config.simulate())
    {

      armor_detection_actionlib_client_.waitForServer();

      ROS_INFO("Armor detection module has been connected!");

      //
      enemy_tf_.header.frame_id = "map";
      enemy_tf_.child_frame_id = "enemy";

      armor_detection_goal_.command = 1;
      armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                                 actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                                 boost::bind(&Blackboard::ArmorDetectionFeedbackCallback, this, _1));
    }
  }

  ~Blackboard() = default;

  //111
  void GameSurvivorCallback(const roborts_msgs::GameSurvivor &game_survivor)
  {
    robot_game_survivor = game_survivor;
  }

  // Enemy
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr &feedback)
  {
    static int undetected_count_ = 0;
    static int undetected_count_search = 0;
    static float enemy_pose_position_x = 0.0;
    static float enemy_pose_position_y = 0.0;

    if (feedback->detected)
    {
      enemy_detected_ = true;
      ROS_INFO("Find Enemy!");

      tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
      geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
      camera_pose_msg = feedback->enemy_pos;
      camera_pose_msg.pose.position.y = 0 - camera_pose_msg.pose.position.y;
      double distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
                                  camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
      double yaw = atan(camera_pose_msg.pose.position.y / camera_pose_msg.pose.position.x);
      ROS_INFO("relative_x_:%f  relative_y_:%f", camera_pose_msg.pose.position.x, camera_pose_msg.pose.position.y);
      //camera_pose_msg.pose.position.z=camera_pose_msg.pose.position.z;
      relative_x_ = camera_pose_msg.pose.position.x;
      relative_y_ = camera_pose_msg.pose.position.y;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,
                                                              0,
                                                              yaw);
      //camera_pose_msg.pose.position.x = camera_pose_msg.pose.position.x;
      // camera_pose_msg.pose.position.x = 0.0;
      camera_pose_msg.pose.orientation.w = quaternion.w();
      camera_pose_msg.pose.orientation.x = quaternion.x();
      camera_pose_msg.pose.orientation.y = quaternion.y();
      camera_pose_msg.pose.orientation.z = quaternion.z();
      poseStampedMsgToTF(camera_pose_msg, tf_pose);
      //
      tf_pose.frame_id_ = "camera";
      tf_pose.stamp_ = ros::Time(0);
      try
      {
        tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
        tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

        //
        ros::Time current_time = ros::Time::now();
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
        enemy_tf_.header.stamp = current_time;
        enemy_tf_.transform.rotation = q;
        enemy_tf_.transform.translation.x = global_pose_msg.pose.position.x;
        enemy_tf_.transform.translation.y = global_pose_msg.pose.position.y;
        enemy_tf_.transform.translation.z = 0.0;
        tf_broadcaster_.sendTransform(enemy_tf_);

        enemy_pose_position_x = global_pose_msg.pose.position.x;
        enemy_pose_position_y = global_pose_msg.pose.position.y;

        if (GetDistance(global_pose_msg, enemy_pose_) > 0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2)
        {
          enemy_pose_ = global_pose_msg;
        }

        undetected_count_ = 5;
        undetected_count_search = 300;
        arrive_goal_flag_ = false;
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("tf error when transform enemy pose from camera to map");
      }
    }
    else
    {

      if (undetected_count_ > 0)
      {
        undetected_count_--;
        enemy_detected_ = true;

        ros::Time current_time = ros::Time::now();
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
        enemy_tf_.header.stamp = current_time;
        enemy_tf_.transform.rotation = q;
        enemy_tf_.transform.translation.x = enemy_pose_position_x;
        enemy_tf_.transform.translation.y = enemy_pose_position_y;
        enemy_tf_.transform.translation.z = 0.0;
        tf_broadcaster_.sendTransform(enemy_tf_);
      }
      else
      {
        ROS_INFO("no find emeny");
        enemy_detected_ = false;
      }
      undetected_count_search--;
      undetected_count_search_ = undetected_count_search;
    }
  }

  geometry_msgs::PoseStamped GetEnemy() const
  {
    return enemy_pose_;
  }

  //获得机器人颜色
  auto robot_color()
  {
    return robot_color_;
  }

  //敌方是否检测到
  bool IsEnemyDetected()
  {
    ROS_INFO("%s: %d", __FUNCTION__, (int)enemy_detected_);
    return enemy_detected_;
  }

  //是否需要补弹
  bool IsNeedSupply()
  {
    //在补弹区域判断
    bool supply_position = IsSupplyArea();
    ROS_INFO("%s: %d", __FUNCTION__, (int)supply_bullet_);

    //能否补弹的标识符
    ROS_INFO("supplier_can_status: %d", supplier_can_status);
    //return supply_bullet_ && (supplier_can_status > 0);
    if (supply_bullet_ && (supplier_can_status > 0))
    {
      if (!supply_position)
        return true;
      else
      {
        return false;
      }
    }
    return false;
  }

  /*新思路,加上异常处理
收到裁判系统信号,补到弹,待5秒离开
没有收到信号,等8秒必须离开.异常处理
*/

  // supply stayIsStaySupply
  bool IsStaySupply()
  {
    static int supplyDelayTimes = 0;
    static int send_supply_signal = 1;
    static int noGetSupply = 0;
    bool supply_position = IsSupplyArea();
    bool supply_bullet_temp = supply_bullet_;
    static bool time_running = false;
    static int supply_failure_time = 0;

    if (supply_position)
    {
      if (supply_bullet_temp)
      {
        supply_failure_time++;
        if (supply_failure_time > 100)
        {
          supply_failure_time = 0;
          supplier_can_status = 0;
          return false;
        }
        //补弹只发送一条命令的标识符
        if (send_supply_signal == 1)
        {
          roborts_msgs::ProjectileSupply projectileSupply;
          projectileSupply.number = 50;
          send_supply_signal = 0;
          ros_projectile_supply_pub_.publish(projectileSupply);
          return true;
        }
      }

      if (!supply_bullet_temp)
      {
        supplyDelayTimes++;
        std::cout << "supplyDelayTimes:" << supplyDelayTimes << std::endl;
        //有弹之后,待5秒离开补弹区域
        if (supplyDelayTimes >= 50)
        {
          supplyDelayTimes = 0;
          send_supply_signal = 1;
          time_running = false;
          supply_failure_time = 0;
          return false;
        }
        else
        {
          return true;
        }
      }
      else
      {
        return true;
      }
    }
    else
    {
      return false;
    }
  }

  //加成判断
  bool IsNeedBonus()
  {
    bool bonus_position = IsBonusArea();
    //没有加成,可以加成,并且不在加成位置,则返回加成区
    if ((!bonus_) && can_supply && (!bonus_position))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  //IsEscape
  bool IsEscape()
  {
    return supply_bullet_ && (supplier_can_status <= 0);
  }

  /*
加成也需要加上异常处理.还未实现
在加成区,待5秒,收到加成信号,离开
在加成区,待8秒,没有收到加成信号,也要离开
*/
  bool IsStayBonus()
  {
    bool bonus_position = IsBonusArea();
    static int BonusDelayTimes = 0;
    if (bonus_position && (!bonus_) && can_supply)
    {
      BonusDelayTimes++;
      std::cout << "BonusDelayTimes:" << BonusDelayTimes << std::endl;
      if (BonusDelayTimes >= 100)
      {
        BonusDelayTimes = 0;
        can_supply = false;
        return false;
      }
      else
      {
        return true;
      }
    }
    else
    {
      BonusDelayTimes = 0;
      std::cout << "BonusDelayTimes:" << BonusDelayTimes << std::endl;
      return false;
    }
  }

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal)
  {
    new_goal_ = true;
    goal_ = *goal;
  }

  // Supply
  //此处加上了多机通信部分的反馈,但没有测试
  void SupplyCallback(const roborts_msgs::MucInterface::ConstPtr &supply_bullet)
  {
    supply_bullet_ = supply_bullet->need_supply;
    roborts_msgs::Teammate team_mate;
    team_mate.find_enemy = IsEnemyDetected();
    team_mate.self_x = GetRobotMapPose().pose.position.x;
    team_mate.self_x = GetRobotMapPose().pose.position.y;
    team_mate.enemy_x = GetEnemy().pose.position.x;
    team_mate.enemy_y = GetEnemy().pose.position.y;
    ros_teammate_msg_pub_.publish(team_mate);

    ROS_INFO("%s: %d", __FUNCTION__, (int)supply_bullet_);
  }

  // RobotDamageCallback
  void RobotDamageCallback(const roborts_msgs::RobotDamage::ConstPtr &robot_damage)
  {

    hurt_type_ = robot_damage->damage_type;
    armor_id_ = robot_damage->damage_source;
    armor_hurt_time = ros::Time::now();
    ROS_INFO("hurt_type_: %d", hurt_type_);
    ROS_INFO("armor_id_: %d", armor_id_);
  }

  //RobotStatusCallback
  void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &robot_status)
  {
    robot_id_ = robot_status->id;
    if (robot_status->id / 10)
    {
      robot_color_ = "blue";
    }
    else
    {
      robot_color_ = "red";
    }
    remain_hp_ = robot_status->remain_hp;
    heat_cooling_limit_ = robot_status->heat_cooling_limit;
    heat_cooling_rate_ = robot_status->heat_cooling_rate;
    ROS_INFO("remain_hp_: %d", remain_hp_);
    ROS_INFO("heat_cooling_limit_: %d", heat_cooling_limit_);
    ROS_INFO("heat_cooling_rate_: %d", heat_cooling_rate_);
  }

  //RobotSupplyCallback
  void SupplierStatusCallback(const roborts_msgs::SupplierStatus::ConstPtr &robot_status)
  {
    supply_status_ = robot_status->status;
    if (supply_status_ == 2)
    {
      if (supplier_can_status > 0)
        supplier_can_status--;
    }
    ROS_INFO("supply_status_: %d", supply_status_);
  }

  //GameStatusCallback
  void GameStatusCallback(const roborts_msgs::GameStatus::ConstPtr &game_status)
  {

    game_status_ = game_status->game_status;
    remaining_time_ = game_status->remaining_time;
    if ((game_status_ == 4) && (remaining_time_ % 60 == 59))
    {
      supplier_can_status = 2;
    }
    if (remaining_time_ % 60 == 0)
    {
      bonus_occupied = true;
    }
    ROS_INFO("game_status_: %d", game_status_);
  }

  //BonusStatusCallback
  //此处已测试.can_supply是能否加成的判断
  void BonusStatusCallback(const roborts_msgs::BonusStatus::ConstPtr &bonus_status)
  {

    // 加成区域的回调函数
    red_bonus_ = bonus_status->red_bonus;
    blue_bonus_ = bonus_status->blue_bonus;
    if (robot_color_ == "blue")
    {
      if (blue_bonus_ == 0)
      {
        can_supply = true;
      }
      else if (blue_bonus_ == 2)
      {
        can_supply = false;
      }
    }
    if (robot_color_ == "red")
    {
      if (red_bonus_ == 0)
      {
        can_supply = true;
      }
      else if (red_bonus_ == 2)
      {
        can_supply = false;
      }
    }
    ROS_INFO("can_supply: %d", (int)can_supply);
    ROS_INFO("blue_bonus_: %d", blue_bonus_);
  }

  //RobotBonusCallback
  void RobotBonusCallback(const roborts_msgs::RobotBonus::ConstPtr &robot_bonus)
  {
    //true bonus have
    //false bonus true
    bonus_ = robot_bonus->bonus;
    if (bonus_)
    {
      bonus_occupied = false;
    }
    ROS_INFO("bonus_: %d", bonus_);
  }

  void TeammateCallback(const roborts_msgs::Teammate::ConstPtr &team_mate)
  {
    team_enemy_detected = team_mate->find_enemy;
    team_enemy_pose_x = team_mate->enemy_x;
    team_enemy_pose_y = team_mate->enemy_y;
    ROS_INFO("team_enemy_detected: %d", (int)team_enemy_detected);
    ROS_INFO("team_enemy_pose_x: %f", team_enemy_pose_x);
    ROS_INFO("team_enemy_pose_y: %f", team_enemy_pose_y);
  }

  //比赛开始判断
  int GetGameStart()
  {
    return game_status_;
  }

  //是car1还是car2
  //car1 补弹,为false
  //car2 加成,为true
  //bonus or supply proiority
  bool mainCondition()
  {
    //bonus
    return true;
  }
  //设置search 其中一个变量,到达敌方消失区域则开始搜索,没有测试完整效果
  bool SetSearchFlag(bool arrive_goal_flag)
  {
    arrive_goal_flag_ = arrive_goal_flag;
    return true;
  }
  // car_turn 1:扭腰
  void CarTurn(int i)
  {
    std_msgs::Int8 turn_msg;
    turn_msg.data = i;
    cat_walk_.publish(turn_msg);
    ROS_INFO("car_turn publish %d", i);
  }

  //返回受伤害装甲板id
  int GetArmorId()
  {
    int hurt_time = ros::Time::now().sec - armor_hurt_time.sec;
    if (hurt_time < 5)
    {
      return armor_id_;
    }
    else
      return 0;
  }

  //armorid condition
  bool IsArmorIdTurn()
  {
    if (GetArmorId() > 0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  geometry_msgs::PoseStamped GetGoal() const
  {
    return goal_;
  }

  bool IsNewGoal()
  {
    if (new_goal_)
    {
      new_goal_ = false;
      return true;
    }
    else
    {
      return false;
    }
  }

  //SetGimbalMode,设置云台模式
  //0为底盘跟随云台
  //1为云台跟随底盘
  int SetGimbalMode(int mode)
  {
    gimbal_mode_client_srv.request.gimbal_mode = mode;
    if (gimbal_client_.call(gimbal_mode_client_srv))
    {
      ROS_INFO("gimbal_mode_: %d", mode);
    }
    else
    {
      ROS_ERROR("Failed to call service gimbal_mode_client_srv");
      return 1;
    }

    return 0;
  }

  //ctroller shoot
  int CtrlShoot(int mode, int number)
  {
    shoot_client_srv.request.mode = mode;
    shoot_client_srv.request.number = number;
    if (shoot_client_.call(shoot_client_srv))
    {
      ROS_INFO("shoot_client_mode: %d", mode);
      ROS_INFO("shoot_client_number: %d", number);
    }
    else
    {
      ROS_ERROR("Failed to call service shoot_client_srv");
      return 1;
    }

    return 0;
  }
  //判断是否在补给区
  bool IsSupplyArea()
  {
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
    std::string proto_file_path = full_path;
    if (!LoadParam(proto_file_path))
    {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
    auto robot_map_pose = GetRobotMapPose();
    auto dx = supply_position_.pose.position.x - robot_map_pose.pose.position.x;
    auto dy = supply_position_.pose.position.y - robot_map_pose.pose.position.y;

    auto boot_yaw = tf::getYaw(supply_position_.pose.orientation); //Helper function for getting yaw from a Quaternion.
    auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

    tf::Quaternion rot1, rot2;                                      //The Quaternion implements quaternion to perform linear algebra rotations in combination with Matrix3x3, Vector3 and Transform.
                                                                    //四元数与矩阵x3x3、向量3和变换相结合，实现四元数的线性代数旋转 tf::quaternionMsgToTF函数取/tf中四元数消息转换为四元数
    tf::quaternionMsgToTF(supply_position_.pose.orientation, rot1); //使用geometry_msgs::PoseStampedPtr pose构造一个对象  tf::quaternionMsgToTF(pose->pose.orientation,q);   //把geomsg形式的四元数转化为tf形式，得到q
    tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);   //
    auto d_yaw = rot1.angleShortestPath(rot2);                      //返回这个四元数与另一个四元数之间沿最短路径的夹角

    if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5)
    {
      // std::cout<<"distance:"<<std::sqrt(std::pow(dx, 2) + std::pow(dy, 2))<<std::endl;
      return false;
    }
    // std::cout<<"distance:"<<std::sqrt(std::pow(dx, 2) + std::pow(dy, 2))<<std::endl;
    return true;
  }

  //判断是否在bonus区

  bool IsBonusArea()
  {
    std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
    std::string proto_file_path = full_path;
    if (!LoaBonusParam(proto_file_path))
    {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

    auto robot_map_pose = GetRobotMapPose();
    robot_map_pose_ = robot_map_pose;
    auto dx = bonus_position_.pose.position.x - robot_map_pose.pose.position.x;
    auto dy = bonus_position_.pose.position.y - robot_map_pose.pose.position.y;

    auto boot_yaw = tf::getYaw(bonus_position_.pose.orientation); //Helper function for getting yaw from a Quaternion.
    auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

    tf::Quaternion rot1, rot2;                                     //The Quaternion implements quaternion to perform linear algebra rotations in combination with Matrix3x3, Vector3 and Transform.
                                                                   //四元数与矩阵x3x3、向量3和变换相结合，实现四元数的线性代数旋转 tf::quaternionMsgToTF函数取/tf中四元数消息转换为四元数
    tf::quaternionMsgToTF(bonus_position_.pose.orientation, rot1); //使用geometry_msgs::PoseStampedPtr pose构造一个对象  tf::quaternionMsgToTF(pose->pose.orientation,q);   //把geomsg形式的四元数转化为tf形式，得到q
    tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);  //
    auto d_yaw = rot1.angleShortestPath(rot2);                     //返回这个四元数与另一个四元数之间沿最短路径的夹角

    if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5)
    {
      // std::cout<<"distance:"<<std::sqrt(std::pow(dx, 2) + std::pow(dy, 2))<<std::endl;
      return false;
    }
    // std::cout<<"distance:"<<std::sqrt(std::pow(dx, 2) + std::pow(dy, 2))<<std::endl;
    return true;
  }

  bool LoadParam(const std::string &proto_file_path)
  {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config))
    {
      return false;
    }

    supply_position_.header.frame_id = "map";
    supply_position_.pose.position.x = decision_config.supply_bot().start_position().x();
    supply_position_.pose.position.z = decision_config.supply_bot().start_position().z();
    supply_position_.pose.position.y = decision_config.supply_bot().start_position().y();

    auto master_quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.supply_bot().start_position().roll(),
                                                                     decision_config.supply_bot().start_position().pitch(),
                                                                     decision_config.supply_bot().start_position().yaw());
    supply_position_.pose.orientation = master_quaternion;

    return true;
  }

  //当时为了修改省时,用的bonus_bot().blue(),可以在修改
  bool LoaBonusParam(const std::string &proto_file_path)
  {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config))
    {
      return false;
    }

    bonus_position_.header.frame_id = "map";
    bonus_position_.pose.position.x = decision_config.bonus_bot().blue().x();
    bonus_position_.pose.position.z = decision_config.bonus_bot().blue().z();
    bonus_position_.pose.position.y = decision_config.bonus_bot().blue().y();

    auto master_quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.bonus_bot().blue().roll(),
                                                                     decision_config.bonus_bot().blue().pitch(),
                                                                     decision_config.bonus_bot().blue().yaw());
    bonus_position_.pose.orientation = master_quaternion;

    return true;
  }

  double GetRelative_x()
  {
    return relative_x_;
  }

  double GetRelative_y()
  {
    return relative_y_;
  }

  //goal condition(到敌方消失位置)
  bool IsGoal()
  {
    std::cout << "undetected_count_search_:" << undetected_count_search_ << std::endl;
    if (undetected_count_search_ >= 200 && undetected_count_search_ < 295 && (!arrive_goal_flag_))
      return true;
    else
      return false;
  }

  //search condition
  bool IsSearch()
  {
    std::cout << "undetected_count_search_:" << undetected_count_search_ << std::endl;
    if (undetected_count_search_ > 0 && arrive_goal_flag_)
      return true;
    else
      return false;
  }
  /*---------------------------------- Tools ------------------------------------------*/

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2)
  {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2)
  {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  const geometry_msgs::PoseStamped GetRobotMapPose()
  {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap()
  {
    return costmap_ptr_;
  }

  const CostMap2D *GetCostMap2D()
  {
    return costmap_2d_;
  }

  const unsigned char *GetCharMap()
  {
    return charmap_;
  }

private:
  void UpdateRobotPose()
  {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity(); //?

    robot_tf_pose.frame_id_ = "base_link";
    robot_tf_pose.stamp_ = ros::Time();
    try
    {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex)
    {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;

  //supply bullet
  bool supply_bullet_;

  ros::Subscriber supply_sub_;
  ros::Subscriber ros_robot_damage_sub_;
  ros::Subscriber ros_robot_status_sub_;
  ros::Subscriber ros_supplier_status_sub_;
  ros::Subscriber ros_game_status_sub_;
  ros::Subscriber ros_bonus_status_sub_;
  ros::Subscriber ros_robot_bonus_sub_;

  ros::Subscriber ros_game_survivor_sub_;

  ros::Subscriber ros_teammate_msg_sub_;
  ros::Publisher ros_teammate_msg_pub_;

  ros::Publisher ros_projectile_supply_pub_;
  //car turn
  ros::Publisher cat_walk_;

  //service
  //gimbal
  ros::ServiceClient gimbal_client_;
  //shoot
  ros::ServiceClient shoot_client_;

  //gimbal mode
  roborts_msgs::GimbalMode gimbal_mode_client_srv;
  roborts_msgs::GameSurvivor robot_game_survivor;
  //shoot
  roborts_msgs::ShootCmd shoot_client_srv;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //armor time
  ros::Time armor_hurt_time;
  bool chase_enemy_;
  bool enemy_detected_;
  //! boot position
  geometry_msgs::PoseStamped supply_position_;
  geometry_msgs::PoseStamped bonus_position_;

  //robot_damage
  int armor_id_;
  int hurt_type_;

  //robot_status
  int robot_id_;
  std::string robot_color_;
  int remain_hp_;
  int heat_cooling_limit_;
  int heat_cooling_rate_;

  //SupplierStatus
  int supply_status_;

  int game_status_;

  //bonus_status
  int red_bonus_;
  int blue_bonus_;
  //
  bool bonus_occupied;

  //bonus
  bool bonus_;

  bool can_supply;
  bool go_bonus;

  //relative_x.y
  double relative_x_;
  double relative_y_;
  //game time
  int remaining_time_;
  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  geometry_msgs::PoseStamped enemy_pose_;

  //Enemy Detected buf
  std::deque<bool> enemy_deque;

  //search
  int undetected_count_search_;

  //判断是否到达目标点变量
  bool arrive_goal_flag_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D *costmap_2d_;
  unsigned char *charmap_;

  //enemy_tf_
  geometry_msgs::TransformStamped enemy_tf_;
  tf::TransformBroadcaster tf_broadcaster_;

  // 队友的提供的信息
  bool team_enemy_detected;
  float team_enemy_pose_x;
  float team_enemy_pose_y;
  //! robot map pose
  geometry_msgs::PoseStamped robot_map_pose_;

  //supplier status
  int supplier_can_status;
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
