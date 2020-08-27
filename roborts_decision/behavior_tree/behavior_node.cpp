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

#include <chrono>
#include <thread>
#include <vector>

//#include "../blackboard/blackboard.h"
//#include "behavior_state.h"
#include "behavior_node.h"

namespace roborts_decision
{
class PatrolAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  PatrolAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, GimbalExecutor *&gimbal_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), patrol_behavior_(chassis_executor, gimbal_executor, blackboard, proto_file_path)
  {

    ros::NodeHandle nh;
    cmd_vel_acc_pub_ = nh.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 100);
    blackboard_ = blackboard;
  }
  ~PatrolAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  GimbalExecutor *gimbal_executor_;
  Blackboard::Ptr blackboard_ptr_;
  Blackboard *blackboard_;
  std::string proto_file_path_;
  roborts_decision::PatrolBehavior patrol_behavior_;

  roborts_msgs::TwistAccel zero_twist_accel_;

  //! velocity with accel publisher in ROS
  ros::Publisher cmd_vel_acc_pub_;

  //  geometry_msgs::Twist whirl_vel_;
  roborts_msgs::TwistAccel whirl_vel_;

protected:
  void OnInitialize()
  {
    cmd_vel_acc_pub_.publish(zero_twist_accel_);
    //blackboard_->CarTurn(0);
    ROS_INFO("PatrolAction OnInitialize");
  }
  BehaviorState Update()
  {
    patrol_behavior_.Run();
    ROS_INFO("PatrolAction Update");
  }

  void OnTerminate(BehaviorState state)
  {

    ROS_INFO("PatrolAction OnTerminate");
  }
};

class ChaseAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  ChaseAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, GimbalExecutor *&gimbal_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), chase_behavior_(chassis_executor, gimbal_executor, blackboard, proto_file_path)
  {
    ros::NodeHandle nh;
    cmd_vel_acc_pub_ = nh.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 100);
    blackboard_ = blackboard;
  }
  ~ChaseAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  GimbalExecutor *gimbal_executor_;
  Blackboard::Ptr blackboard_ptr_;
  Blackboard *blackboard_;
  std::string proto_file_path_;
  roborts_decision::ChaseBehavior chase_behavior_;

  roborts_msgs::TwistAccel zero_twist_accel_;

  //! velocity with accel publisher in ROS
  ros::Publisher cmd_vel_acc_pub_;

  //  geometry_msgs::Twist whirl_vel_;
  roborts_msgs::TwistAccel whirl_vel_;

protected:
  void OnInitialize()
  {
    // cmd_vel_acc_pub_.publish(zero_twist_accel_);
    //blackboard_->CarTurn(0);
    ROS_INFO("ChaseAction OnInitialize");
  }
  BehaviorState Update()
  {
    chase_behavior_.Run();
    ROS_INFO("ChaseAction Update");
  }

  void OnTerminate(BehaviorState state)
  {

    ROS_INFO("ChaseAction OnTerminate");
  }
};

class EscapeAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  EscapeAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, GimbalExecutor *&gimbal_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), escape_behavior_(chassis_executor, gimbal_executor, blackboard, proto_file_path)
  {
    ros::NodeHandle nh;
    cmd_vel_acc_pub_ = nh.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 100);
    chassis_executor_ = chassis_executor;
    blackboard_ = blackboard;
  }
  ~EscapeAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  GimbalExecutor *gimbal_executor_;
  Blackboard::Ptr blackboard_ptr_;
  Blackboard *blackboard_;
  std::string proto_file_path_;
  roborts_decision::EscapeBehavior escape_behavior_;
  roborts_msgs::TwistAccel zero_twist_accel_;

  //! velocity with accel publisher in ROS
  ros::Publisher cmd_vel_acc_pub_;

  //  geometry_msgs::Twist whirl_vel_;
  roborts_msgs::TwistAccel whirl_vel_;

protected:
  void OnInitialize()
  {
    //blackboard_->CarTurn(0);
    ROS_INFO("EscapeAction OnInitialize");
  }
  BehaviorState Update()
  {
    escape_behavior_.Run();
    ROS_INFO("EscapeAction Update");
  }

  void OnTerminate(BehaviorState state)
  {
    // cmd_vel_acc_pub_.publish(zero_twist_accel_);
    ROS_INFO("EscapeAction OnTerminate");
  }
};

class BackSupplyAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  BackSupplyAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, GimbalExecutor *&gimbal_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), back_supply_behavior_(chassis_executor, gimbal_executor, blackboard, proto_file_path)
  {
    blackboard_ = blackboard;
  }
  ~BackSupplyAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  GimbalExecutor *gimbal_executor_;
  Blackboard::Ptr blackboard_ptr_;
  Blackboard *blackboard_;
  std::string proto_file_path_;
  roborts_decision::BackSupplyBehavior back_supply_behavior_;

protected:
  void OnInitialize()
  {
    //blackboard_->CarTurn(0);
    ROS_INFO("BackSupplyAction OnInitialize");
  }
  BehaviorState Update()
  {
    back_supply_behavior_.Run();
    ROS_INFO("BackSupplyAction Update");
  }

  void OnTerminate(BehaviorState state)
  {

    ROS_INFO("BackSupplyAction OnTerminate");
  }
};

class SupplyAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  SupplyAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, GimbalExecutor *&gimbal_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), supply_behavior_(chassis_executor, gimbal_executor, blackboard, proto_file_path)
  {
    blackboard_ = blackboard;
  }
  ~SupplyAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  GimbalExecutor *gimbal_executor_;
  Blackboard::Ptr blackboard_ptr_;
  Blackboard *blackboard_;
  std::string proto_file_path_;
  roborts_decision::SupplyBehavior supply_behavior_;

protected:
  void OnInitialize()
  {
    //blackboard_->CarTurn(0);
    ROS_INFO("SupplyAction OnInitialize");
  }
  BehaviorState Update()
  {
    supply_behavior_.Run();
    ROS_INFO("SupplyAction Update");
  }

  void OnTerminate(BehaviorState state)
  {

    ROS_INFO("SupplyAction OnTerminate");
  }
};

class BackBonusAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  BackBonusAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, GimbalExecutor *&gimbal_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), back_bonus_behavior_(chassis_executor, gimbal_executor, blackboard, proto_file_path)
  {
    blackboard_ = blackboard;
  }
  ~BackBonusAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  GimbalExecutor *gimbal_executor_;
  Blackboard::Ptr blackboard_ptr_;
  Blackboard *blackboard_;
  std::string proto_file_path_;
  roborts_decision::BackBonusBehavior back_bonus_behavior_;

protected:
  void OnInitialize()
  {
    //blackboard_->CarTurn(0);
    ROS_INFO("BackBonusAction OnInitialize");
  }
  BehaviorState Update()
  {
    back_bonus_behavior_.Run();
    ROS_INFO("BackBonusAction Update");
  }

  void OnTerminate(BehaviorState state)
  {

    ROS_INFO("BackBonusAction OnTerminate");
  }
};

class BonusAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  BonusAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, GimbalExecutor *&gimbal_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), bonus_behavior_(chassis_executor, gimbal_executor, blackboard, proto_file_path)
  {
    blackboard_ = blackboard;
  }
  ~BonusAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  GimbalExecutor *gimbal_executor_;
  Blackboard::Ptr blackboard_ptr_;
  Blackboard *blackboard_;
  std::string proto_file_path_;
  roborts_decision::BonusBehavior bonus_behavior_;

protected:
  void OnInitialize()
  {
    //blackboard_->CarTurn(0);
    ROS_INFO("BonusAction OnInitialize");
  }
  BehaviorState Update()
  {
    bonus_behavior_.Run();
    ROS_INFO("BonusAction Update");
  }

  void OnTerminate(BehaviorState state)
  {

    ROS_INFO("BonusAction OnTerminate");
  }
};

class SearchAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  SearchAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, GimbalExecutor *&gimbal_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), search_behavior_(chassis_executor, gimbal_executor, blackboard, proto_file_path)
  {
    this->blackboard_ = blackboard;
  }
  ~SearchAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  GimbalExecutor *gimbal_executor_;
  Blackboard *blackboard_;
  std::string proto_file_path_;
  roborts_decision::SearchBehavior search_behavior_;

protected:
  void OnInitialize()
  {
    //blackboard_->CarTurn(0);
    ROS_INFO("SearchAction OnInitialize");
  }
  BehaviorState Update()
  {
    //敌方消失位置传入

    search_behavior_.Run();
    ROS_INFO("SearchAction Update");
  }

  void OnTerminate(BehaviorState state)
  {

    ROS_INFO("SearchAction OnTerminate");
  }
};

//定点搜索
class GoalAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  GoalAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, GimbalExecutor *&gimbal_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), goal_behavior_(chassis_executor, gimbal_executor, blackboard, proto_file_path)
  {
    this->blackboard_ = blackboard;
  }
  ~GoalAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  GimbalExecutor *gimbal_executor_;
  Blackboard *blackboard_;
  std::string proto_file_path_;
  roborts_decision::GoalBehavior goal_behavior_;

protected:
  void OnInitialize()
  {
    //blackboard_->CarTurn(0);
    ROS_INFO("GoalAction OnInitialize");
  }
  BehaviorState Update()
  {
    //敌方消失位置传入
    //Goal_behavior_.SetLastPosition(blackboard_->GetGoal());
    goal_behavior_.Run();
    ROS_INFO("GoalAction Update");
  }

  void OnTerminate(BehaviorState state)
  {

    ROS_INFO("GoalAction OnTerminate");
  }
};

//根据装甲板id切换云台转向
class ArmorIdAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  ArmorIdAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, GimbalExecutor *&gimbal_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), armorId_behavior_(chassis_executor, gimbal_executor, blackboard, proto_file_path)
  {
    this->blackboard_ = blackboard;
  }
  ~ArmorIdAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  GimbalExecutor *gimbal_executor_;
  Blackboard *blackboard_;
  std::string proto_file_path_;
  roborts_decision::ArmorIdBehavior armorId_behavior_;

protected:
  void OnInitialize()
  {
    //blackboard_->CarTurn(0);
    ROS_INFO("ArmorIdAction OnInitialize");
  }
  BehaviorState Update()
  {
    //敌方消失位置传入
    //ArmorId_behavior_.SetLastPosition(blackboard_->GetArmorId());
    armorId_behavior_.Run();
    ROS_INFO("ArmorIdAction Update");
  }

  void OnTerminate(BehaviorState state)
  {

    ROS_INFO("ArmorIdAction OnTerminate");
  }
};
} // namespace roborts_decision
