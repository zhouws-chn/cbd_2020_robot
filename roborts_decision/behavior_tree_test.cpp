#include <ros/ros.h>
#include <iostream>
#include <functional>
#include "executor/chassis_executor.h"
#include "executor/gimbal_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/armor_id_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/supply_behavior.h"
#include "example_behavior/back_supply_behavior.h"
#include "example_behavior/bonus_behavior.h"
#include "example_behavior/back_bonus_behavior.h"
#include "behavior_tree/behavior_node.cpp"
#include "behavior_tree/behavior_tree.h"
#include "behavior_tree/behavior_node.h"

// bool chaseUpdate()
// {
//   static int count = 0;
//   count++;
//   bool flag = false;
//   if (count % 1000 > 500)
//   {
//     flag = true;
//   }
//   ROS_INFO("chaseUpdate %d state:%d-------------------", count, flag);

//   return flag;
// }

// bool supplyUpdate()
// {
//   static int count = 0;
//   count++;
//   bool flag = false;
//   if (count % 2000 < 100)
//   {
//     flag = true;
//   }
//   ROS_INFO("supplyUpdate %d state:%d-------------------", count, flag);

//   return flag;
// }

// bool searchUpdate()
// {
//   static int count = 0;
//   count++;
//   bool flag = false;
//   if (count % 200 > 100)
//   {
//     flag = true;
//   }
//   ROS_INFO("searchUpdate %d state:%d-------------------", count, flag);

//   return flag;
// }

// bool escapeUpdate()
// {
//   static int count = 0;
//   count++;
//   bool flag = false;
//   if (count % 400 > 200)
//   {
//     flag = true;
//   }
//   ROS_INFO("escapeUpdate %d state:%d-------------------", count, flag);

//   return flag;
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_tree_test");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto gimbal_executor = new roborts_decision::GimbalExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  roborts_decision::Blackboard::Ptr blackboard_ptr(blackboard);

  //std::function<bool()> chase_precondition = std::bind(&roborts_decision::Blackboard::IsEnemyDetected, blackboard);

  std::function<bool()> chase_precondition = std::bind(&roborts_decision::Blackboard::IsEnemyDetected, blackboard);

  std::function<bool()> supply_precondition = std::bind(&roborts_decision::Blackboard::IsStaySupply, blackboard);

  std::function<bool()> escape_precondition = std::bind(&roborts_decision::Blackboard::IsEscape, blackboard);

  std::function<bool()> back_supply_precondition = std::bind(&roborts_decision::Blackboard::IsNeedSupply, blackboard);

  std::function<bool()> back_bonus_precondition = std::bind(&roborts_decision::Blackboard::IsNeedBonus, blackboard);

  std::function<bool()> bonus_precondition = std::bind(&roborts_decision::Blackboard::IsStayBonus, blackboard);

  std::function<bool()> search_precondition = std::bind(&roborts_decision::Blackboard::IsSearch, blackboard);

  std::function<bool()> armor_id_precondition = std::bind(&roborts_decision::Blackboard::IsArmorIdTurn, blackboard);

  std::function<bool()> main_precondition = std::bind(&roborts_decision::Blackboard::mainCondition, blackboard);

  //此处搜索goal,判断condition
  std::function<bool()> goal_precondition = std::bind(&roborts_decision::Blackboard::IsGoal, blackboard);

  roborts_decision::SelectorNode::Ptr patrol_armor_select_node_ptr(new roborts_decision::SelectorNode("patrol_armor_select_node", blackboard_ptr));

  roborts_decision::SelectorNode::Ptr bonus_select_node_ptr(new roborts_decision::SelectorNode("bonus_select_node", blackboard_ptr));

  roborts_decision::SelectorNode::Ptr supply_select_node_ptr(new roborts_decision::SelectorNode("supply_select_node", blackboard_ptr));

  roborts_decision::SelectorNode::Ptr root_node_ptr(new roborts_decision::SelectorNode("root_node", blackboard_ptr));

  roborts_decision::SelectorNode::Ptr chase_select_node_ptr(new roborts_decision::SelectorNode("chase_select_node", blackboard_ptr));

  roborts_decision::SelectorNode::Ptr search_select_node_ptr(new roborts_decision::SelectorNode("search_select_node", blackboard_ptr));

  roborts_decision::SequenceNode::Ptr sequence_node_ptr(new roborts_decision::SequenceNode("sequence_node", blackboard_ptr));

  roborts_decision::ActionNode::Ptr patrol_action_node_ptr(new roborts_decision::PatrolAction("patrol_action_node", blackboard_ptr, blackboard, chassis_executor, gimbal_executor, full_path));

  roborts_decision::ActionNode::Ptr chase_action_node_ptr(new roborts_decision::ChaseAction("chase_action_node", blackboard_ptr, blackboard, chassis_executor, gimbal_executor, full_path));

  roborts_decision::ActionNode::Ptr supply_action_node_ptr(new roborts_decision::SupplyAction("supply_action_node", blackboard_ptr, blackboard, chassis_executor, gimbal_executor, full_path));

  roborts_decision::ActionNode::Ptr search_action_node_ptr(new roborts_decision::SearchAction("search_action_node", blackboard_ptr, blackboard, chassis_executor, gimbal_executor, full_path));

  roborts_decision::ActionNode::Ptr escape_action_node_ptr(new roborts_decision::EscapeAction("escape_action_node", blackboard_ptr, blackboard, chassis_executor, gimbal_executor, full_path));

  roborts_decision::ActionNode::Ptr back_supply_area_action_node_ptr(new roborts_decision::BackSupplyAction("back_supply_area_action_node", blackboard_ptr, blackboard, chassis_executor, gimbal_executor, full_path));

  roborts_decision::ActionNode::Ptr back_bonus_area_action_node_ptr(new roborts_decision::BackBonusAction("back_bonus_area_action_node", blackboard_ptr, blackboard, chassis_executor, gimbal_executor, full_path));

  roborts_decision::ActionNode::Ptr bonus_action_node_ptr(new roborts_decision::BonusAction("bonus_action_node", blackboard_ptr, blackboard, chassis_executor, gimbal_executor, full_path));

  roborts_decision::ActionNode::Ptr goal_action_node_ptr(new roborts_decision::GoalAction("goal_action_node", blackboard_ptr, blackboard, chassis_executor, gimbal_executor, full_path));

  roborts_decision::ActionNode::Ptr armor_id_action_node_ptr(new roborts_decision::ArmorIdAction("armor_id_action_node", blackboard_ptr, blackboard, chassis_executor, gimbal_executor, full_path));

  // roborts_decision::PreconditionNode::Ptr chase_precondition_node_ptr(new roborts_decision::PreconditionNode("chase_precondition_node", blackboard_ptr,
  //                                                                                                           chaseUpdate, roborts_decision::AbortType::BOTH));

  // roborts_decision::PreconditionNode::Ptr supply_precondition_node_ptr(new roborts_decision::PreconditionNode("supply_precondition_node", blackboard_ptr,
  //                                                                                                             supplyUpdate, roborts_decision::AbortType::BOTH));

  // roborts_decision::PreconditionNode::Ptr search_precondition_node_ptr(new roborts_decision::PreconditionNode("search_precondition_node", blackboard_ptr,
  //                                                                                                             searchUpdate, roborts_decision::AbortType::BOTH));

  // roborts_decision::PreconditionNode::Ptr escape_precondition_node_ptr(new roborts_decision::PreconditionNode("escape_precondition_node", blackboard_ptr,
  //                                                                                                             escapeUpdate, roborts_decision::AbortType::BOTH));

  roborts_decision::PreconditionNode::Ptr chase_precondition_node_ptr(new roborts_decision::PreconditionNode("chase_precondition_node", blackboard_ptr,
                                                                                                             chase_precondition, roborts_decision::AbortType::BOTH));

  roborts_decision::PreconditionNode::Ptr supply_precondition_node_ptr(new roborts_decision::PreconditionNode("supply_precondition_node", blackboard_ptr,
                                                                                                              supply_precondition, roborts_decision::AbortType::BOTH));

  roborts_decision::PreconditionNode::Ptr escape_precondition_node_ptr(new roborts_decision::PreconditionNode("escape_precondition_node", blackboard_ptr,
                                                                                                              escape_precondition, roborts_decision::AbortType::BOTH));

  roborts_decision::PreconditionNode::Ptr back_supply_precondition_node_ptr(new roborts_decision::PreconditionNode("back_supply_area_precondition_node", blackboard_ptr,
                                                                                                                   back_supply_precondition, roborts_decision::AbortType::BOTH));

  roborts_decision::PreconditionNode::Ptr bonus_precondition_node_ptr(new roborts_decision::PreconditionNode("bonus_precondition_node", blackboard_ptr,
                                                                                                             bonus_precondition, roborts_decision::AbortType::BOTH));

  roborts_decision::PreconditionNode::Ptr back_bonus_precondition_node_ptr(new roborts_decision::PreconditionNode("back_bonus_area_precondition_node", blackboard_ptr,
                                                                                                                  back_bonus_precondition, roborts_decision::AbortType::BOTH));

  roborts_decision::PreconditionNode::Ptr search_precondition_node_ptr(new roborts_decision::PreconditionNode("search_precondition_node", blackboard_ptr,
                                                                                                              search_precondition, roborts_decision::AbortType::BOTH));

  roborts_decision::PreconditionNode::Ptr goal_precondition_node_ptr(new roborts_decision::PreconditionNode("goal_precondition_node", blackboard_ptr,
                                                                                                            goal_precondition, roborts_decision::AbortType::BOTH));

  roborts_decision::PreconditionNode::Ptr armor_id_precondition_node_ptr(new roborts_decision::PreconditionNode("goal_precondition_node", blackboard_ptr,
                                                                                                                armor_id_precondition, roborts_decision::AbortType::BOTH));

  roborts_decision::PreconditionNode::Ptr main_id_precondition_node_ptr(new roborts_decision::PreconditionNode("main_precondition", blackboard_ptr,
                                                                                                               main_precondition, roborts_decision::AbortType::BOTH));

  root_node_ptr->AddChildren(escape_precondition_node_ptr);
  escape_precondition_node_ptr->SetChild(escape_action_node_ptr);

  root_node_ptr->AddChildren(main_id_precondition_node_ptr);
  //bonus
  main_id_precondition_node_ptr->SetChild(bonus_select_node_ptr);
  //supply
  root_node_ptr->AddChildren(supply_select_node_ptr);
  // root_node_ptr->AddChildren(armor_id_action_node_ptr);

  //bonus node
  bonus_select_node_ptr->AddChildren(back_bonus_precondition_node_ptr);
  bonus_select_node_ptr->AddChildren(bonus_precondition_node_ptr);

  bonus_select_node_ptr->AddChildren(back_supply_precondition_node_ptr);
  bonus_select_node_ptr->AddChildren(supply_precondition_node_ptr);

  bonus_select_node_ptr->AddChildren(chase_select_node_ptr);

  bonus_precondition_node_ptr->SetChild(bonus_action_node_ptr);
  back_bonus_precondition_node_ptr->SetChild(back_bonus_area_action_node_ptr);

  supply_precondition_node_ptr->SetChild(supply_action_node_ptr);
  back_supply_precondition_node_ptr->SetChild(back_supply_area_action_node_ptr);

  chase_select_node_ptr->AddChildren(chase_precondition_node_ptr);
  chase_precondition_node_ptr->SetChild(chase_action_node_ptr);
  chase_select_node_ptr->AddChildren(search_select_node_ptr);

  search_select_node_ptr->AddChildren(search_precondition_node_ptr);
  search_precondition_node_ptr->SetChild(search_action_node_ptr);
  search_select_node_ptr->AddChildren(goal_precondition_node_ptr);
  goal_precondition_node_ptr->SetChild(goal_action_node_ptr);
  // search_select_node_ptr->AddChildren(patrol_action_node_ptr);
  search_select_node_ptr->AddChildren(patrol_armor_select_node_ptr);
  patrol_armor_select_node_ptr->AddChildren(armor_id_precondition_node_ptr);
  armor_id_precondition_node_ptr->SetChild(armor_id_action_node_ptr);
  patrol_armor_select_node_ptr->AddChildren(patrol_action_node_ptr);

  //supply node

  supply_select_node_ptr->AddChildren(back_supply_precondition_node_ptr);
  supply_select_node_ptr->AddChildren(supply_precondition_node_ptr);
  supply_select_node_ptr->AddChildren(back_bonus_precondition_node_ptr);
  supply_select_node_ptr->AddChildren(bonus_precondition_node_ptr);

  supply_select_node_ptr->AddChildren(chase_select_node_ptr);

  bonus_precondition_node_ptr->SetChild(bonus_action_node_ptr);
  back_bonus_precondition_node_ptr->SetChild(back_bonus_area_action_node_ptr);

  supply_precondition_node_ptr->SetChild(supply_action_node_ptr);
  back_supply_precondition_node_ptr->SetChild(back_supply_area_action_node_ptr);

  chase_select_node_ptr->AddChildren(chase_precondition_node_ptr);
  chase_precondition_node_ptr->SetChild(chase_action_node_ptr);
  chase_select_node_ptr->AddChildren(search_select_node_ptr);

  search_select_node_ptr->AddChildren(search_precondition_node_ptr);
  search_precondition_node_ptr->SetChild(search_action_node_ptr);
  search_select_node_ptr->AddChildren(goal_precondition_node_ptr);
  goal_precondition_node_ptr->SetChild(goal_action_node_ptr);
  // search_select_node_ptr->AddChildren(patrol_action_node_ptr);
  search_select_node_ptr->AddChildren(patrol_armor_select_node_ptr);
  patrol_armor_select_node_ptr->AddChildren(armor_id_precondition_node_ptr);
  armor_id_precondition_node_ptr->SetChild(armor_id_action_node_ptr);
  patrol_armor_select_node_ptr->AddChildren(patrol_action_node_ptr);

  roborts_decision::BehaviorTree node_tree(root_node_ptr, 100);

  while (ros::ok())
  {
    ros::spinOnce();
    if (blackboard->GetGameStart() == 4)
      break;
    ROS_INFO("GetGameStart : %d", blackboard->GetGameStart());
    usleep(50000);
  }

  node_tree.Run();
  return 0;
}