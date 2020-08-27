#ifndef ROBORTS_DECISION_ARMORID_BEHAVIOR_H
#define ROBORTS_DECISION_ARMORID_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision
{
class ArmorIdBehavior
{
public:
  ArmorIdBehavior(ChassisExecutor *&chassis_executor,
                  GimbalExecutor *&gimbal_executor,
                  Blackboard *&blackboard,
                  const std::string &proto_file_path) : chassis_executor_(chassis_executor),
                                                        gimbal_executor_(gimbal_executor),
                                                        blackboard_(blackboard)
  {
    }

  void Run()
  {
   //更新底盘信息
    auto chassis_executor_state = ChassisUpdate();
    //更新云台信息
    auto gimbal_executor_state = GimbalUpdate();
    //取消扭腰
    blackboard_->CarTurn(0);
    int execution_mode_ = chassis_executor_->GetExcutionMode();
    std::cout << "ArmorId ExcutionMode:" << execution_mode_ << std::endl;

    if (execution_mode_ == 3)
    {
      chassis_executor_state = BehaviorState::IDLE;
     
    }

    //根据装甲板id转向
    int armor_id = blackboard_->GetArmorId();
    roborts_msgs::GimbalAngle gimbal_angle;
    gimbal_angle.yaw_mode = false;
    gimbal_angle.pitch_mode = false;
    std::cout << "armor_id" << armor_id << std::endl;

    if (armor_id == 0)
    {
    }
    else if (armor_id == 1) //left
    {
      gimbal_angle.yaw_angle = 1.5;
      gimbal_angle.pitch_angle = 0;
      blackboard_->SetGimbalMode(1);
      gimbal_executor_->Execute(gimbal_angle);
    }
    else if (armor_id == 3) //right
    {
      gimbal_angle.yaw_angle = -1.5;
      gimbal_angle.pitch_angle = 0;
      blackboard_->SetGimbalMode(1);
      gimbal_executor_->Execute(gimbal_angle);
    }
    else if (armor_id == 2) //back
    {
      gimbal_angle.yaw_angle = 1.57;
      gimbal_angle.pitch_angle = 0;
      blackboard_->SetGimbalMode(0);
      gimbal_executor_->Execute(gimbal_angle);
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

  ~ArmorIdBehavior() = default;

private:
  //! executor
  ChassisExecutor *const chassis_executor_;
  GimbalExecutor *const gimbal_executor_;

  //! perception information
  Blackboard *const blackboard_;

  //! ArmorId buffer
  std::vector<geometry_msgs::PoseStamped> ArmorId_goals_;
  unsigned int ArmorId_count_;
  unsigned int point_size_;
};
} // namespace roborts_decision
#endif //ROBORTS_DECISION_ARMORID_BEHAVIOR_H
