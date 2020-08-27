#ifndef ROBORTS_DECISION_CHASE_BEHAVIOR_H
#define ROBORTS_DECISION_CHASE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"
#include "std_msgs/Int8.h"

namespace roborts_decision
{
class ChaseBehavior
{
public:
  ChaseBehavior(ChassisExecutor *&chassis_executor,
                GimbalExecutor *&gimbal_executor,
                Blackboard *&blackboard,
                const std::string &proto_file_path) : chassis_executor_(chassis_executor),
                                                      gimbal_executor_(gimbal_executor),
                                                      blackboard_(blackboard)
  { //因为 const 对象或引用类型只能初始化

    chase_goal_.header.frame_id = "map";
    chase_goal_.pose.orientation.x = 0;
    chase_goal_.pose.orientation.y = 0;
    chase_goal_.pose.orientation.z = 0;
    chase_goal_.pose.orientation.w = 1;

    chase_goal_.pose.position.x = 0;
    chase_goal_.pose.position.y = 0;
    chase_goal_.pose.position.z = 0;

    chase_buffer_.resize(2); //resize()的作用是改变vector中元素的数目,调整容器大小以使其包含2个元素。
    chase_count_ = 0;

    cancel_goal_ = true;
  }

  void Run()
  {

    auto chassis_executor_state = ChassisUpdate();
    auto gimbal_executor_state = GimbalUpdate();

    int execution_mode_ = chassis_executor_->GetExcutionMode();
    std::cout << "chase ExcutionMode:" << execution_mode_ << std::endl;

    if (execution_mode_ == 3)
    {
      chassis_executor_state = BehaviorState::IDLE;
    }
    auto robot_map_pose = blackboard_->GetRobotMapPose();

    chase_buffer_[chase_count_++ % 2] = blackboard_->GetEnemy();

    chase_count_ = chase_count_ % 2;
    std::cout << "chase_enemy_x:" << chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x << std::endl;
    std::cout << "chase_enemy_y:" << chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y << std::endl;
    auto dx = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x - robot_map_pose.pose.position.x;
    auto dy = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y - robot_map_pose.pose.position.y;

    auto enemy_yaw = std::atan2(chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y, chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x); //两点坐标求角度时，atan函数求角度时无方向（矢量），atan2函数求角度时有方向（矢量）

    //获取机器人与敌方相对x,y.这是以机器人自身为坐标系的相对
    auto relative_x = blackboard_->GetRelative_x();
    auto relative_y = blackboard_->GetRelative_y();
    auto relative_yaw = relative_y / relative_x;
    //ROS_INFO("chaserelative_x_:%f  chaserelative_y_:%f", relative_x, relative_y);
    std::cout << "relative_yaw:" << relative_yaw << std::endl;
    double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    ROS_INFO(" distance:%f", distance);

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(robot_map_pose.pose.orientation, quat);
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double robot_roll, robot_pitch, robot_yaw;
    tf::Matrix3x3(quat).getRPY(robot_roll, robot_pitch, robot_yaw);

    std::cout << "robort_yaw:" << robot_yaw << std::endl;
    //或者相对yaw,目地是使机器人追到敌方使,yaw和敌方一致,精确瞄准装甲板打击
    auto yaw = relative_yaw + robot_yaw;
    std::cout << "yaw:" << yaw << std::endl;

    //2m以内,底盘跟随云台,扭腰射击
    if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.0)
    {
      if (cancel_goal_)
      {
        chassis_executor_->Cancel();
        blackboard_->SetGimbalMode(0); //底盘跟随云台
        //控制射击,已经移交检测
        //blackboard_->CtrlShoot(1,1);

        blackboard_->CarTurn(1);
        ROS_INFO(" blackbord CarTurn true");
      }
      return;
    }
    //2m-2.5m
    else if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 2.0 && std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.5)
    {
      blackboard_->CarTurn(0);
      blackboard_->SetGimbalMode(1);
      return;
    }
    //2.5m之外追上敌方,下面代码多为代价地图
    else
    {
      blackboard_->CarTurn(0);
      blackboard_->SetGimbalMode(1);

      auto orientation = tf::createQuaternionMsgFromYaw(yaw);
      geometry_msgs::PoseStamped reduce_goal;
      reduce_goal.pose.orientation = orientation;

      reduce_goal.header.frame_id = "map";
      reduce_goal.header.stamp = ros::Time::now();
      reduce_goal.pose.position.x = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x - 1.2 * cos(enemy_yaw); //?1.2
      reduce_goal.pose.position.y = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y - 1.2 * sin(enemy_yaw);
      auto enemy_x = reduce_goal.pose.position.x;
      auto enemy_y = reduce_goal.pose.position.y;
      reduce_goal.pose.position.z = 0;

      unsigned int goal_cell_x, goal_cell_y;

      // if necessary add mutex lock
      //blackboard_->GetCostMap2D()->GetMutex()->lock();
      auto get_enemy_cell = blackboard_->GetCostMap2D()->World2Map(enemy_x,
                                                                   enemy_y,
                                                                   goal_cell_x,
                                                                   goal_cell_y);
      //blackboard_->GetCostMap2D()->GetMutex()->unlock();

      if (!get_enemy_cell)
      {
        return;
      }

      auto robot_x = robot_map_pose.pose.position.x;
      auto robot_y = robot_map_pose.pose.position.y;
      unsigned int robot_cell_x, robot_cell_y;
      double goal_x, goal_y;
      blackboard_->GetCostMap2D()->World2Map(robot_x,
                                             robot_y,
                                             robot_cell_x,
                                             robot_cell_y); //?

      if (blackboard_->GetCostMap2D()->GetCost(goal_cell_x, goal_cell_y) >= 253)
      {

        bool find_goal = false;
        for (FastLineIterator line(goal_cell_x, goal_cell_y, robot_cell_x, robot_cell_x); line.IsValid(); line.Advance())
        {

          auto point_cost = blackboard_->GetCostMap2D()->GetCost((unsigned int)(line.GetX()), (unsigned int)(line.GetY())); //current point's cost

          if (point_cost >= 253)
          {
            continue;
          }
          else
          {
            find_goal = true;
            blackboard_->GetCostMap2D()->Map2World((unsigned int)(line.GetX()),
                                                   (unsigned int)(line.GetY()),
                                                   goal_x,
                                                   goal_y);

            reduce_goal.pose.position.x = goal_x;
            reduce_goal.pose.position.y = goal_y;
            break;
          }
        }
        if (find_goal)
        {
          cancel_goal_ = true;
          chassis_executor_->Execute(reduce_goal);
        }
        else
        {
          if (cancel_goal_)
          {
            chassis_executor_->Cancel();
            cancel_goal_ = false;
          }
          return;
        }
      }
      else
      {
        cancel_goal_ = true;
        chassis_executor_->Execute(reduce_goal);
      }
    }
    //}
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

  void SetGoal(geometry_msgs::PoseStamped chase_goal)
  {
    chase_goal_ = chase_goal;
  }

  ~ChaseBehavior() = default;

private:
  //! executor
  ChassisExecutor *const chassis_executor_;
  GimbalExecutor *const gimbal_executor_;
  //! perception information
  Blackboard *const blackboard_;

  //! chase goal
  geometry_msgs::PoseStamped chase_goal_;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

  //! cancel flag
  bool cancel_goal_;

  // roborts_msgs::TwistAccel zero_twist_accel_;

  // //! velocity with accel publisher in ROS
  // ros::Publisher cmd_vel_acc_pub_;

  // //  geometry_msgs::Twist whirl_vel_;
  // roborts_msgs::TwistAccel whirl_vel_;
};
} // namespace roborts_decision

#endif //ROBORTS_DECISION_CHASE_BEHAVIOR_H
