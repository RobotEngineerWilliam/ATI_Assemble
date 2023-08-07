//
// Created by zhao on 2023/6/30.
//

#ifndef SRC_TRAPEZOID_PLAN_H
#define SRC_TRAPEZOID_PLAN_H

#include <vector>
#include "libs/jktypes.h"

typedef float Joint_Value[6];
typedef int Joint_Flag[6];


/**
* @brief    单自由度梯形速度轨迹规划，若距离过短则采用均分规划
* @param start    起始关节位置
* @param end      结束关节位置
* @param start_vel 起始速度
* @param end_vel 终点速度
* @param speed_c 巡航速度
* @param acc_times 加速时间(点数目)
* @param dec_times 减速时间(点数目)
* @param total_times 总时间(点数目)
* @param trajectory 输出轨迹
*/
void Single_Trapezoid_Series(double start, double end, float start_vel, float end_vel,
                             float speed_c, int acc_times, int dec_times, int total_times,
                             std::vector<float>& trajectory);


/**
* @brief    单自由度均分规划
* @param start    起始关节位置
* @param end      结束关节位置
* @param total_times 总点数
* @param trajectory 输出轨迹
*/
void Single_Average_Series(float start, float end, int total_times, std::vector<float>& trajectory);


/**
* @brief    梯形速度轨迹规划，若距离过短则采用均分规划
* @param joint_start    起始关节位置
* @param joint_end      结束关节位置
* @param trajectory 输出轨迹序列
* @param vel_lim 关节速度限制
* @param acc_lim 关节加速度限制
* @param start_vel 起始速度
* @param end_vel 终止速度
*/
void Trapezoid_Velocity_Series(JointValue joint_start, JointValue joint_end, std::vector<JointValue>& trajectory,
                               Joint_Value vel_lim, Joint_Value acc_lim, Joint_Value start_vel = 0, Joint_Value end_vel = 0);

#endif //SRC_TRAPEZOID_PLAN_H
