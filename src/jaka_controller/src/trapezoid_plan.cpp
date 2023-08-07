//
// Created by zhao on 2023/6/30.
//

#include "jaka_controller/trapezoid_plan.h"
#include "cmath"


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
                             std::vector<float>& trajectory)
{
    float path_point;
    float deta;
    float vel;
    int t = 0;
    int dir = 0;

    path_point = start;
    vel = start_vel;
    trajectory.clear();

    dir = (end - start) > 0 ? 1 : -1;

    while(t < total_times)//total times
    {
        if(t < acc_times)//acc times
        {
            vel = vel + (speed_c - start_vel) / acc_times;
        }
        else if(t >= acc_times && t < (total_times - dec_times))
        {
            vel = speed_c;
        }
        else
        {
            vel = vel - (speed_c - end_vel) / dec_times;
        }
        //std::cout << vel << std::endl;

        deta = fabs(end - path_point) > vel ? dir * vel : end - path_point;
        path_point = path_point + deta;

        trajectory.push_back(path_point);

        t++;
    }
    //std::cout << "Trapezoid Series Complete!" << std::endl;
}

/**
* @brief    单自由度均分规划
* @param start    起始关节位置
* @param end      结束关节位置
* @param total_times 总点数
* @param trajectory 输出轨迹
*/
void Single_Average_Series(float start, float end, int total_times, std::vector<float>& trajectory)
{
    float path_point;
    float deta;
    int t = 0;

    path_point = start;
    deta = (end - start) / total_times;
    trajectory.clear();

    while(t < total_times)
    {
        path_point = path_point + deta;

        trajectory.push_back(path_point);

        t++;
    }
    //std::cout << "Average Series Complete!" << std::endl;
}


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
                               Joint_Value vel_lim, Joint_Value acc_lim, Joint_Value start_vel, Joint_Value end_vel)
{
    trajectory.clear();
    Joint_Value length, path_point;
    Joint_Flag joint_trapezoid;

    //Length of acceleration and deceleration
    Joint_Value acc_length, dec_length;
    int longest_index = 0;
    float time;
    int Times;
    int Acc_Times, Dec_Times;
    float len = 0;

    std::vector<float> path_series[6];


    //Find the longest joint
    for(int i = 0; i < 6; i++)
    {
        //Caculate the length of acceleration and deceleration
        acc_length[i] = (vel_lim[i] * vel_lim[i] - start_vel[i] * start_vel[i]) / (2 * acc_lim[i]);
        dec_length[i] = (vel_lim[i] * vel_lim[i] - end_vel[i] * end_vel[i]) / (2 * acc_lim[i]);

        length[i] = fabs(joint_end.jVal[i] - joint_start.jVal[i]);

        if(length[i] > len)
        {
            longest_index = i;
            len = length[i];
        }

        if(length[i] < acc_length[i] + dec_length[i])
        {
            joint_trapezoid[i] = 0;
        }
        else
        {
            joint_trapezoid[i] = 1;
        }
        //std::cout << " " << i << " " << joint_trapezoid[i] << " " ;
        //std::cout << " l " << length[i] << " a " << acc_length[i] << " d " << dec_length[i] << std::endl;
    }

    //Caculate times if the longest is Trapezoid
    if(joint_trapezoid[longest_index] == 1)
    {/*
        time = (vel_lim[longest_index] - start_vel[longest_index]) / acc_lim[longest_index] +
                (vel_lim[longest_index] - end_vel[longest_index]) / acc_lim[longest_index] +
                (length[longest_index] - acc_length[longest_index] - dec_length[longest_index]) / vel_lim[longest_index]; */

        Acc_Times = static_cast<int>((vel_lim[longest_index] - start_vel[longest_index]) / acc_lim[longest_index] + 1);
        Dec_Times = static_cast<int>((vel_lim[longest_index] - end_vel[longest_index]) / acc_lim[longest_index] + 1);
        //Acc_Times = static_cast<int>((vel_lim[longest_index] - start_vel[longest_index]) / acc_lim[longest_index]);
        //Dec_Times = static_cast<int>((vel_lim[longest_index] - end_vel[longest_index]) / acc_lim[longest_index]);


        time = Acc_Times + Dec_Times +
               (length[longest_index] - acc_length[longest_index] - dec_length[longest_index]) / vel_lim[longest_index];

        //std::cout << " " << i << " " << joint_trapezoid[i] << " " ;
        //std::cout << " t " << time << " a " << Acc_Times << " d " << Dec_Times << std::endl;
    }
    else
    {
        /* time = (vel_lim[longest_index] - start_vel[longest_index]) / acc_lim[longest_index] +
                (vel_lim[longest_index] - end_vel[longest_index]) / acc_lim[longest_index]; */

        time = 2 * length[longest_index] / (start_vel[longest_index] + end_vel[longest_index]);
    }

    //Times of trajectory
    //Times = static_cast<int>(time + 1);
    Times = static_cast<int>(time);

    //std::cout << " " << Times << std::endl;

    //Planning the trajectory
    for(int i = 0; i < 6; i++)
    {
        if(joint_trapezoid[i] == 1)
        {

            /* if(i == longest_index)
            {
                Single_Trapezoid_Series(joint_start.jVal[i], joint_end.jVal[i], start_vel[i], end_vel[i], vel_lim[i], Acc_Times, Dec_Times, Times, path_series[i]);
            }
            else
            { */
            float cruise_vel;

            cruise_vel = fabs((2 * fabs(joint_end.jVal[i] - joint_start.jVal[i]) - start_vel[i] * Acc_Times - end_vel[i] * Dec_Times) / (2 * Times - Acc_Times - Dec_Times));

            Single_Trapezoid_Series(joint_start.jVal[i], joint_end.jVal[i], start_vel[i], end_vel[i], cruise_vel, Acc_Times, Dec_Times, Times, path_series[i]);
            //}
        }
        else
        {
            Single_Average_Series(joint_start.jVal[i], joint_end.jVal[i], Times, path_series[i]);
        }
    }

    for(int i = 0; i < path_series[longest_index].size(); i++)
    {
        JointValue joint_data;
        //joint_data.clear();
        for(int j = 0; j < 6; j++)
        {
            joint_data.jVal[j] = path_series[j][i];
        }

        trajectory.push_back(joint_data);
    }

    //std::cout << "Planning Complete!" << std::endl;
}




