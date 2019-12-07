#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <string>
#include "geometry_msgs/Twist.h"

#include <StructSerial.h>

#define FREQUENCY 10
#define COMMAND_SIZE 11

#define MAXZERONUM 5

serial::Serial ser;
using namespace std;

float speed_cmd[3] = {0};
float pos_cmd[3] = {0};
uint8_t qr_scan_cmd = 0;

const char *mystrstr(const char *pOri, int OriNum, const char *pFind, int FindNum)
{
    // char *p = NULL;
    if (OriNum < FindNum)
        return NULL;
    else
    {
        int i = 0, j = 0, Match = 0;
        for (i = 0; i < OriNum && FindNum + i + 1 <= OriNum; i++)
        {
            int e = i;
            for (j = 0; j < FindNum; j++)
            {
                if (!memcmp(pFind + j, pOri + e, 1))
                {
                    e++;
                    Match++;
                }
                else
                    Match = 0;
            }
            if (Match == FindNum)
            {
                return (pOri + i);
                break;
            }
        }
    }

    return NULL;
}

class DataUpdater
{
public:
    DataUpdater(ros::NodeHandle &nh);

    ~DataUpdater();

    geometry_msgs::Twist getCarVel();

private:
    // callback functions
    void carVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

    void carQrCallback(const std_msgs::Bool &msg);

    // subscribers
    ros::Subscriber carVelSub;
    ros::Subscriber carQrSub;

    // data
    geometry_msgs::Twist::ConstPtr carVel; //boost::shared_ptr<const geometry_msgs::Twist_<std::allocator<void> > >
    // the latest time that recieve a message
    ros::Time carVelTime;
    int ACCEPTTIME;
};

DataUpdater::DataUpdater(ros::NodeHandle &nh)
{
    carVelSub = nh.subscribe("cmd_vel", 1, &DataUpdater::carVelCallback, this);
    carQrSub = nh.subscribe("cmd_qr", 1, &DataUpdater::carQrCallback, this);
    ACCEPTTIME = 500000000; // 0.5s, unit: ns(10^-9s) TODO: this may need change
}

DataUpdater::~DataUpdater()
{
}

void DataUpdater::carVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    carVel = msg;
    //ROS_INFO("car_vel: %.4f, %.4f, %.4f", msg->linear.x, msg->linear.y, msg->angular.z);
    carVelTime = ros::Time::now();
}

void DataUpdater::carQrCallback(const std_msgs::Bool &msg)
{
    qr_scan_cmd = 1;
}

geometry_msgs::Twist DataUpdater::getCarVel()
{
    ros::Time nowTime = ros::Time::now();
    ros::Duration timediff = nowTime - carVelTime;
    if (carVel && timediff.nsec < ACCEPTTIME && timediff.sec == 0) // this message is new enough
        return *carVel;
    geometry_msgs::Twist dummyZero;
    //ROS_INFO("RETURN DUMMY");
    return dummyZero;
}

void genCmd(char *cmdData, short vx, short vy, short va)
{
    void *tempP1 = &vx;
    void *tempP2 = &vy;
    void *tempP3 = &va;
    char *velxChar = (char *)tempP1;
    char *velyChar = (char *)tempP2;
    char *velaChar = (char *)tempP3;

    cmdData[0] = (char)0x53;
    cmdData[1] = (char)0x4A;
    cmdData[2] = (char)0x54;
    cmdData[3] = (char)0x55;
    cmdData[4] = velxChar[1]; //swap the high byte and the low byte
    cmdData[5] = velxChar[0];
    cmdData[6] = velyChar[1];
    cmdData[7] = velyChar[0];
    cmdData[8] = velaChar[1];
    cmdData[9] = velaChar[0];
    cmdData[10] = (char)0x01; //
}

void velFilter(float &vx, float &vy, float &va, int &count, float &lastvx, float &lastvy, float &lastva)
{
    if (vx == 0 && vy == 0 && va == 0)
    {
        count++;
        if (count > MAXZERONUM)
        {
            count = MAXZERONUM;
        }
        else
        {
            vx = lastvx;
            vy = lastvy;
            va = lastva;
            // ROS_INFO("*****FILT OUT********");
        }
    }
    else
    {
        count = 0;
        lastvx = vx;
        lastvy = vy;
        lastva = va;
    }
}

bool send_struct_command_serial(serial::Serial &ser)
{
    struct_command_data command;
    struct_command_data *command_ptr = &command;
    char *command_ptr_char = (char *)command_ptr;

    uint8_t cmd_buff[150];
    int i = 0;
    for (i = 0; i < 3; i++)
    {
        command.speed_cmd[i] = speed_cmd[i];
        command.pos_cmd[i] = pos_cmd[i];
    }

    command.qr_scan_cmd = qr_scan_cmd;
    ROS_INFO("qr_scan_cmd %d", qr_scan_cmd);
    if (qr_scan_cmd != 0)
    {
        qr_scan_cmd = 0;
    }
    command.check_front_cmd = CHECK_FRONT_CMD;
    command.check_back_cmd = CHECK_BACK_CMD;

    cmd_buff[0] = 'C';
    cmd_buff[1] = 'M';
    cmd_buff[2] = 'D';
    cmd_buff[COMMAND_DATA_LENGTH + 3 + 0] = 'c';
    cmd_buff[COMMAND_DATA_LENGTH + 3 + 1] = 'm';
    cmd_buff[COMMAND_DATA_LENGTH + 3 + 2] = 'd';
    cmd_buff[COMMAND_DATA_LENGTH + 3 + 3] = '\0';
    cmd_buff[COMMAND_DATA_LENGTH + 3 + 4] = '\n';

    memcpy(cmd_buff + 3, command_ptr_char, COMMAND_DATA_LENGTH);

    ser.write(cmd_buff, COMMAND_DATA_LENGTH + 8);
    // ROS_INFO("COMMAND_DATA_LENGTH + 8 %ld",COMMAND_DATA_LENGTH + 8);
    // ROS_INFO("%d %d %d %d %d %d %d %d",(uint8_t)cmd_buff[0]
    //     ,(uint8_t)cmd_buff[1]
    //     ,(uint8_t)cmd_buff[2]
    //     ,(uint8_t)cmd_buff[3]
    //     ,(uint8_t)cmd_buff[4]
    //     ,(uint8_t)cmd_buff[5]
    //     ,(uint8_t)cmd_buff[6]
    //     ,(uint8_t)cmd_buff[7]);
    ROS_INFO("speed command %f %f %f", speed_cmd[0], speed_cmd[1], speed_cmd[2]);
    return true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "agv_serial_server");
    ros::NodeHandle nh;
    DataUpdater dataUpdater(nh);
    ros::Rate loop_rate(FREQUENCY);

    ros::Publisher int_publisher = nh.advertise<std_msgs::Int32>("int_test", 1000);
    std_msgs::Int32 int_msg;
    int iter=0;
    bool flag = true;
    while (ros::ok())
    {
        int_msg.data = iter++;
        int_publisher.publish(int_msg);
        ROS_INFO("testing..");
        ros::spinOnce();

        loop_rate.sleep();
    }
}
