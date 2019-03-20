#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
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

class DataUpdater {
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

DataUpdater::DataUpdater(ros::NodeHandle &nh) {
    carVelSub = nh.subscribe("cmd_vel", 1, &DataUpdater::carVelCallback, this);
    carQrSub = nh.subscribe("cmd_qr", 1, &DataUpdater::carQrCallback, this);
    ACCEPTTIME = 500000000; // 0.5s, unit: ns(10^-9s) TODO: this may need change
}

DataUpdater::~DataUpdater() {
}

void DataUpdater::carVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    carVel = msg;
    //ROS_INFO("car_vel: %.4f, %.4f, %.4f", msg->linear.x, msg->linear.y, msg->angular.z);
    carVelTime = ros::Time::now();
}
void DataUpdater::carQrCallback(const std_msgs::Bool &msg) {
    qr_scan_cmd=1;
}
geometry_msgs::Twist DataUpdater::getCarVel() {
    ros::Time nowTime = ros::Time::now();
    ros::Duration timediff = nowTime - carVelTime;
    if (carVel && timediff.nsec < ACCEPTTIME && timediff.sec == 0) // this message is new enough
        return *carVel;
    geometry_msgs::Twist dummyZero;
    //ROS_INFO("RETURN DUMMY");
    return dummyZero;
}

void genCmd(char *cmdData, short vx, short vy, short va) {
    void *tempP1 = &vx;
    void *tempP2 = &vy;
    void *tempP3 = &va;
    char *velxChar = (char *) tempP1;
    char *velyChar = (char *) tempP2;
    char *velaChar = (char *) tempP3;

    cmdData[0] = (char) 0x53;
    cmdData[1] = (char) 0x4A;
    cmdData[2] = (char) 0x54;
    cmdData[3] = (char) 0x55;
    cmdData[4] = velxChar[1]; //swap the high byte and the low byte
    cmdData[5] = velxChar[0];
    cmdData[6] = velyChar[1];
    cmdData[7] = velyChar[0];
    cmdData[8] = velaChar[1];
    cmdData[9] = velaChar[0];
    cmdData[10] = (char) 0x01; //

}

void velFilter(float &vx, float &vy, float &va, int &count, float &lastvx, float &lastvy, float &lastva) {
    if (vx == 0 && vy == 0 && va == 0) {
        count++;
        if (count > MAXZERONUM) {
            count = MAXZERONUM;
        } else {
            vx = lastvx;
            vy = lastvy;
            va = lastva;
            // ROS_INFO("*****FILT OUT********");
        }
    } else {
        count = 0;
        lastvx = vx;
        lastvy = vy;
        lastva = va;
    }
}

bool send_struct_command_serial(serial::Serial &ser) {
    struct_command_data command;
    struct_command_data *command_ptr = &command;
    char *command_ptr_char = (char *) command_ptr;

    uint8_t cmd_buff[150];
    int i = 0;
    for (i = 0; i < 3; i++) {
        command.speed_cmd[i] = speed_cmd[i];
        command.pos_cmd[i] = pos_cmd[i];
    }

    command.qr_scan_cmd = qr_scan_cmd;
    if(qr_scan_cmd!=0){
        qr_scan_cmd=0;
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
    ROS_INFO("%f %f %f",speed_cmd[0],speed_cmd[1],speed_cmd[2]);
    return true;
}

int main(int argc, char **argv) {
    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    ros::init(argc, argv, "agv_serial_server");
    ros::NodeHandle nh;
    DataUpdater dataUpdater(nh);
    ros::Rate loop_rate(FREQUENCY);

    bool serialflag = true;
    std::string datastr = "";
    datastr.clear();
// char feedback_buff[200];
    const char *front_fbk = "FBK";
    const char *back_fbk = "fbk";
// printf("FEEDBACK_DATA_LENGTH %d\n",FEEDBACK_DATA_LENGTH); 

    geometry_msgs::Twist carVelocity;
    int zeroCount = 0;
    float lastvx = 0;
    float lastvy = 0;
    float lastva = 0;
    float vx, vy, va;
    float velx, vely, vela;
    char cmd[COMMAND_SIZE + 1];
    cmd[COMMAND_SIZE] = '\0';


    while (ros::ok()) {

        carVelocity = dataUpdater.getCarVel();
        velx = (float) carVelocity.linear.x;
        vely = (float) carVelocity.linear.y;
        vela = (float) carVelocity.angular.z;
        vx = (float) (velx * 1000.0);
        vy = (float) (vely * 1000.0);
        va = (float) (vela * 1000.0);
        velFilter(vx, vy, va, zeroCount, lastvx, lastvy, lastva); // filt out single zero
        speed_cmd[0] = vx;
        speed_cmd[1] = vy;
        speed_cmd[2] = va;
//        genCmd(cmd, vx, vy, -va); // adjust the opsite direction
        string stringSend;
        try {
//            uint8_t *uint8_t_cmd_ptr = (uint8_t *) cmd;
//            ser.write(uint8_t_cmd_ptr, COMMAND_SIZE);
            // ROS_INFO("stringSend.length() %ld",stringSend.length());
            send_struct_command_serial(ser);
        } catch (serial::IOException &e) {
            ROS_INFO("Write error..");
        }

        datastr.clear();
        // ROS_INFO("%s", ser.available() ? "available" : "not available");
        datastr += ser.read(ser.available());
        if (datastr.length() > 0) {
            // cout<< datastr<<endl;
            // ROS_INFO("%d",datastr.length());
            const char *head = strstr(datastr.data(), front_fbk);
            // cout<<datastr.length()<<datastr.data()[FEEDBACK_DATA_LENGTH]<<endl;
            // printf("%s\n",datastr.data()+FEEDBACK_DATA_LENGTH+5);
            if (head != NULL) {
                // printf("%s\n",head);
                if (head[FEEDBACK_DATA_LENGTH + 3 + 0] == back_fbk[0] &&
                    head[FEEDBACK_DATA_LENGTH + 3 + 1] == back_fbk[1] &&
                    head[FEEDBACK_DATA_LENGTH + 3 + 2] == back_fbk[2]) {
                    struct_feedback_data *feedback_ptr = (struct_feedback_data *) (head + 3);
                    if (feedback_ptr->check_front_fbk == CHECK_FRONT_FBK &&
                        feedback_ptr->check_back_fbk == CHECK_BACK_FBK) {
                        ROS_INFO("Valid serial data recieved.");
                        ROS_INFO("speed %f %f %f", feedback_ptr->speed_fbk[0],
                                 feedback_ptr->speed_fbk[1], feedback_ptr->speed_fbk[2]);
                        ROS_INFO("positon %f %f %f", feedback_ptr->pos_fbk[0],
                                 feedback_ptr->pos_fbk[1], feedback_ptr->pos_fbk[2]);
                        ROS_INFO("imu_a %f %f %f", feedback_ptr->a_fbk[0],
                                 feedback_ptr->a_fbk[1], feedback_ptr->a_fbk[2]);
                        ROS_INFO("imu_g %f %f %f", feedback_ptr->g_fbk[0],
                                 feedback_ptr->g_fbk[1], feedback_ptr->g_fbk[2]);
                        ROS_INFO("ultra_sound %f %f %f %f %f %f %f %f %f %f %f %f",
                                 feedback_ptr->ultra_sound_signal_fbk[0],
                                 feedback_ptr->ultra_sound_signal_fbk[1],
                                 feedback_ptr->ultra_sound_signal_fbk[2],
                                 feedback_ptr->ultra_sound_signal_fbk[3],
                                 feedback_ptr->ultra_sound_signal_fbk[4],
                                 feedback_ptr->ultra_sound_signal_fbk[5],
                                 feedback_ptr->ultra_sound_signal_fbk[6],
                                 feedback_ptr->ultra_sound_signal_fbk[7],
                                 feedback_ptr->ultra_sound_signal_fbk[8],
                                 feedback_ptr->ultra_sound_signal_fbk[9],
                                 feedback_ptr->ultra_sound_signal_fbk[10],
                                 feedback_ptr->ultra_sound_signal_fbk[11]);
                        ROS_INFO("qr_code %s\n", feedback_ptr->qr_scan_fbk);
                    }
                    // datastr.clear();
                }
            }
        }
        // ROS_INFO("loop end.");

        // ROS_INFO("end");
        // }

        // serialflag = !serialflag;
        // string stringSend;
        // char charSend[50];
        // sprintf(charSend, "ANGLEUTD%dANGLEUTD\n\r", (int) (1 * 10));

        // // ROS_INFO("%s", charSend);
        // stringSend = charSend;
        // ser.write(stringSend);

        ros::spinOnce();

        loop_rate.sleep();
    }
}
