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

float speed_cmd[3]={0};
float pos_cmd[3]={0};
uint8_t  qr_scan_cmd=0;

class DataUpdater
{
public:
    DataUpdater(ros::NodeHandle &nh);
    ~DataUpdater();
    geometry_msgs::Twist getCarVel();

private:
    // callback functions
    void carVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    // subscribers
    ros::Subscriber carVelSub;

    // data
    geometry_msgs::Twist::ConstPtr carVel; //boost::shared_ptr<const geometry_msgs::Twist_<std::allocator<void> > > 
    // the latest time that recieve a message
    ros::Time carVelTime;
    int ACCEPTTIME;
};

DataUpdater::DataUpdater(ros::NodeHandle &nh)
{
    carVelSub = nh.subscribe("cmd_vel",1,&DataUpdater::carVelCallback,this);
    ACCEPTTIME = 500000000; // 0.5s, unit: ns(10^-9s) TODO: this may need change
}

DataUpdater::~DataUpdater()
{
}

void DataUpdater::carVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    carVel = msg;
    //ROS_INFO("car_vel: %.4f, %.4f, %.4f", msg->linear.x, msg->linear.y, msg->angular.z);
    carVelTime = ros::Time::now();
}

geometry_msgs::Twist DataUpdater::getCarVel()
{
    ros::Time nowTime = ros::Time::now();
    ros::Duration timediff = nowTime-carVelTime;
    if(carVel && timediff.nsec<ACCEPTTIME && timediff.sec==0 ) // this message is new enough
        return *carVel;
    geometry_msgs::Twist dummyZero;
    //ROS_INFO("RETURN DUMMY");
    return dummyZero;
}

void genCmd(char* cmdData, short vx, short vy, short va)
{
    void* tempP1 = &vx;
    void* tempP2 = &vy;
    void* tempP3 = &va;
    char* velxChar = (char*)tempP1;
    char* velyChar = (char*)tempP2;
    char* velaChar = (char*)tempP3;

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

void velFilter(short &vx, short &vy, short &va, int &count, short &lastvx, short &lastvy, short &lastva)
{
    if(vx==0 && vy==0 && va==0)
    {
        count++;
        if(count>MAXZERONUM)
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
    std::string datastr="";
    datastr.clear();
// char feedback_buff[200];
    const char* front_fbk="FBK";
    const char* back_fbk="fbk";
// printf("FEEDBACK_DATA_LENGTH %d\n",FEEDBACK_DATA_LENGTH); 

    geometry_msgs::Twist carVelocity;
    int zeroCount = 0;
    short lastvx=0;
    short lastvy=0;
    short lastva=0;
    short vx,vy,va;
    double velx,vely,vela;
    char cmd[COMMAND_SIZE+1];
    cmd[COMMAND_SIZE]='\0';


    while (ros::ok()) {

        carVelocity = dataUpdater.getCarVel();
        velx = carVelocity.linear.x;
        vely = carVelocity.linear.y;
        vela = carVelocity.angular.z;
        vx = (int)(velx*1000.0);
        vy = (int)(vely*1000.0);
        va = (int)(vela*1000.0);
        velFilter(vx,vy,va,zeroCount,lastvx,lastvy,lastva); // filt out single zero
        genCmd(cmd,vx,vy,-va); // adjust the opsite direction
        string stringSend;
        try{
            uint8_t* uint8_t_cmd_ptr = (uint8_t*)cmd;
            ser.write(uint8_t_cmd_ptr,COMMAND_SIZE);
            // ROS_INFO("stringSend.length() %ld",stringSend.length());
            // ROS_INFO("Send cmd %02x, %02x, %02x, %02x, %02x, %02x.",(unsigned char)cmd[4],(unsigned char)cmd[5],(unsigned char)cmd[6],(unsigned char)cmd[7],(unsigned char)cmd[8],(unsigned char)cmd[9]);
        }catch(serial::IOException &e)
        {
          ROS_INFO("Write error..");
        }

        datastr.clear();
            // ROS_INFO("%s", ser.available() ? "available" : "not available");
        datastr += ser.read(ser.available());         
        if(datastr.length()>0){
        // cout<< datastr<<endl;
            // ROS_INFO("%d",datastr.length());
            const char* head = strstr(datastr.data(),front_fbk);
            // cout<<datastr.length()<<datastr.data()[FEEDBACK_DATA_LENGTH]<<endl;
          // printf("%s\n",datastr.data()+FEEDBACK_DATA_LENGTH+5);
            if(head!=NULL){
           // printf("%s\n",head);
                if(head[FEEDBACK_DATA_LENGTH+3+0]==back_fbk[0]&&
                    head[FEEDBACK_DATA_LENGTH+3+1]==back_fbk[1]&&
                    head[FEEDBACK_DATA_LENGTH+3+2]==back_fbk[2])
                {
                    struct_feedback_data* feedback_ptr=(struct_feedback_data*)(head+3);
                    if(feedback_ptr->check_front_fbk==CHECK_FRONT_FBK&&
                        feedback_ptr->check_back_fbk==CHECK_BACK_FBK){
                        ROS_INFO("Valid serial data recieved.");
                    ROS_INFO("speed %f %f %f",feedback_ptr->speed_fbk[0],
                        feedback_ptr->speed_fbk[1],feedback_ptr->speed_fbk[2] );
                    ROS_INFO("positon %f %f %f",feedback_ptr->pos_fbk[0],
                        feedback_ptr->pos_fbk[1],feedback_ptr->pos_fbk[2] );
                    ROS_INFO("imu_a %f %f %f",feedback_ptr->a_fbk[0],
                        feedback_ptr->a_fbk[1],feedback_ptr->a_fbk[2] );
                    ROS_INFO("imu_g %f %f %f",feedback_ptr->g_fbk[0],
                        feedback_ptr->g_fbk[1],feedback_ptr->g_fbk[2] );
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
                    ROS_INFO("qr_code %s\n",feedback_ptr->qr_scan_fbk );
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
