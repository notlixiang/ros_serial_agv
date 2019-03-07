#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <string>

#include <StructSerial.h>

#define USHORT unsigned short int
#define BYTE unsigned char
#define CHAR char
#define UCHAR unsigned char
#define UINT unsigned int
#define DWORD unsigned int
#define PVOID void *
#define ULONG unsigned int
#define INT int
#define UINT32 UINT
#define LPVOID void *
#define BOOL BYTE
#define TRUE 1
#define FALSE 0

serial::Serial ser;
using namespace std;

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

    
    ros::Rate loop_rate(20);
    ros::NodeHandle n;

    bool serialflag = true;
    std::string datastr="";
    datastr.clear();
// char feedback_buff[200];
    const char* front_fbk="FBK";
    const char* back_fbk="fbk";
// printf("FEEDBACK_DATA_LENGTH %d\n",FEEDBACK_DATA_LENGTH);  
    while (ros::ok()) {
        // ROS_INFO("start");
        // if (serialflag) {
        datastr.clear();
            // ROS_INFO("%s", ser.available() ? "available" : "not available");
        datastr += ser.read(ser.available());            

        const char* head = strstr(datastr.data(),front_fbk);
            // cout<<datastr.length()<<datastr.data()[FEEDBACK_DATA_LENGTH]<<endl;
          // printf("%s\n",datastr.data()+FEEDBACK_DATA_LENGTH+5);
          // printf("%s\n",head);
        if(head!=NULL){
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
                }
            // datastr.clear();
            }
        }

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
