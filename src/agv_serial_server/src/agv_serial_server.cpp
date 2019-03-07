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


// bool call_ur_twist(jrc_srvs::call_twist::Request &req,
//                    jrc_srvs::call_twist::Response &res) {
// //  ros::NodeHandle n;

// //  ROS_INFO("%s",ser.available()?"available":"not available");
//     if (1) {
//         ee_angle_send = req.angle;

//         ROS_INFO("Called twist %f", req.angle);
//     } else {
//         ROS_ERROR("Serial unavailable!");
//         return -1;
//     }
//     return true;
// }

// bool call_ur_grasp(jrc_srvs::call_grasp::Request &req,
//                    jrc_srvs::call_grasp::Response &res) {
//     ros::NodeHandle n;
//     ros::ServiceClient client =
//             n.serviceClient<ur_msgs::SetIO>("/ur_driver/set_io");
//     ur_msgs::SetIO srv;
//     int state = req.grasp ? 0 : 1;

//     srv.request.fun = (int8_t) 1;
//     srv.request.pin = (int8_t) 0;
//     srv.request.state = state;
//     if (client.call(srv)) {
//         ROS_INFO("Called ur io. fun %d pin %d state %f", srv.request.fun,
//                  srv.request.pin, srv.request.state);

//     } else {
//         ROS_ERROR("Failed to call ur io");
//         return 1;
//     }

//     // usleep(1000000);//+1s

//     // res.grasped=ur_grasped;
//     res.acted = true;
//     return true;
// }


// bool call_ur_grasp_state(jrc_srvs::call_grasp_state::Request &req,
//                          jrc_srvs::call_grasp_state::Response &res) {
// //  ros::NodeHandle n;

// //  ROS_INFO("%s",ser.available()?"available":"not available");
//     if (1) {
//         res.grasped = ur_grasped;
//         ROS_INFO("Return grasp_state %s", ur_grasped ? "true" : "false");
//     } else {
//         ROS_ERROR("Serial unavailable!");
//         return -1;
//     }
//     return true;
// }

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

    
    ros::Rate loop_rate(10);
    ros::NodeHandle n;

    bool serialflag = true;
std::string datastr="";
datastr.clear();
char feedback_buff[100];
    while (ros::ok()) {
        if (serialflag) {
            ROS_INFO("%s", ser.available() ? "available" : "not available");
            datastr += ser.read(ser.available());
            ROS_INFO("%s", datastr.data());
int success_num_read=sscanf(datastr.data(), "%*s\nFBK%sfbk", &feedback_buff);
            datastr.clear();
        }
struct_feedback_data* struct_feedback_data_ptr=(struct_feedback_data*)feedback_buff;


        serialflag = !serialflag;
        string stringSend;
        char charSend[50];
        sprintf(charSend, "ANGLEUTD%dANGLEUTD\n\r", (int) (1 * 10));

        // ROS_INFO("%s", charSend);
        stringSend = charSend;
        // ser.write(stringSend);

        ros::spinOnce();

        loop_rate.sleep();
    }
}
