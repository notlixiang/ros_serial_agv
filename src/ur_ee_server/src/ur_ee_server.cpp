#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <string>

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

double ee_angle_send = 0;
bool ur_grasped = false;

bool call_ur_twist(jrc_srvs::call_twist::Request &req,
                   jrc_srvs::call_twist::Response &res) {
//  ros::NodeHandle n;

//  ROS_INFO("%s",ser.available()?"available":"not available");
    if (1) {
        ee_angle_send = req.angle;

        ROS_INFO("Called twist %f", req.angle);
    } else {
        ROS_ERROR("Serial unavailable!");
        return -1;
    }
    return true;
}


//void pressureStateCallback(const ur_msgs::IOStates &msg) {
//    if ((msg.digital_in_states[16]).pin == 16) {
//        if ((msg.digital_in_states[16]).state == true) {
//            ur_grasped = true;
//            ROS_INFO("ur_grasped true");
//        } else {
//            ur_grasped = false;
//            ROS_INFO("ur_grasped false");
//        }
//
//    } else {
//        ROS_INFO("PIN is not 16 PIN: %d", (msg.digital_in_states[16]).pin);
//    }
//}

bool call_ur_grasp(jrc_srvs::call_grasp::Request &req,
                   jrc_srvs::call_grasp::Response &res) {
    ros::NodeHandle n;
    ros::ServiceClient client =
            n.serviceClient<ur_msgs::SetIO>("/ur_driver/set_io");
    ur_msgs::SetIO srv;
    int state = req.grasp ? 0 : 1;

    srv.request.fun = (int8_t) 1;
    srv.request.pin = (int8_t) 0;
    srv.request.state = state;
    if (client.call(srv)) {
        ROS_INFO("Called ur io. fun %d pin %d state %f", srv.request.fun,
                 srv.request.pin, srv.request.state);

    } else {
        ROS_ERROR("Failed to call ur io");
        return 1;
    }

    // usleep(1000000);//+1s

    // res.grasped=ur_grasped;
    res.acted = true;
    return true;
}


bool call_ur_grasp_state(jrc_srvs::call_grasp_state::Request &req,
                         jrc_srvs::call_grasp_state::Response &res) {
//  ros::NodeHandle n;

//  ROS_INFO("%s",ser.available()?"available":"not available");
    if (1) {
        res.grasped = ur_grasped;
        ROS_INFO("Return grasp_state %s", ur_grasped ? "true" : "false");
    } else {
        ROS_ERROR("Serial unavailable!");
        return -1;
    }
    return true;
}

int main(int argc, char **argv) {
    try {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(9600);
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

    ros::init(argc, argv, "ur_ee_server");
    ros::NodeHandle nh;

    ros::ServiceServer service_twist = nh.advertiseService("call_twist", call_ur_twist);
    ROS_INFO("Ready to twist.");
    ros::ServiceServer service_grasp = nh.advertiseService("call_grasp", call_ur_grasp);
    ros::ServiceServer service_grasp_state = nh.advertiseService("call_grasp_state", call_ur_grasp_state);
    ROS_INFO("Ready to grasp.");


    ros::Rate loop_rate(10);
    ros::NodeHandle n;

    ros::Publisher twist_angle_pub =
            n.advertise<std_msgs::Float32>("/twist_angle", 1000);
    std_msgs::Float32 twist_angle_msg;
    std::string inString = "";

    ros::Publisher grasp_state_pub =
            n.advertise<std_msgs::Bool>("/grasp_state", 1000);
    std_msgs::Bool grasp_state_msg;

    float ee_angle = 0;

    int sucked = 0;

    int ee_angle_int = 0;
    bool serialflag = true;
std::string datastr="";
datastr.clear();
    while (ros::ok()) {
        if (serialflag) {
            ROS_INFO("%s", ser.available() ? "available" : "not available");
            datastr += ser.read(ser.available());
            // ROS_INFO("%s", datastr.data());

            int success_num_read=sscanf(datastr.data(), "%*s\nANGLEDTU%dSUCKED%dOVER", &ee_angle_int, &sucked);
// if(success_num_read==2)
// {
datastr.clear();
// }

            
// ROS_INFO("success_num_read %d", success_num_read);
            ur_grasped = (bool) sucked;
            ee_angle = ee_angle_int / 10;

            twist_angle_msg.data = ee_angle;
            twist_angle_pub.publish(twist_angle_msg);
        }
        serialflag = !serialflag;
        string stringSend;
        char charSend[50];
        sprintf(charSend, "ANGLEUTD%dANGLEUTD\n\r", (int) (ee_angle_send * 10));

        ROS_INFO("%s", charSend);
        stringSend = charSend;
        ser.write(stringSend);


        grasp_state_msg.data = ur_grasped;
        grasp_state_pub.publish(grasp_state_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
}
