#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "IKSolverInterface.h"
#include <string>
#include <iostream>
#include <fstream>
#include <array>
#include <vector>

using namespace std;

typedef void* IKSolver;


string readFile(string fp) {
    
    string line;
    string allText = "";
    ifstream myfile (fp);
    
    if (myfile.is_open()) {
        while (getline(myfile, line))
            allText += line;
        myfile.close();
    }

    return allText;
}


void poseToArray(const geometry_msgs::Pose::ConstPtr& msg, array<double, 7>& trans) {
    trans[0] = msg->position.x;
    trans[1] = msg->position.y;
    trans[2] = msg->position.z;

    trans[3] = msg->orientation.w;
    trans[4] = msg->orientation.x;
    trans[5] = msg->orientation.y;
    trans[6] = msg->orientation.z;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_free_ik");
    ros::NodeHandle n;



    string urdf_ptr, ee_frame_ptr, arm_colliders_ptr, environment_ptr;
    string urdf_fp, arm_collider_fp, environment_fp;
    vector<double> current_q;
    n.getParam("/urdf_fp", urdf_fp);
    n.getParam("/ee_frame", ee_frame_ptr);
    n.getParam("/arm_collider_fp", arm_collider_fp);
    n.getParam("/environment_fp", environment_fp);
    n.getParam("/initial_q", current_q);

    urdf_ptr = readFile(urdf_fp);
    arm_colliders_ptr = readFile(arm_collider_fp);
    environment_ptr = readFile(environment_fp);

    IKSolver iksolver = new_solver(urdf_ptr.c_str(), ee_frame_ptr.c_str(), arm_colliders_ptr.c_str(), environment_ptr.c_str());
    if (current_q.size() != dof(iksolver)) {
        deallocate(iksolver);
        return 1;
    }
    
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("out", 1);
    array<double, 7> trans_ptr = array<double, 7>();
    ros::Subscriber sub = n.subscribe<geometry_msgs::Pose>("in", 1, [&](const geometry_msgs::Pose::ConstPtr& msg) {

        poseToArray(msg, trans_ptr);

        vector<double> q_ptr = vector<double>();
        for (size_t i = 0; i < dof(iksolver); i++) {
            q_ptr.push_back(0);
        }

        if (solve(iksolver, current_q.data(), trans_ptr, q_ptr.data())) {
            for (size_t i = 0; i < current_q.size(); i++) {
                current_q[i] = q_ptr[i];
            }

            auto out_msg = std_msgs::Float64MultiArray();
            out_msg.data = q_ptr;

            pub.publish(out_msg);
        }
        
    });

    ros::spin();
    return 0;
}