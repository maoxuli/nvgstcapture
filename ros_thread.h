#ifndef __ROS_THREAD_H
#define __ROS_THREAD_H

#include <ros/ros.h> 

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

class RosThread 
{
public: 
    RosThread(); 

private: 
    int _argc; 
    char** _argv; 
    std::string _node_name; 
    ros::NodeHandle _private_nh; 

    std::string _recording_control_service_name; 
    ros::ServiceServer _recording_control_service; 
    bool recording_control_callback(std_srvs::SetBool::Request  &request,
                                    std_srvs::SetBool::Response &response); 

    std::atomic<bool> _stop; 
    std::thread _thread; 
    void node_thread(); 

    GIOChannel* _channel;

}; 

#endif 
