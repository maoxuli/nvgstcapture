#ifndef __ROS_NODE_H
#define __ROS_NODE_H

#include <ros/ros.h> 

#include <thread>
#include <mutex>
#include <condition_variable>

class RosNode 
{
public: 
    RosNode(int argc, char** argv); 
    ~RosNode(); 

    void update_recording_status(); 

private: 
    ros::Publisher _recording_status_pub;

    // dynmic reconfigure server 
    dynamic_reconfigure::Server<camera::CaptureParamsConfig> _cpature_params_srv;
    void capture_params_allback(camera::CaptureParamsConfig& config, uint32_t level);

    ros::ServiceServer _recording_control_srv; 
    bool recording_control_callback(std_srvs::SetBool::Request  &request,
                                    std_srvs::SetBool::Response &response); 

    std::thread _thread; 
    void work_thread(); 
}; 

#endif 
