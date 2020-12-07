#include "ros_thread.h"

RosThread::RosThread(int argc, char* argv[], const std::string& node_name)
: _argc(argc)
, _argv(argv)
, _node_name(node_name)
{
    // channel of standard IO
    _channel = g_io_channel_unix_new (0);

    // ROS node is initialized in thread 
    std::cout << "ROS thread starting" << std::endl; 
    _stop = false; 
    _thread = std::thread(std::bind(&RosThread::node_thread, this));
}

RosThread::~RosThread()
{
    if (_thread.joinable()) {
        std::cout << "ROS thread stopping" << std::endl; 
        _stop = true; 
        _thread.join();
        std::cout << "ROS thread stopped!" << std::endl; 
    }
}

void RosThread::node_thread() 
{
    std::cout << "thread in" << std::endl; 
    ros::init(_argc, _argv, _node_name); 
    ros::start(); 

    _private_nh = ros::NodeHandle("~"); 

    _recording_control_service_name = "recording_control"; 
    LoadParam(_private_nh, "recording_control_service_name", _recording_control_service_name); 

    _recording_control_service = _private_nh.advertiseService(_recording_control_service_name, 
                                                &RosThread::recording_control_callback, this);
    
    while (!_stop && ros::ok()) 
    {

        ros::spinOnce(); 
    }

    ros::shutdown(); 
    std::cout << "thread out" << std::endl; 
}

void RosThread::recording_control_callback(std_srvs::SetBool::Request  &request,
                                           std_srvs::SetBool::Response &response) 
{

}
