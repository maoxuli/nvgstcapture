#include "ros_node.h"
#include <camera/RecordingStatus.h>
#include "nvgstcapture.h"

extern "C" 
{
    AppCtx *app;
    bool set_capture_params(); 
    bool start_recording(); 
    bool stop_recording(); 
} 

RosNode::RosNode(int argc, char** argv)
{
    // node name may be overriden by arguments 
    ros::init(_argc, _argv, "camera"); 
    ros::start(); 

    // we manually start and shutdown ros, so 
    // handle is not necessary to keep 
    ros::NodeHandle nh, private_nh("~"); 

    // dynamic reconfigure service 
    dynamic_reconfigure::Server<camera::CaptureParamsConfig>::CallbackType cb;
    cb = boost::bind(&RosNode::capture_params_callback, this, _1, _2);
    _capture_params_srv.setCallback(cb);

    // reocording control service, under node name  
    std::string recording_control_service = "recording_control"; 
    ROS_INFO_STREAM("Advertise recording control service: " << recording_control_service); 
    _recording_control_srv= private_nh.advertiseService(_recording_control_service, 
                                        &RosNode::recording_control_callback, this);

    // publish recording status topic
    std::string recording_status_topic = "recording_status";  
    ROS_INFO_STREAM("Advertise recording status topic: " << recording_status_topic);
    _recording_status_pub = private_nh.advertise<camera_msgs::RecordingStatus>(recording_status_topic, 2); 
    
    // working thread 
    ROS_INFO("ROS node thread starting");
    _thread = std::thread(std::bind(&RosThread::work_thread, this));
}

RosNode::~RosNode()
{
    ROS_INFO("ROS node thread stopping"); 
    ros::shutdown(); 
    _thread.join();
}

void RosNode::work_thread() 
{
    ROS_INFO("ROS node thread in"); 
    ros::spin();
    ROS_INFO("ROS node thread out"); 
}

// dynamic reconfigure callback 
void RosNode::capture_params_callback(camera::CaptureParamsConfig& config, uint32_t level)
{
    // set_capture_params() 
}

bool RosNode::recording_control_callback(std_srvs::SetBool::Request  &request,
                                         std_srvs::SetBool::Response &response) 
{
    // start_recording()
    // stop_recording()
}

void RosNode::update_recording_status()
{

}
