#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>

class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh)
  {
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
  }

  void start(const std::string& base_frame)
  { 
    myworkcell_core::LocalizePart srv;

    srv.request.base_frame = base_frame;
    ROS_INFO("Attempting to localize part");
    ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

    
    if (!vision_client_.call(srv))
    {
      ROS_ERROR("Could not localize part");
      return;
    }
    ROS_INFO_STREAM("part localized: " << srv.response);
  }

private:
  // Planning components
  ros::ServiceClient vision_client_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "myworkcell_node");
  ros::NodeHandle nh;
  ROS_INFO("ScanNPlan node has been initialized");

  ros::NodeHandle private_node_handle ("~");
  std::string base_frame;
  // parameter name, string object reference, default value
  // must be called before app.start() because it is used inside it
  private_node_handle.param<std::string>("base_frame", base_frame, "world");
  // the node must be launched with launch file and "base_frame" param value is set

  ScanNPlan app(nh);
  ros::Duration(.5).sleep();  // wait for the class to initialize
  app.start(base_frame);
  ros::spin();
}