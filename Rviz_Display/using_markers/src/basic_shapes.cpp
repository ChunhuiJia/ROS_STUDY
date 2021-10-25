#include <ros/ros.h>
#include <visualization_msgs/Marker.h>    //Marker可以显示基本形状
/*
Marker核心元素：
Marker.header.frame_id = "/my_frame";   //设置该Marker的坐标系
Marker.header.stamp  = ros::Time::now();  //设置时间戳
Marker.ns = "basic_shapes";  //使用相同命名空间和id发送的任何标记都将覆盖旧标记
Marker.id = 0;  //不知道有什么用
Marker.shape = visualization_msgs::Marker::CUBE/SPHERE/ARROW/CYLINDER;   //Marker的形状
Marker.action = visualization_msgs::Marker::ADD;   //设置marker的动作，选项是ADD,DELETE,在ROS indigo中还有DELETEALL

Marker.pose.position.x = 0;   //Marker的位置，相对于frame_id的坐标系
Marker.pose.position.y = 0;
Marker.pose.position.z = 0;
Marker.pose.orientation.x = 0;  //Marker的方向
Marker.pose.orientation.y = 0;
Marker.pose.orientation.z = 0;
Marker.pose.orientation.w = 1.0;
Marker.scale.x = 1.0;   设置Marker的比例——这里的 1x1x1 表示一边是 1m
Marker.scale.y = 0.5;
Marker.scale.z = 0.3;

Marker.color.r = 0.0f;  //设置Marker的颜色
Marker.color.g = 1.0f;
Marker.color.b = 0.0f;
Marker.color.a = 1.0;    //透明度，一定要设置成比0大的值

Marker.lifetime = ros::Duration(0.5);  //对象在被自动删除之前应该持续多长时间。 0 表示永远
*/
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::ARROW;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    //相对于
    marker.pose.position.x = 1;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1;    //对于ARROW，是箭头的长度
    marker.scale.y = 0.2;   //对于ARROW，是箭头的宽度
    marker.scale.z = 0.2;   //对于ARROW，是箭头的高度

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.2;    //透明度

    // marker.lifetime = ros::Duration();
marker.lifetime = ros::Duration(0.5);

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    // Cycle between different shapes
    // switch (shape)
    // {
    // case visualization_msgs::Marker::CUBE:
    //   shape = visualization_msgs::Marker::SPHERE;
    //   break;
    // case visualization_msgs::Marker::SPHERE:
    //   shape = visualization_msgs::Marker::ARROW;
    //   break;
    // case visualization_msgs::Marker::ARROW:
    //   shape = visualization_msgs::Marker::CYLINDER;
    //   break;
    // case visualization_msgs::Marker::CYLINDER:
    //   shape = visualization_msgs::Marker::CUBE;
    //   break;
    // }

    r.sleep();
  }
}