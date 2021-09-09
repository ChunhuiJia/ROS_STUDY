#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
/*
    需求：发布两个坐标系的相对关系

    流程：
        1.包含头文件；
        2.设置编码 节点初始化 NodeHandle;
        3.创建发布对象；
        4.组织被发布的消息；
        5.发布数据；
        6.spin();
*/
int main(int argc, char **argv)
{
    //     2.设置编码 节点初始化 NodeHandle;
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"static_pub");
    ros::NodeHandle nh;
    /*setlocale(LC_ALL,"");       ：   C++中的locale设置：
    C/C++程序中，locale（即系统区域设置，即国家或地区设置）将决定程序所使用的当前语言编码、日期格式、数字格式及其它与区域有关的设置，
    locale设置的正确与否将影响到程序中字符串处理（wchar_t如何输出、strftime()的格式等）。
    因此，对于每一个程序，都应该慎重处理locale设置。
    参考链接1：https://blog.csdn.net/haiross/article/details/45074355
    参考链接2：https://blog.csdn.net/sxhlovehmm/article/details/40919429
    */
    //     3.创建发布对象；
    tf2_ros::StaticTransformBroadcaster pub;
    //     4.组织被发布的消息；
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = "base_link";
    tfs.child_frame_id="laser";
    tfs.transform.translation.x=0.2;
    tfs.transform.translation.y=0.0;
    tfs.transform.translation.z=0.5;

    // 需要根据欧拉角转换
    tf2::Quaternion qtn;  //创建四元数对象
    //向该对象设置欧拉角，这个对象可以将欧拉角转换成四元数
    qtn.setRPY(0,0,0);   //由于小车中的雷达和本次没有角度偏差，只有位移偏差，所以将欧拉角都设置为0，单位：rad
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();

    //     5.发布数据；
    pub.sendTransform(tfs);
    //     6.spin();
    ros::spin();  //ROS的回旋函数，会进入循环
    return 0;
}