#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
/*
    订阅方：订阅发布的坐标系相对关系，传入一个坐标点，调用tf实现转化

    流程：
        1.包含头文件
        2.编码、初始化、NodeHandle（必须的）
        3.创建订阅对象；  ---->  订阅坐标系相对关系
        4.组织一个坐标点数据；
        5.转换算法，需要调用TF内置实现；
        6.最后输出
*/
int main(int argc, char **argv)
{
    // 2.编码、初始化、NodeHandle（必须的）
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "static_sub");
    ros::NodeHandle nh;
    // 3.创建订阅对象；  ---->  订阅坐标系相对关系
    //3-1.创建一个buffer缓存
    tf2_ros::Buffer buffer;
    //3-2.再创建监听对象（监听对象可以将订阅的数据存入buffer）
    tf2_ros::TransformListener listener(buffer);   //订阅者订阅到消息的时候，会把消息放到buffer里。结合使用
    // 4.组织一个坐标点数据；
    //参考的坐标系
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "turtle1";
    //时间戳
    //ps.header.stamp = ros::Time(0.0);   //设置0.0相当于告诉坐标转换不要管我的时间戳了
    // ps.header.stamp = ros::Time::now();  
    //但是按照up说的会拿点的时间戳和坐标变换的时间戳进行比对是否比较相近，不相近就会抛出异常，
    //但是按理按理来说也10hz的时间戳也没差多少啊，那怎么样才正确呢？还有啥用呢
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;
    //添加休眠,要不然可能还没有收到buffer，就运行到ps_out = buffer.transform(ps,"base_link");，会导致运行时报错
    // ros::Duration(2).sleep();
    // 5.转换算法，需要调用TF内置实现；
    ros::Rate rate(10);
    while(ros::ok())
    {
        //核心代码  --- 将ps转换成相对于base_link的坐标点
        geometry_msgs::PointStamped ps_out;
        /*
        调用了buffer的转换函数transform
        参数1：被转换的坐标点（包含了原坐标系的信息）
        参数2：目标坐标系
        返回值：输出的目标坐标系下的坐标点

        ps1：调用时必须包含头文件 tf2_geometry_msgs/tf2_geometry_msgs.h，否则编译器报错
        ps2：运行时存在的问题：抛出一个异常base_link不存在
                        原因：订阅数据是一个耗时操作，可能在调用transform转换函数时，坐标系的相对关系还没有订阅到，因此出现异常
                        解决：
                            方案1：在调用转换函数前，执行休眠ros::Duration(2).sleep();
                            方案2：进行异常处理(建议)
        */
       try
       {
           ps_out = buffer.transform(ps,"world");    
            // 6.最后输出
            ROS_INFO("转换后的坐标值：（%.2f,%.2f,%.2f）,参考的坐标系：%s",
            ps_out.point.x,
            ps_out.point.y,
            ps_out.point.z,
            ps_out.header.frame_id.c_str());
       }
       catch(const std::exception& e)
       {
            //    std::cerr << e.what() << '\n';
            ROS_INFO("异常消息：%s",e.what());
       }
       
        
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}