## 参考资料

教程链接:[【奥特学园】ROS机器人入门课程《ROS理论与实践》零基础教程](https://www.bilibili.com/video/BV1Ci4y1L7ZZ?p=182)

配套文档链接：[奥特学园ROS配套文档](http://www.autolabor.com.cn/book/ROSTutorials/)

## TF坐标转换

### 5.1.1 坐标msg消息

订阅发布模型中数据载体msg是一个重要实现，首先需要了解一下，在坐标转换实现中常用的<code>geometry_msgs/TransformStamped</code> 和<code>geometry_msgs/PointStamped</code>  

前者用于传输坐标系相关位置信息，后者用于传输某个坐标系内坐标点的信息。在坐标变换中，频繁的需要使用到坐标系的相对关系以及坐标点信息。    

#### 1.geometry_msgs/TransformStamped

命令行键入：<code>rosmsg info geometry_msgs/TransformStamped</code>  

> rosmsg命令可以显示msg格式

```javascript
std_msgs/Header header      #头信息
  uint32 seq                                       #|--序列号
  time stamp                                     #|--时间戳
  string frame_id                             #|--坐标ID
string child_frame_id           #子坐标系的id
geometry_msgs/Transform transform        #坐标信息
  geometry_msgs/Vector3 translation       #偏移量
    float64 x                                                                     #|-- x方向的偏移量
    float64 y                                                                     #|-- y方向的偏移量
    float64 z                                                                     #|-- z方向的偏移量
  geometry_msgs/Quaternion rotation     #四元数
    float64 x
    float64 y
    float64 z
    float64 w
```

### 5.1.2 静态坐标变换

所谓静态坐标变换，是指两个坐标系之间的相对位置是固定的。  

**需求描述**  

现在一机器人模型，核心构成包含主体与雷达，各对应一坐标系，坐标系的原点分别位于主体与雷达的物理中心，已知雷达原点相对于主体原点位移关系如下：x 0.2 ,y 0.1, z 0.5.当前雷达检测到一障碍物，在雷达坐标系中障碍物的坐标为(2.0, 3.0, 5.0),请问，该障碍物相对于主体的坐标是多少？  

![image-20210909101312947](/home/novauto/Learn/ROS_STUDY/AoTeXueYuan_ROS/README.assets/image-20210909101312947.png)

![image-20210909101520912](/home/novauto/Learn/ROS_STUDY/AoTeXueYuan_ROS/README.assets/image-20210909101520912.png)

**实现分析**

1.坐标系相对关系，可以通过发布方发布  

2.订阅方，订阅到发布的坐标系相对关系，再传入坐标点信息（可以写死），然后借助于tf实现坐标变换，并将结果输出。

**实现流程**：C++与Python实现流程一致  

1.新建功能包，添加依赖  

2.编写发布方实现  

3.编写订阅方实现  

4.执行并查看结果  

**方案A：C++实现**  

**1.创建功能包**  

创建项目功能包依赖于tf2、tf2_ros、tf2_geometry__msgs、roscpp、rospy、std_msgs、geometry_msgs  

**2.发布方**  

下面程序设置的是child相对于base的平移和旋转关系。  

```c++
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
```

![image-20210909143536929](/home/novauto/Learn/ROS_STUDY/AoTeXueYuan_ROS/README.assets/image-20210909143536929.png)

**3.订阅方** 

```c++
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
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "laser";
    ps.header.stamp = ros::Time::now();
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
           ps_out = buffer.transform(ps,"base_link");    
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
```

**补充1：**  

当坐标系之间的相对位置固定时，那么所需参数也是固定的：父系坐标名称、子级坐标系名称、x偏移量、y偏移量、z偏移量、x翻滚角度、y俯仰角度、z偏航角度，实现逻辑相同，参数不同，那么ROS系统就已经封装好了专门的节点，使用方式如下：  

`rosrun tf2_ros static_transform_publisher x偏移量 y偏移量 z偏移量 z偏航角度 y偏航角度 x翻转角度 父级坐标系 子级坐标系`  

示例：`rosrun tf2_ros static_transform_publisher 0.2 0 0.5 0 0 0 /baselink /laser`  

也建议使用该中方式直接实现静态坐标系相对信息发布。  

**补充2：**  

可以借助于rviz显示坐标系关系，具体操作：  

- 新建窗口输入命令：`rviz`  

- 在启动的rviz中设置Fixed Frame为`base_link`;  
- 点击左下的add按钮，在弹出的窗口中选择TF组件，即可显示坐标关系。  

### 5.1.3 动态坐标变换

所谓动态坐标变换，，是指两个坐标系之间的相对位置是变化的。

**需求描述：**

启动turtlesim_node，该节点中窗体有一个世界坐标系（左下角为坐标系原点），乌龟是另一个坐标系，键盘控制乌龟运动，将两个坐标系的相对位置动态发布。

![img](http://www.autolabor.com.cn/book/ROSTutorials/assets/坐标变换_动态.gif)
