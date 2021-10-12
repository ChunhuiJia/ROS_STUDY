## TF2中欧拉角和四元数的转换

参考资料：

[ros中欧拉角与四元数的转换](https://blog.csdn.net/weixin_42454034/article/details/107136341)(实测好使)

```cpp
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//欧拉角转四元数
  tf2::Quaternion orientation;
  float yaw=1;
  orientation.setRPY(0.0, 0.0, yaw);

//四元数转欧拉角
    tf2::Quaternion imu_quat(
      imu_data.orientation.x,
      imu_data.orientation.y,
      imu_data.orientation.z,
      imu_data.orientation.w);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf2::Matrix3x3 m(imu_quat);
    m.getRPY(roll, pitch, yaw);//进行转换，得出的结果的单位是：rad
    

```

