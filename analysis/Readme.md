**velocity_angular.png 全零 = msckf_vio 未发布角速度，该图可以忽略**

查看 msckf_vio 源码 msckf_vio.cpp:1395，发布 odom 时只填充了线速度 
```c++
tf::vectorEigenToMsg(body_velocity, odom_msg.twist.twist.linear);```
没有任何代码给 twist.twist.angular 赋值，所以它保持 ROS 消息的默认值 0.0。
原因：MSCKF-VIO 的状态向量只估计位置、姿态和线速度，不包含角速度。虽然 IMU 陀螺仪能提供角速度原始测量，但算法没有将其作为估计量发布到 odom 话题中。这是 msckf_vio 代码的设计选择，不影响定位精度。
---