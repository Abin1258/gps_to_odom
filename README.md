# gps_to_odom
gps/RTK odometry

rosrun gps_to_odom gps_to_odom


sudo apt-get install ros-noetic-geodesy

sudo apt-get install ros-noetic-geographic-msgs

记得命名安装了eigen库啊, 为啥提示找不到库文件啊？
初步怀疑环境的配置问题．

查到到eigen的安装路径为：
/usr/include/eigen3/Eigen

两种解决办法：
1. 将/usr/include/eigen3/Eigen递归复制到/usr/include/Eigen
cp -rf /usr/include/eigen3/Eigen /usr/include/Eigen -R
————————————————
版权声明：本文为CSDN博主「AndyCheng_hgcc」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/chengde6896383/article/details/88339643
