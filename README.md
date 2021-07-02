# 4.28的rosbag
# 间断卡尔曼滤波融合imu数据和识别的aruco码数据
# 包的名字叫做kalman_location_fusion，在kalman_ws下面
# https://www.cnblogs.com/Yanfang20180701/p/12559521.html  
 将非图片数据提取为.csv格式存储： rostopic echo -b xx.bag -p /mynteye/left/image_mono > cam0data.csv
