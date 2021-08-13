# На штуке, где будет подниматься драйвер realsense
`roslaunch realsense_slam realsense_bringup.launch`

# На своем компе

1. Посмотреть, в какой топик пишет realsense драйвер.
2. Открыть файл `realsense_slam/param/costmap_common_params_REAL.yaml` (там в конце названия файла real)
3. Вместо `bruh_topic00000000000000000` написать топик с pointcloud от realsense и сменить тип на соответствующий.
4. `roslaunch realsense_slam unitree_real.launch`
5. `roslaunch realsense_slam rtabmpap.launch imu_topic:=IMU_data`
6. `roslaunch realsense_slam move_base real:=true`
7. `rosrun rviz rviz -d ~/lidar_test/src/realsense_slam/rviz/realsense.rviz`