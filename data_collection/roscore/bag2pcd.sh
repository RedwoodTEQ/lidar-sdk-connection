#!/bin/sh
echo "Compressing and archiving data before exiting."
echo "Please wait..."
cd /sharedstorage
echo "1234" >> /sharedstorage/archive.txt
rm -rf pcd_out/
for f in *.bag ; do
        . /opt/ros/melodic/setup.sh && rosrun pcl_ros bag_to_pcd ${f} /rslidar_points pcd_out
done
i=0
while -f "lidar${i}.pcd.gz"; do
        i += 1
done
tar -zcvf lidar${i}.pcd.gz pcd_out/