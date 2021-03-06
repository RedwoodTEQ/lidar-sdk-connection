import glob
import rosbag
point_cloud_bag_files_full_path = sorted(glob.glob("/storage/rslidar_points_*.bag"))
with open('/output/py.log', 'w') as logfile:
    for filepath in point_cloud_bag_files_full_path:
        with open('/output/' + filepath.split('/')[2] + ".decoded", 'w') as out_fp:
            bag = rosbag.Bag(filepath)
            for topic, msg, t in bag.read_messages(topics=['/rslidar_points']):
                # print(msg)
                out_fp.write(str(msg))
            bag.close()
