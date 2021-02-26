import glob
import rosbag

point_cloud_bag_files = glob.glob("/storage/rslidar_points_*.bag")


for filename in point_cloud_bag_files:
    with open('/output/' + filename + ".decoded", 'w') as out_fp:
        bag = rosbag.Bag(filename)
        for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
            out_fp.write(msg)
        bag.close()
