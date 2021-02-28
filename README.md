# Lidar SDK Connection

### Tested on native Ubuntu 18.04.5 installation with RS-LiDAR-16
### First Setup Instructions
1. Install Docker (https://docs.docker.com/engine/install/ubuntu/).
2. (Optional) Follow steps to manage docker as non-root user and also enable docker service through systemd (https://docs.docker.com/engine/install/linux-postinstall/) - This is optional, however we will assume from this step onwards that docker can be managed by a non-root user.
3. Install docker-compose (https://docs.docker.com/compose/install/).
4. Ensure docker daemon is running: `$ docker ps`
5. Create a docker volume. This will allow us to persist the data collected from Lidar device: `$ docker volume create --name=sharedstorage`
6. Connect Lidar device to an available ethernet port on the computer.
8. Power on Lidar device.
7. Download Wireshark (https://www.wireshark.org/).
8. Run Wireshark and start listening on the ethernet interface that the Lidar device is connected to.
9. Identify the IP address that the Lidar device is sending packets to. (If there is a constant stream of UDP packets then the network interface is set up correctly. If there are ARP packets, then the network interface needs to be changed to the IP that the ARP packet is looking for.)
10. Modify ethernet interface settings as required. (e.g. `$ sudo ifconfig eth0 192.168.0.2`, if Lidar is sending packets to 192.168.0.2)
11. Verify that packets are being received from Lidar device using Wireshark.
12. Navigate to a directory where the Linux user has permissions in order to set up the sdk (e.g. user home directory `$ cd ~`).
12. Download/clone this repository: `$ git clone git@github.com:RedwoodTEQ/lidar-sdk-connection.git`
13. Go to data collection directory: `$ cd lidar-sdk-connection/data_collection`
14. Start the data collection. This will automatically build the required docker images and start the required docker containers: `$ docker-compose up`
15. To stop data collection: `$ docker-compose down`

### Data Collection (after initial setup)
1. Ensure docker daemon is running and the 'sharedstorage' docker volume is present.
2. Connect and power on Lidar device.
3. Check that network is configured correctly and Lidar packets are being received by the ethernet interface.
4. Go to the directory where the repository was cloned: `$ cd <parent-directory>/lidar-sdk-connection`.
5. Go to the data collection directory `$ cd data_collection`.
6. Run `$ docker-compose up -d` to build container and start data collection.
7. When finished collecting data, run `$ docker-compose down` to stop data collection.

### Data Decoding/Extraction after collection
#### Notes:
* Decoding bag files into plain text format is extremely space inefficient:
A bag file containing point cloud data typically consumes 300-400MB of disk space for every 30 seconds of data collection whilst the plain text format of the same data will typically consume 4x as much disk space (i.e. 1.2GB - 1.6GB). As such, it is would probably be best to process the data directly from the bag file (e.g. through Python or C++ API) rather than decode it into plain text before processing.
* I am investigating the use of the /rslidar_packets topic which appears to be a lot more space efficient than the /rslidar_points topic - the data processing may be updated in the future to use the packets topic instead.
* .bag files are read from the /storage volume and the output will be sent to the /output volume. The files stored on these volumes will persist even outside of the lifetime of the container and can be read by other containers.
#### Prerequisites:
* Docker daemon is running and the 'sharedstorage' docker volume is present and has been populated with one or more .bag files with /rslidar_points data.
* If there are no .bag files to process, this process will not do anything.
#### To decode data into plain text:
1. Ensure docker daemon is running and the 'output' docker volume is present. This volume can be created if it does not already exist by running: `$ docker volume create --name=output`.
2. Go to the data decode directory `$ cd data_decode`
3. Run `$ docker-compose up` to build container and start decoding the data.
4. When the process exits with message: `docker_rosbag_decode_roscore_master exited with code 0`, then the decoding process is complete.
5. The decoded data will now be accessible in the /output volume for further processing.

