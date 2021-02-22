# Lidar SDK Connection

### Tested on native Ubuntu 18.04.5 installation
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
12. Navigate to a directory where the Linux user has permissions in order to set up the sdk (e.g. `$ cd /home/<user>`).
12. Download/clone this repository: `$ git clone git@github.com:RedwoodTEQ/lidar-sdk-connection.git`
13. Go to data collection directory: `$ cd lidar-sdk-connection/data_collection`
14. Start the data collection. This will automatically build the required docker images and start the required docker containers: `$ docker-compose up`
15. To stop data collection: `$ docker-compose down`

### Data Collection (after initial setup)
1. Ensure docker daemon is running and the 'sharedstorage' docker volume is present.
2. Connect and power on Lidar device.
3. Check that network is configured correctly and Lidar packets are being received by the ethernet interface.
4. Go to the data collection directory: `$ cd <base-directory>/lidar-sdk-connection/data_collection`.
5. Run `$ docker-compose up` to start data collection.
6. When finished collecting data, run `$ docker-compose down` to stop data collection.

### Data Decoding/Extraction after collection
1. TODO