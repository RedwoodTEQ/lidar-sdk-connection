# Lidar SDK Connection

### Tested on native Ubuntu 18.04.5 installation
### Setup steps
1. Install Docker (https://docs.docker.com/engine/install/ubuntu/).
2. (Optional) Follow steps to manage docker as non-root user and also enable docker service through systemd (https://docs.docker.com/engine/install/linux-postinstall/) - This is optional, however we will assume from this step onwards that docker can be managed by a non-root user.
3. Install docker-compose (https://docs.docker.com/compose/install/).
4. Ensure docker daemon is running. `$ docker ps`
5. Create a docker volume. This will allow us to persist the data collected from Lidar device. `$ docker volume create --name=sharedstorage`
6. Download/clone this repository. `$ git clone git@github.com:RedwoodTEQ/lidar-sdk-connection.git`
7. Go to data collection directory `$ cd lidar-sdk-connection/data_collection`
8. Start the data collection. This will automatically build the required images and start the required docker containers.`docker-compose up`
9. To stop data collection `docker-compose down`