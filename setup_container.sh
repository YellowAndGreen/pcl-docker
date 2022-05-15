#XSOCK=./tmp/.X11-unix
#XAUTH=./tmp/.docker.xauth
#touch $XAUTH
CONTAINER_USER=pcl

docker run -it --gpus all --name="pcl-docker" --cap-add sys_ptrace -p 127.0.0.1:2222:22 --user=pcl --volume=./docker_dir:/home/pcl/docker_dir --volume=./example_project:/home/pcl/docker_dir/example_project dlopezmadrid/pcl-docker:latest



docker run -it --gpus all --name="pcl-docker" --cap-add sys_ptrace -p 127.0.0.1:2222:22 --user=pcl --volume=./example_project:/home/pcl/docker_dir/example_project dlopezmadrid/pcl-docker:latest