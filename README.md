# McMaster Mars Rover

## Quick Start
### 1. Install Docker 
* https://docs.docker.com/get-docker/
* If you're running Windows you'll want to set up WSL 2 before installing Docker. For more details see Windows Install.

### 2. Clone this repo to your desired location
    
```
git clone https://github.com/MacRover/Rover.git
```

### 3. Build the docker file within the cloned directory

```
docker build -t macrover/rover .
```

This may take a while the first time you run it.

### 4. Start the macrover/rover docker container

```
docker run -it macrover/rover
```

You may also want to set up X11 forwarding in order to work with RVIZ, gazebo, rqt, etc. I've set this up with WSL but haven't done it on native linux yet so I'll update this later

## Windows Install (Windows 10 v2004+)
### 1. Set up WSL 2 on Windows
* Here's the official how-to guide: https://docs.microsoft.com/en-us/windows/wsl/install-win10
* Recommended distro is Ubuntu 18.04 LTS or 20.04 LTS
* Set up a new user
* You may also want to install Windows Terminal for an improved terminal prompt UI https://www.microsoft.com/en-ca/p/windows-terminal/9n0dx20hk701
* **Make sure** you're setting up WSL 2 and not just WSL 1. WSL 2 requires a few extra steps to configure. Docker won't work properly if you use WSL 1.
### 2. Install Docker for Windows
* https://hub.docker.com/editions/community/docker-ce-desktop-windows
* Stable version recommended
* Make sure docker is set to "Use the WSL 2 based engine" 
![use WSL2 image](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/docker_use_wsl2.png)
* Make sure that integration with your Ubuntu distro is enabled 
![wsl2 docker integration image](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/enable_docker_in_wsl.png)
### 3. Install an X server for Windows
* Recommended to install VcXsrv https://sourceforge.net/projects/vcxsrv/ as it is actively developed and open-source
* This runs on your base Windows OS
### 4. Configure VcXsrv to work with your Ubuntu WSL install
* In your Ubuntu terminal prompt (Should be able to search for "Ubuntu" in the start menu to find this), type:

  ```
  nano ~/.bashrc
  ```

  Don't delete anything, but paste the following lines at the end of the file (use arrow keys to scroll down):

  ```
  # Connect to Windows X11 server
  export DISPLAY=`grep -oP "(?<=nameserver ).+" /etc/resolv.conf`:0.0
  export LIBGL_ALWAYS_INDIRECT=1
  ```

  Press `Ctl+X` and then `Y` to save and exit.

  Finally, run 
  ```
  source ~/.bashrc
  ```
* Open XLaunch on Windows (search for XLaunch in the start menu)

  Select `Multiple Windows` and set display number to `-1`. Click next.

  ![xlaunch 1st prompt](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/xlaunch_prompt_1.png)

  Select `Start no client`. Click next.

  **Uncheck** `Native OpenGL` and **check** `Disable access control`. Click next.

  ![xlaunch prompt 2](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/xlaunch_prompt_2.png)

  Save the configuration file somewhere you can remember. Next time you can just open this and skip the config options. Click finish.

  If you get a prompt from Windows firewall, make sure to allow permissions on **both** private and public networks.

### 5. Clone this Rover repo to your Ubuntu WSL distro
* In the terminal on Ubuntu, type:
  ```
  git clone https://github.com/MacRover/Rover.git
  ```
* You can clone it to wherever you want in your Linux install, just make sure you know where it is.
* After cloning, `cd` into the repo

  ```
  cd Rover
  ```
### 6. Build the Docker image
* Inside the git repo (where you should already be), type:

  ```
  docker build -t macrover/rover .
  ```
  If you run into issues where the docker command is not found, make sure docker is running and that you've set up integration with your Ubuntu distro. If all else fails, try rebooting and see if it works. Just make sure to start up the X11 server (using the saved config file from before), start up docker, and `cd` into the cloned `Rover` git repository again when the computer completes the reboot.
	
    Now you're ready to start the docker container. Type the following into your Ubuntu terminal window:

  ```
  docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" macrover/rover
  ```

  The extra flags here allow the X11 server running on Windows to interact with the docker container. To start the container in the background, replace the `-it` flag with `-d`.

  ***NOTE:*** Sometimes after a reboot of your computer the `DISPLAY` variable will change. If this happens, the X11 link will stop working and you won't be able to open any GUI elements. To fix this you will have to create a new docker container using the same `docker run` command above the new `DISPLAY` variable will be used automatically. 

### 7. VSCode Integration (optional but useful)
* [Download and install VSCode](https://code.visualstudio.com/download)
* Download and install the [Remote Development Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)
* Open the `Remote Explorer` pane. Select `Containers` in the dropdown and open the macrover/rover container. If it's not already running, VSCode will start the container. 

  ![open containers vscode](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/vscode_container_select.png)

	From here you can open `/root/Rover` or `/root/Rover/ROS_WS`.

* If you open the latter (`/root/Rover/ROS_WS`), you are in the catkin workspace. This means the [VSCode ROS extension](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) will automatically detect your version.

  ![ros on vscode](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/vscode_ros_extension.png)

## Linux Install

### Docker & Nvidia Hardware acceleration

This set of instructions applies only if running on linux with and nvidia gpu. This install assumes you have the most recent nvidia GPU drivers (v440+)

#### 1. Install docker

* https://docs.docker.com/engine/install/ubuntu/

* Install prerequisites 
  ```
  sudo apt update
  ```
  ```
  sudo apt install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
  ```
* Add gpg key and verify
  ```
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
  ```
  ```
  sudo apt-key fingerprint 0EBFCD88
  ```
* Add docker repository
  ```
  sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu$(lsb_release -cs)stable"
  ```
* Install docker engine
  ```
  sudo apt update
  ```
  ```
  sudo apt install docker-ce docker-ce-cli containerd.io
  ```
* Check that docker install works
  ```
  sudo docker run hello-world
  ```
  Note: you may experience a permissions error here. This means you must add your current user to the `docker` group. To do this, do the following:
  ```
  sudo usermod -aG docker $USER
  ```
  And then reload the groups to register the changes:
  ```
  newgrp docker
  ```
  Try running the hello-world image again and it should work.

#### 2. Install nvidia cuda support
* Follow the deb (network) instructions found here https://developer.nvidia.com/cuda-downloads

* For ***Ubuntu 20.04***, the installation commands are: 
  ```
  $ wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
  $ sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
  $ sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub
  $ sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
  $ sudo apt update
  $ sudo apt -y install cuda
  ```
* Please check the website for your specific install commands
#### 3. Install nvidia-docker2

* Add nvidia repos
  ```
  curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
  ```
  ```
  distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
  ```
  ```
  curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
  ```
  ```
  sudo apt update
  ```
* Install nvidia-docker2
  ```
  sudo apt install nvidia-docker2
  ```
* Restart docker daemon
  ```
  sudo systemctl restart docker
  ```
* Verify that nvidia-docker2 works
  ```
  docker run --gpus all nvidia/cuda:10.2 nvidia-smi
  ```
  If your installed CUDA release is not 10.2, check the verison using the following command:
  ```
  nvcc --version
  ```
  and replace `nvidia/cuda:10.2` with your installed release.

#### 4. Clone the Rover repo and build the docker container
* Clone the current repo
  ```
  git clone https://github.com/MacRover/Rover.git
  ```
* cd into the repo and build the dockerfile
  ```
  cd Rover
  ```
  ```
  docker build -t macrover/rover .
  ```
#### 5. Start the container
* There is a special script to start the docker container with X11 forwarding
  ```
  chmod +x start_nvidia_docker.bash
  ```
  ```
  ./start_nvidia_docker.bash
  ```
* Test that the X11 forwarding works. In a new terminal pane type:
  ```
  docker exec -it macrover /bin/bash
  ```
  ```
  rviz
  ```
  Rviz should open. If it fails shut down the container, run 
  ```
  sudo rm -rf /tmp/.docker.xauth
  ```
  and then run the start script again.
* ***NOTE*** THE INSTALL SCRIPT DELETES THE CONTAINER EACH TIME IT IS TURNED OFF!! 

  This means that **shutting down the container without commiting changes will delete all progress** you have done inside the container!

  To **disable** this feature, remove the `--rm` flag from line 25 of `start_nvidia_docker.bash`.

### Native Linux install
Check out the [ROS installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu) on the ROS wiki.
