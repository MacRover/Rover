# McMaster Mars Rover

## Quick Start
1. Install Docker https://docs.docker.com/get-docker/
    * If you're running Windows you'll want to set up WSL 2 before installing Docker. For more details see Windows Install.

2. Clone this repo to your desired location
    
    `git clone https://github.com/MacRover/Rover.git`

3. Build the docker file within the cloned directory. This may take a while the first time you run it.

    ```
    docker build -t macrover/rover .
    ```

4. Start the macrover/rover docker container

    ```
    docker run -it macrover/rover
    ```

    You may also want to set up X11 forwarding in order to work with RVIZ, gazebo, rqt, etc. I've set this up with WSL but haven't done it on native linux yet so I'll update this later

## Windows Install (Windows 10 v2004+)
1. Set up WSL 2 on Windows
    * Here's the official how-to guide: https://docs.microsoft.com/en-us/windows/wsl/install-win10
    * Recommended distro is Ubuntu 18.04 LTS or 20.04 LTS
    * Set up a new user
    * You may also want to install Windows Terminal for an improved terminal prompt UI https://www.microsoft.com/en-ca/p/windows-terminal/9n0dx20hk701
    * **Make sure** you're setting up WSL 2 and not just WSL 1. WSL 2 requires a few extra steps to configure. Docker won't work properly if you use WSL 1.
2. Install Docker for Windows
    * https://hub.docker.com/editions/community/docker-ce-desktop-windows
    * Stable version recommended
    * Make sure docker is set to "Use the WSL 2 based engine" 
        ![use WSL2 image](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/docker_use_wsl2.png)
    * Make sure that integration with your Ubuntu distro is enabled 
        ![wsl2 docker integration image](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/enable_docker_in_wsl.png)
3. Install an X server for Windows
    * Recommended to install VcXsrv https://sourceforge.net/projects/vcxsrv/ as it is actively developed and open-source
    * This runs on your base Windows OS
4. Configure VcXsrv to work with your Ubuntu WSL install
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

5. Clone this Rover repo to your Ubuntu WSL distro
    * In the terminal on Ubuntu, type:
        ```
        git clone https://github.com/MacRover/Rover.git
        ```
    * You can clone it to wherever you want in your Linux install, just make sure you know where it is.
    * After cloning, `cd` into the repo
    
        ```
        cd Rover
        ```
6. Build the Docker image
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

7. VSCode Integration (optional but useful)
    * [Download and install VSCode](https://code.visualstudio.com/download)
    * Download and install the [Remote Development Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)
    * Open the `Remote Explorer` pane. Select `Containers` in the dropdown and open the macrover/rover container. If it's not already running, VSCode will start the container. 
    
        ![open containers vscode](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/vscode_container_select.png)
    
        From here you can open `/root/Rover` or `/root/Rover/ROS_WS`.
    
    * If you open the latter (`/root/Rover/ROS_WS`), you are in the catkin workspace. This means the [VSCode ROS extension](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) will automatically detect your version.

        ![ros on vscode](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/vscode_ros_extension.png)

## Linux Install

1. This section will be completed in more detail when I actually install ROS on Linux (GNU/Linux for all the Stallmans out there). For now check out the [ROS installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu) on the ROS wiki.
        