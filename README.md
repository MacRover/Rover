# McMaster Mars Rover

## Quick Start (ROS already installed)

### 1. Clone this repo to your desired location

```
git clone https://github.com/MacRover/Rover.git
```

### 2. Build the ROS environment

```
cd ROS_WS
catkin_make
```

## Windows Install (Windows 10 v2004+)

### 1. Set up WSL 2 on Windows

- Here's the official how-to guide: https://docs.microsoft.com/en-us/windows/wsl/install-win10
- Recommended distro is Ubuntu 18.04 LTS or 20.04 LTS
- Set up a new user
- You may also want to install Windows Terminal for an improved terminal prompt UI https://www.microsoft.com/en-ca/p/windows-terminal/9n0dx20hk701
- **Make sure** you're setting up WSL 2 and not just WSL 1. WSL 2 requires a few extra steps to configure. Docker won't work properly if you use WSL 1.

### 2. Install an X server for Windows

- Recommended to install VcXsrv https://sourceforge.net/projects/vcxsrv/ as it is actively developed and open-source
- This runs on your base Windows OS

### 3. Configure VcXsrv to work with your Ubuntu WSL install

- In your Ubuntu terminal prompt (Should be able to search for "Ubuntu" in the start menu to find this), type:

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

- Open XLaunch on Windows (search for XLaunch in the start menu)

  Select `Multiple Windows` and set display number to `-1`. Click next.

  ![xlaunch 1st prompt](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/xlaunch_prompt_1.png)

  Select `Start no client`. Click next.

  **Uncheck** `Native OpenGL` and **check** `Disable access control`. Click next.

  ![xlaunch prompt 2](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/xlaunch_prompt_2.png)

  Save the configuration file somewhere you can remember. Next time you can just open this and skip the config options. Click finish.

  If you get a prompt from Windows firewall, make sure to allow permissions on **both** private and public networks.

### 4. Clone this Rover repo to your Ubuntu WSL distro

- In the terminal on Ubuntu, type:
  ```
  git clone https://github.com/MacRover/Rover.git
  ```
- You can clone it to wherever you want in your Linux install, just make sure you know where it is.
- After cloning, `cd` into the repo

  ```
  cd Rover
  ```

### 5. Install ROS

- Check out the [ROS installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu) on the ROS wiki.

### 6. Build the ROS environment

- Inside the git repo (where you should already be), type:

  ```
  cd ROS_WS
  catkin_make
  ```

### 7. VSCode Integration (optional but useful)

- [Download and install VSCode](https://code.visualstudio.com/download)
- Download and install the [Remote Development Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)
- Open the `Remote Explorer` pane. Select `WSL Targets` in the dropdown and open this repo.

- If you open the latter (`/Rover/ROS_WS`), you are in the catkin workspace. This means the [VSCode ROS extension](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) will automatically detect your version.

  ![ros on vscode](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/vscode_ros_extension.png)

### Native Linux install

Check out the [ROS installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu) on the ROS wiki.
