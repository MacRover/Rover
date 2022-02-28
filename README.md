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

## Windows Install (Windows 11)

### 1. Set up WSL 2 on Windows

- Here's the official how-to guide: https://docs.microsoft.com/en-us/windows/wsl/install-win10
- Recommended distro is Ubuntu 18.04 LTS
- Detailed installation instructions and video walkthrough [can be found here](https://github.com/MacRover/GDSC-ROS-Workshop#pre-workshop-installation) 

### 2. Clone this Rover repo to your Ubuntu WSL distro

- In the terminal on Ubuntu, type:
  ```
  git clone https://github.com/MacRover/Rover.git
  ```
- You can clone it to wherever you want in your Linux install, just make sure you know where it is.
- After cloning, `cd` into the repo

  ```
  cd Rover
  ```

### 3. Install ROS

- Check out the [ROS installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu) on the ROS wiki.
- If you followed the detailed instructions link in step 1, you have already completed this step.

### 4. Build the ROS environment

- Inside the git repo (where you should already be), type:

  ```
  cd ROS_WS
  catkin_make
  ```

### 5. VSCode Integration (optional but useful)

- [Download and install VSCode](https://code.visualstudio.com/download)
- Download and install the [Remote Development Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)
- Open the `Remote Explorer` pane. Select `WSL Targets` in the dropdown and open this repo.

- If you open the latter (`/Rover/ROS_WS`), you are in the catkin workspace. This means the [VSCode ROS extension](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) will automatically detect your version.

  ![ros on vscode](https://raw.githubusercontent.com/MacRover/Rover/master/assets/images/vscode_ros_extension.png)

### Native Linux install

Check out the [ROS installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu) on the ROS wiki. It is highly recommended you install ROS on Ubuntu 18.04.
