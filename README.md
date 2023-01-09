# rosbag_gui_player
A GUI rosbag player tool for python3.  
You can play rosbag as if you are operating a media player by this tool.

# How to use
## 1. Clone this package
Clone This package to YourWorkspace.
> git clone https://github.com/tosiyuki/rosbag_gui_player.git

## 2. Run the script
### Terminal1
> cd ~/YourWorkspace  
> source devel/setup.bash  
> roscore

### Terminal2
> cd ~/YourWorkspace  
> source devel/setup.bash  
> python3 src/MainWindow.py

## 3. Select a bag file which you want to play
![](images/pic1.PNG "")

load rosbag file.

![](images/pic2.PNG "")

## 4. Discribe GUI

![](images/pic3.PNG "")

# Publish topic config function.

## Save config

Click Config -> Save Config

![](images/pic4.PNG "")

Select saving config file name and push save.

![](images/pic5.PNG "")

## Load Config

Click Config -> Load Config

![](images/pic6.PNG "")

Select loading config file name and push open.

![](images/pic7.PNG "")

Selected topicks are checked.

![](images/pic8.PNG "")

# License
MIT


