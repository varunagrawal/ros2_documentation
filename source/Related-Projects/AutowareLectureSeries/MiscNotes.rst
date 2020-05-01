Installation of ADE
====

```
mkdir adehome
cd adehome 

wget https://gitlab.com/ApexAI/ade-cli/uploads/85a5af81339fe55555ee412f9a3a734b/ade+x86_64
sudo chmod +x ade
export PATH=$PATH:$PWD
# test execution 
echo $PATH
which ade
# Update ade
ade update-cli
# Now setup ade
touch .adehome 
git clone --recurse-submodules https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
cd AutowareAuto/
ade start
# this will take awhile
ade enter
```

source /opt/ros/dashing/setup.bash 
sudo apt update
sudo apt install ros-dashing-turtlesim
sudo apt install ros-dashing-rqt-*
sudo apt-install byobu

# Now we'll start byobu and see what we can do!
byobu
# F2 NEW TERMINAL / F3/F4 Next/Prev terminal
source /opt/ros/dashing/setup.bash
ros2 run turtlesim turtlesim_node
# Turtle should pop up!

# Now press F2 
source /opt/ros/dashing/setup.bash
ros2 run turtlesim turtle_teleop_key

# Congratulations! You are now running ROS!



# Installation of Slides Toolchain
```
git clone git@github.com:adamzap/landslide.git
git clone https://github.com/adamzap/landslide.git
cd landslide
python setup.py build
sudo pip install markupsafe
sudo python setup.py install
# Now install prince pdf
cd ~/Downloads 
wget https://www.princexml.com/download/prince_13.5-1_ubuntu18.04_amd64.deb
sudo dpkg -i prince_13.5-1_ubuntu18.04_amd64.deb 
rm prince_13.5-1_ubuntu18.04_amd64.deb
# If you wish to view slides. 
sudo apt install okular 
```

To generate slides
===
landslide Lesson2Basics.rst -d Lesson2Slides.pdf
okular Lesson2Slides.pdf
