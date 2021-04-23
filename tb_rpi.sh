cd turtlebot3_ws/src/
rm -r scripts
svn export https://github.com/hjunleon/r2auto_nav.git/trunk/Scripts/RPi/scripts
cd ..
colcon build --symlink-install
source ~/.bashrc
source install/setup.bash
