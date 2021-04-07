cd colcon_ws/src/
rm -r scripts
svn export https://github.com/hjunleon/r2auto_nav.git/trunk/Scripts/Workstation/scripts
cd colcon_ws/
colcon build --packages-select scripts
source install/setup.bash
