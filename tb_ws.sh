cd colcon_ws/src/
rm -r scripts
svn export https://github.com/hjunleon/r2auto_nav.git/trunk/Scripts/Workstation/scripts
cd colcon_ws/
colcon build --symlink-install
source ~/.bashrc
source install/setup.bash
