^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package usb_cam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.8 (2014-08-21)
------------------
* autoexposure and exposure settings now exposed via ROS parameters
* added ability to call v4l-utils as well as correctly set autofocus
* cleanup of output
* Merge pull request #15 from mistoll/develop
  added support for RGB24 pixel format
* Added RGB24 as pixel format
* Contributors: Michael Stoll, Russell Toris

0.1.7 (2014-08-20)
------------------
* changelog fixed
* minor cleanup and ability to change camera name and info
* Contributors: Russell Toris

0.1.6 (2014-08-15)
------------------
* Merge pull request #14 from KaijenHsiao/master
  added support for 10-bit mono cameras advertising as YUV
* added support for 10-bit mono cameras advertising as YUV (such as Leopard Imaging's LI-USB30-V034)
* Update CHANGELOG.rst
* changelog updated
* Merge pull request #13 from vrabaud/develop
  add a a ros::spinOnce to get set_camera_info working
* add a a ros::spinOnce to get set_camera_info working
  This is explained in the docs of CameraInfoManager
  https://github.com/ros-perception/image_common/blob/hydro-devel/camera_info_manager/include/camera_info_manager/camera_info_manager.h#L71
  Also, this fixes https://github.com/ros-perception/image_pipeline/issues/78
* Contributors: Kaijen Hsiao, Russell Toris, Vincent Rabaud, sosentos

0.1.5 (2014-07-28)
------------------
* auto format
* cleanup of readme and such
* Merge branch 'hydro-devel' of github.com:bosch-ros-pkg/usb_cam
* Merge pull request #11 from pronobis/hydro-devel
  Fixed a bug with av_free missing by adding a proper include.
* Fixed a bug with av_free missing by adding a proper include on Ubuntu 14.04.
* Merge pull request #7 from cottsay/groovy-devel
  Use pkg-config to find avcodec and swscale
* Merge pull request #5 from FriedCircuits/hydro-devel
  Remove requirments for self_test
* Use pkg-config to find avcodec and swscale
* Update package.xml
* Remove selftest
* Remove selftest
* Update usb_cam_node.cpp
* Merge pull request #2 from jonbinney/7_17
  swap out deprecated libavcodec functions
* swap out deprecated libavcodec functions
* Contributors: Andrzej Pronobis, Jon Binney, Russell Toris, Scott K Logan, William

0.1.3 (2013-07-11)
------------------
* Merge pull request #1 from jonbinney/rosify
  Bag of improvements
* add framerate parameter
* use ROS_* for output
* use camera_info_manager
* Contributors: Jon Binney, Russell Toris

0.1.2 (2013-05-06)
------------------
* installs usb_cam_node
* Contributors: Russell Toris

0.1.1 (2013-05-02)
------------------
* cmake fixed
* ffmpeg added
* Contributors: Russell Toris

0.1.0 (2013-05-01)
------------------
* Update package.xml
* minor cleanup
* inital merge
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update CLONE_SETUP.sh
* Update README.md
* Updated the README.md.
* Updated the installation instructions.
* Fixed syntax in the README.
* Updated README for ARDUINO support.
* Fixed update script.
* Updated the readme and updating scripts.
* Updating for installation on Robot.
* Updated installs and README for ROS.
* Make sure the User knows to source the devel/setup.sh.
* Getting rid of subtrees and Catkinized USB CAM.
* Updating home to use ROSWS.
* Fixing the launch file for video1.
* Merge commit '0bc3322966e4c0ed259320827dd1f5cc8460efce'
  Conflicts:
  src/sofie_ros/package.xml
* Removed unnecessary file.
* Compiles.
* Adding the Catkin build scripts.
* Merge commit 'b2c739cb476e1e01425947e46dc2431464f241b3' as 'src/ar_track_alvar'
* Squashed 'src/ar_track_alvar/' content from commit 9ecca95
  git-subtree-dir: src/ar_track_alvar
  git-subtree-split: 9ecca9558edc7d3a9e692eacc93e082bf1e9a3e6
* Merge commit '9feb470d0ebdaa51e426be4d58f419b45928a671' as 'src/sofie_ros'
* Squashed 'src/sofie_ros/' content from commit 3ca5edf
  git-subtree-dir: src/sofie_ros
  git-subtree-split: 3ca5edfba496840b41bfe01dfdff883cacff1a97
* Removing stackts.
* Removing submodules.
* Fixed submodules.
* Removing old package.
* Merge branch 'catkin'
  Conflicts:
  README.md
  cmake_install.cmake
* Brancing package down to stack base.
* Catkininizing.
* (catkin)Catkininizing.
* Modifying the setup of roshome.
* Starting to Catkininize the project.
* (catkin)Starting to Catkininize the project.
* Going to catinize it.
* (catkin)Going to catinize it.
* Modified to new version of sofie_ros.
* Renamed import_csv_data.py to fileUtils.py, because it does more now.
* (catkin)Renamed import_csv_data.py to fileUtils.py, because it does more now.
* Updating to use a csv file specified by the user. Separating PyTables path manipulation into SOFIEHDFFORMAT.
* (catkin)Updating to use a csv file specified by the user. Separating PyTables path manipulation into SOFIEHDFFORMAT.
* Merge branch 'release/0.0.2'
* Created the install script.
* Removed the Python Packages as submodules.
* Merge branch 'release/0.0.1'
* Update the Git submodules.
* Modified the README and CLONE_SETUP.sh
* Added SOFIEHDFFORMAT as a submodule.
* Added the ExperimentControl Repo as a submodule.
* Working the CLONE install.
* Modifiying install script.
* Added a script to update the gitmodules for read-only clones.
* Merge branch 'master' of github.com:agcooke/roshome
* Initial commit
* Added the modules.
* Added usb_cam,
* Updating to Groovy.
* (catkin)Updating to Groovy.
* Added another potential launch file for exporting video from rosbag.
* (catkin)Added another potential launch file for exporting video from rosbag.
* Added a launcher to ros bag the usb_cam, for later playback.
* (catkin)Added a launcher to ros bag the usb_cam, for later playback.
* Added some files that were possibly not correct
* (catkin)Added some files that were possibly not correct
* Fixed bugs with the importing.
* (catkin)Fixed bugs with the importing.
* Added forgotten __init__.py file and changed to importdata sofiehdfformat funciton.
* (catkin)Added forgotten __init__.py file and changed to importdata sofiehdfformat funciton.
* Refractoring to make it possible to log to CSV.
  There were problems handling concurrent writing to
  pytables files. The package now logs to CSV and then
  provides a function to post import the data into
  SOFIEHDFFORMAT.
* (catkin)Refractoring to make it possible to log to CSV.
  There were problems handling concurrent writing to
  pytables files. The package now logs to CSV and then
  provides a function to post import the data into
  SOFIEHDFFORMAT.
* Exporting to a CSV. Does not work yet.
* (catkin)Exporting to a CSV. Does not work yet.
* Added a close on terminate signal handler.
* (catkin)Added a close on terminate signal handler.
* Made the marker size be set via a parameter to the launch file.
* (catkin)Made the marker size be set via a parameter to the launch file.
* Changed the Callibration data.
* (catkin)Changed the Callibration data.
* The ar_pose listener.
* (catkin)The ar_pose listener.
* Changed the sofie driver to directly safe the ar_pose data.
  We are going to perform experiments and this means that the extra
  data might be useful at a later stage.
* (catkin)Changed the sofie driver to directly safe the ar_pose data.
  We are going to perform experiments and this means that the extra
  data might be useful at a later stage.
* Changed the size of the marker.
* Updated the usb_cam config to work for home camera.
* Added callibration files and launch files.
* Turned off history.
* (catkin)Added some comments and renamed.
* Added some comments and renamed.
* (catkin)The Quaternions were mixed around. Fixed the launch file to log to file instead of screen.
* The Quaternions were mixed around. Fixed the launch file to log to file instead of screen.
* (catkin)Updating the README's.
* Updating the README's.
* Updated the launch file to launch ar_pose and rviz for debugging.
* (catkin)Added arguments to the launch script.
* Added arguments to the launch script.
* Added the Stack formating files.
* (catkin)Organising into a stack instead of separate packages.
* Organising into a stack instead of separate packages.
* Trying to figure out how to start and stop the node.
* Adding simple parameters.
* Added the ROS files.
* Basic driver now works for listening on a channel that broadcasts geometry_msgs.msg.QuaternionStamped messages.
* Working on the listerner that will write to HDFFormat.
* Creating a listerner that can write to sofiehdfformat files.
* Initial commit
* Contributors: Adrian Cooke, Russell Toris, ¨Adrian
