rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0997 image:=/image_raw camera:=/my_camera --no-service-check

rosrun  camera_calibration_parsers convert  ost.ini cal.yml
