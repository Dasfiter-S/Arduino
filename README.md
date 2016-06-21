Vendbot library Version 3.0
------------------------------
Grouped all libraries for better version control. Increased the power of the robot from 7v to 12v. The 7v code does not properly support the 12v control of the motors. Started 12v library. Competition code now completed for 12v and can complete the track in 32 seconds. Had to remove the touchscreen due to power drain from the motors while running and interferring with the total run time.

Vendbot library Version 2.1.0
-----------------------------
Competition_2016_WTouch_v2 was added. This file contains touchscreen support. it displays time on rouch intervals. It updates the next and last turns. It also has a diagnostics page that prints out the current line sensor state at a set refresh rate of about 300ms. Also added Arduino V3.HMI for the Touchscreen

Vendbot library Version 2.0.2
-----------------------------
TestDriveQTI_V3_Tim_nav_working.ino now supports true C statements for reading and writing to the line sensors.

Vendbot library Version 2.0.1
-----------------------------
Added Competition_2016_WTouch.ino. Added Arduino Controls V2.hmi this file does not update time constantly flooding the TX RX system on the touchscreen. Buttons work.


Vendbot library Version 2.0.0
-----------------------------
Navigation now working semi-reliably. The robot can traverse the entire track. Any reading of "00000" which means there is nothing will result in the robot backing up and trying to find the line again. 
Incorporated a Nextion touchscreen for easier debugging and ring loading.
Added the Nextion Arduino Controls.hmi file. Added touchScreen.ino

Vendbot library Version 1.3.3
-------------------------------
Fixed the slightLeft() and slightRight() functions. They now work as intended. The robot can now navigate the track with 50% accuracy. Added Tim's navigation method. New file added TestDRiveQTI_V3_Tim_Nav.

Vendbot library Version 1.3.2
-------------------------------
All changes done to TestDriveQTI_V3
Removed Pow functions from isOnLine() function. It was simply a waste of processing power to calculate a constant that could simply be multiplied in. Changed delay() in the motor functions. Cleaned up some of the navigation tests. Readjusted motor directions.

Vendbot library Version 1.3.1
-------------------------------

TestDriveQTI_V3 code has not changed. Added MotorVNH and MotorVNH_Servo. MotorVNH tests the motorshield with default values for calibration. MotorVNH_Servo calibrates the motors and the servos for simultaneous control. Also added Open_CV for VS 2013 express. Needs work on the navigation algorithm.

Vendbot library Version 1.3
-------------------------------

Use TestDriveQTI_V3, the code has been simplified and it now contains a virtual track. It is still a work in progress.
Pathing 2016.txt is the current pathing algoithm we are using.
Track Path.png is the visual path.
Current Path input.txt is what the robot is doing and when it is doing it as it traverses the map.
TestPhotoResistor has been added.

Vendbot library Version 1.2
--------------------------------
Added TestDriveQTI_V2
Added Track Path.png
Added Pathing 2016.txt

Vendbot library Version 1.1
--------------------------------
Added TestDriveQTI_V1
