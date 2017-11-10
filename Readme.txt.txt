Order of execution

ROSMATLABNode.m
initZigBee.m
moveRobot.m
TrajSpeedTest / RezaeeObstAvoidAlgo / formationWithoutObstacle


Callback function ZigbeeRcvCallback is executed on serial receive in MATLAB. currently works when Zumo is loaded with sendToPCviaZigbee.ino




caveat: gains in caclcSpeedsWhileMoving are not yet properly tuned according to mm/sec commands in sendSpeedsCaharacterWise.

calcSpeeds (turn thern move), sendSpeeds (send PWM values) are now no longer used so take care to use the right one in the scripts.

The variable robotIds is usually changed depending on the number of robots in operation as well as commenting out sendSpeeds line of inactive robot (wastes time in wireless transmission).
