PROGRESS REPORT

The project is being done under guidance of Rishabh Singh (Robolution, BIT Mesra)


The mechanical structure for the bot has been made at Robolution’s arena at BIT Mesra with the help of mech-sub team. 

The bot uses MPU 6050 to get the values of angle of inclination of bot wrt vertical axis. Calibration of MPU is complete and the code now uses both, data from gyroscope as well as accelerometer to compute the angle. 

L293D motor driver is being used to drive the two 100 rpm motors.

Currently doing PID tuning to make the bot more stable and trying to use Zeigler Nicholas’ method to get good values for P, I and D. 
