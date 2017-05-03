#setup arduino for the imu:
##Hardware
GT-86 from https://www.aliexpress.com/item/GY-86-10DOF-MS5611-HMC5883L-MPU6050-Module-MWC-Flight-Control-Sensor-Module/1986970073.html
arduino promicro from https://www.aliexpress.com/item/Free-Shipping-New-Pro-Micro-for-arduino-ATmega32U4-5V-16MHz-Module-with-2-row-pin-header/2021979132.html
##Software
download arduino 1.6.9
clone https://github.com/jarzebski/Arduino-MS5611.git into arduino-1.6.9/libraries/
clone https://github.com/jrowberg/i2cdevlib.git
copy from i2cdevlib/Arduino the libraries MPU6050,HMC5883L,I2Cdev into arduino-1.6.9/libraries/
copy https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050 into arduino-1.6.9/libraries/

