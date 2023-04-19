A python module to interface with MS5837-30BA and MS5837-02BA waterproof pressure and temperature sensors. Tested on Raspberry Pi 3 with Raspbian.

Installation
The python SMBus library must be installed.

sudo apt-get install python-smbus2
Download this repository by clicking on the download button in this webpage, or using git:

git clone https://github.com/bluerobotics/ms5837-python
If you would like to try the example, move to the directory where you downloaded the repository, and run python example.py. To use the library, copy the ms5837.py file to your project/program directory and use this import statement in your program: import ms5837.

Raspberry Pi
If you are using a Raspberry Pi, the i2c interface must be enabled. Run sudo raspi-config, and choose to enable the i2c interface in the interfacing options.
