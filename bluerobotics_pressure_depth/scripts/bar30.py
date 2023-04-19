#!/usr/bin/env python3

import ms5837
import time

import rospy
import sensor_msgs.msg import FluidPressure 

import json
import csv

def pub():
   sensor = ms5837.MS5837_30BA() # Default I2C bus is 1 (Raspberry Pi 3)

# We must initialize the sensor before reading it
   if not sensor.init():
        print("Sensor could not be initialized")
        exit(1)

    pub=rospy.Publisher('pressure', FluidPressure, queue_size=10)
    rate = rospy.Rate(10)

    _msgs = FluidPressure()
    report = {'time_log':'', 'pressure':'', 'temperature':''}
# Print readings
   while not rospy.is_shutdown():
          if sensor.read():
                #sensor.pressure(ms5837.UNITS_psi), # Request psi
                #sensor.temperature(), # Default is degrees C (no arguments)
                #sensor.temperature(ms5837.UNITS_Farenheit))) # Request Farenheit
                _msgs.header = rospy.Time.now()
                _msgs.fluid_pressure=sensor.pressure() # Default is mbar (no arguments
                pub.publish(_msgs)
                if _writer is not None:
                  report["time_log"] = str(rospy.Time.now())
                  report["pressure"] = sensor.pressure() 
                  report["temperature"] = sensor.temperature() 

                  report = report.dumps(report)
                  message = json.loads(report)
                  _writer.writerow(message) 
           else:
                rospy.loginfo("Sensor read failed!")

            rate.sleep()

if __name__ == '__main__'
   global _write
   rospy.init_node('bar30', anonymous = True)
   do_log_raw_data = rospy.get_param("~do_log_raw_data", False)

   if(do_log_raw_data)
      fieldnames = ['time_log', 'pressure', 'temperature']
      _file = open('/tmp/bar30.csv', 'w')
      _writer = csv.DicWriter(_file, fieldnames=fieldnames, extrasaction='ignore', delimiter=' ')
      _writer.writeheader()

   try:
       pub()

   except rospy.ROSInterruptException:
       pass
  
