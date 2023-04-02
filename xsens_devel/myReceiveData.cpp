
//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

//--------------------------------------------------------------------------------
// Public Xsens device API C++ example MTi receive data.
//--------------------------------------------------------------------------------
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>

#include <iostream>
#include <iomanip>
#include <list>
#include <string>

#include <fstream>
#include <chrono>
#include <ctime>
#include <signal.h>

Journaller* gJournal = 0;

using namespace std;

class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5)
		: m_maxNumberOfPacketsInBuffer(maxBufferSize)
		, m_numberOfPacketsInBuffer(0)
	{
	}

	virtual ~CallbackHandler() throw()
	{
	}

	bool packetAvailable() const
	{
		xsens::Lock locky(&m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}

	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		xsens::Lock locky(&m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
	{
		xsens::Lock locky(&m_mutex);
		assert(packet != 0);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
			(void)getNextPacket();

		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
	mutable xsens::Mutex m_mutex;

	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	list<XsDataPacket> m_packetBuffer;
};


bool interrupt = false;
void signal_callback_handler(int signum){
  cout<<"Caught signal "<<signum<<endl;
  interrupt = true;
}


//--------------------------------------------------------------------------------
int main(void)
{
	cout << "Creating XsControl object..." << endl;
	XsControl* control = XsControl::construct();
	assert(control != 0);

	// Lambda function for error handling
	auto handleError = [=](string errorString)
	{
		control->destruct();
		cout << errorString << endl;
		cout << "Press [ENTER] to continue." << endl;
		cin.get();
		return -1;
	};

	cout << "Scanning for devices..." << endl;
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();

	// Find an MTi device
	XsPortInfo mtPort;
	for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			mtPort = portInfo;
			break;
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found. Aborting.");

	cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;

	cout << "Opening port..." << endl;
	if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
		return handleError("Could not open port. Aborting.");

	// Get the device object
	XsDevice* device = control->device(mtPort.deviceId());
	assert(device != 0);

	cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << endl;

	// Create and attach callback handler to device
	CallbackHandler callback;
	device->addCallbackHandler(&callback);

	// Put the device into configuration mode before configuring the device
	cout << "Putting device into configuration mode..." << endl;
	if (!device->gotoConfig())
		return handleError("Could not put device into configuration mode. Aborting.");

	cout << "Configuring the device..." << endl;

	// Important for Public XDA!
	// Call this function if you want to record a mtb file:
	device->readEmtsAndDeviceConfiguration();

	XsOutputConfigurationArray configArray;
	configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

	if (device->deviceId().isImu())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
	}
	else if (device->deviceId().isVru() || device->deviceId().isAhrs())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
	}
	else if (device->deviceId().isGnss())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
		configArray.push_back(XsOutputConfiguration(XDI_LatLon, 100));
		configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 100));
		configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, 100));
		configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
	}
	else
	{
		return handleError("Unknown device while configuring. Aborting.");
	}

	if (!device->setOutputConfiguration(configArray))
		return handleError("Could not configure MTi device. Aborting.");

  cout<<"Creating a text file..."<<endl;

  time_t currTime = time(0);
  char* dt = ctime(&currTime);
  std::string date(dt,20);

  ofstream file(date+"imu_data.txt");

  if(file.is_open())
    cout<<"created a text file: "+date+"imu_data.txt\n";
  else
    return handleError("Failed to create a text file. Aborting.");

	cout << "Creating a log file..." << endl;
	string logFileName = "logfile.mtb";
	if (device->createLogFile(logFileName) != XRV_OK)
		return handleError("Failed to create a log file. Aborting.");
	else
		cout << "Created a log file: " << logFileName.c_str() << endl;

	cout << "Putting device into measurement mode..." << endl;
	if (!device->gotoMeasurement())
		return handleError("Could not put device into measurement mode. Aborting.");

	cout << "Starting recording..." << endl;
	if (!device->startRecording())
		return handleError("Failed to start recording. Aborting.");

	int64_t startTime = XsTime::timeStampNow();
	//while (XsTime::timeStampNow() - startTime <= 10000)
  //
  signal(SIGINT, signal_callback_handler);

	while (true && !interrupt)
	{
		if (callback.packetAvailable())
		{
			cout << setw(5) << fixed << setprecision(2);

			// Retrieve a packet
			XsDataPacket packet = callback.getNextPacket();

      auto epoch_time = std::chrono::system_clock::now().time_since_epoch();

      long time_stamp = std::chrono::duration_cast<std::chrono::milliseconds>(epoch_time).count();

      //record time stamp 
      file <<time_stamp;

			if (packet.containsCalibratedData())
			{
				XsVector acc = packet.calibratedAcceleration();
				cout << "\n"
					<< "Acc X:" << acc[0]
					<< ", Acc Y:" << acc[1]
					<< ", Acc Z:" << acc[2];

				XsVector gyr = packet.calibratedGyroscopeData();
				cout << " |Gyr X:" << gyr[0]
					<< ", Gyr Y:" << gyr[1]
					<< ", Gyr Z:" << gyr[2];

				XsVector mag = packet.calibratedMagneticField();
				cout << " |Mag X:" << mag[0]
					<< ", Mag Y:" << mag[1]
					<< ", Mag Z:" << mag[2];

        //record data in text file
        file <<" "<< "Acc X:" << acc[0]
					<< ", Acc Y:" << acc[1]
					<< ", Acc Z:" << acc[2]
				  << " |Gyr X:" << gyr[0]
					<< ", Gyr Y:" << gyr[1]
					<< ", Gyr Z:" << gyr[2]
				  << " |Mag X:" << mag[0]
					<< ", Mag Y:" << mag[1]
					<< ", Mag Z:" << mag[2];
			}

			if (packet.containsOrientation())
			{
				XsQuaternion quaternion = packet.orientationQuaternion();
				cout << "\n"
					<< "q0:" << quaternion.w()
					<< ", q1:" << quaternion.x()
					<< ", q2:" << quaternion.y()
					<< ", q3:" << quaternion.z();

				XsEuler euler = packet.orientationEuler();
				cout << " |Roll:" << euler.roll()
					<< ", Pitch:" << euler.pitch()
					<< ", Yaw:" << euler.yaw();

        //record data in text file
        file <<" " << "|q0:" << quaternion.w()
					<< ", q1:" << quaternion.x()
					<< ", q2:" << quaternion.y()
					<< ", q3:" << quaternion.z()
				  << " |Roll:" << euler.roll()
					<< ", Pitch:" << euler.pitch()
					<< ", Yaw:" << euler.yaw();
			}

			if (packet.containsLatitudeLongitude())
			{
				XsVector latLon = packet.latitudeLongitude();
				cout << " |Lat:" << latLon[0]
					<< ", Lon:" << latLon[1];
			}

			if (packet.containsAltitude())
				cout << " |Alt:" << packet.altitude();

			if (packet.containsVelocity())
			{
				XsVector vel = packet.velocity(XDI_CoordSysEnu);
				cout << " |E:" << vel[0]
					<< ", N:" << vel[1]
					<< ", U:" << vel[2];
			}

      file<<"\n";
			cout << flush;
		}
		XsTime::msleep(0);
	}

	cout << "Stopping recording..." << endl;
	if (!device->stopRecording())
		return handleError("Failed to stop recording. Aborting.");

	cout << "Closing log file..." << endl;
	if (!device->closeLogFile())
		return handleError("Failed to close log file. Aborting.");

	cout << "Closing port..." << endl;
	control->closePort(mtPort.portName().toStdString());

	cout << "Freeing XsControl object..." << endl;
	control->destruct();

  //close file
  file.close();

	cout << "Successful exit." << endl;

	cout << "Press [ENTER] to continue." << endl;
	cin.get();

	return 0;
}
