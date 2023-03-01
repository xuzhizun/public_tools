/*!
 * \file    get_first_pixel.cpp
 * \author  IDS Imaging Development Systems GmbH
 * \date    2022-06-01
 * \since   1.0.0
 *
 * \brief   This application demonstrates how to use the device manager to open a camera
 *          and to display the first pixel value using the IDS peak IPL
 *
 * \version 1.2.1
 *
 * Copyright (C) 2019 - 2022, IDS Imaging Development Systems GmbH.
 *
 * The information in this document is subject to change without notice
 * and should not be construed as a commitment by IDS Imaging Development Systems GmbH.
 * IDS Imaging Development Systems GmbH does not assume any responsibility for any errors
 * that may appear in this document.
 *
 * This document, or source code, is provided solely as an example of how to utilize
 * IDS Imaging Development Systems GmbH software libraries in a sample application.
 * IDS Imaging Development Systems GmbH does not assume any responsibility
 * for the use or reliability of any portion of this document.
 *
 * General permission to copy or modify is hereby granted.
 */

#define VERSION "1.2.0"

#include <cstdint>
#include <iostream>

#include <peak/converters/peak_buffer_converter_ipl.hpp>
#include <peak/peak.hpp>
#include <peak_ipl/peak_ipl.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


/*! \bief Wait for enter function
 *
 * The function waits for the user pressing the enter key.
 *
 * This function is called from main() whenever the program exits,
 * either in consequence of an error or after normal termination.
 */
void wait_for_enter();

int main(int argc, char** argv)
{
    bool exposure_setting = false;
    bool frame_setting = false;
    double exposureTime;
    double frameRate;

    if(argc < 2)
    {
        std::cout<<"Defualt mode"<<'\n'
		<<"-e: for exposure time;"<<'\n'
		<<"-r: for framerate."<<'\n';
    }
    else
    {
	 for(int i = 1; i<argc ; i++)
	 {
	     if( std::string(argv[i]) == "-e")
	     {
		 exposureTime = std::stoi(argv[i+1]);
		 std::cout<<"exposureTime: "<<exposureTime<<'\n';
		 exposure_setting = true;
	     }
		   
		   
	     if( std::string(argv[i]) == "-r")
	     {
		 frameRate = std::stoi(argv[i+1]);
		 std::cout<<"frameRate: "<<frameRate<<'\n';
		 frame_setting = true;
	     }
	 }

	 if(!(frame_setting || exposure_setting))
		    std::cout<<"invalidate parameters, enter the default mode"<<'\n'
		                <<"-e: for exposure time;"<<'\n'
		                <<"-r: for framerate."<<'\n';
    }

    std::cout << "IDS peak API \"OpenCV Application\" Sample v" << VERSION << std::endl;

    // initialize peak library
    peak::Library::Initialize();

    // create a camera manager object
    auto& deviceManager = peak::DeviceManager::Instance();

    try
    {
        // update the cameraManager
        deviceManager.Update();

        // exit program if no camera was found
        if (deviceManager.Devices().empty())
        {
            std::cout << "No camera found. Exiting program." << std::endl << std::endl;
            wait_for_enter();
            // close library before exiting program
            peak::Library::Close();
            return 0;
        }

        // list all available devices
        uint64_t i = 0;
        std::cout << "Devices available: " << std::endl;
        for (const auto& deviceDescriptor : deviceManager.Devices())
        {
            std::cout << i << ": " << deviceDescriptor->ModelName() << " ("
                      << deviceDescriptor->ParentInterface()->DisplayName() << "; "
                      << deviceDescriptor->ParentInterface()->ParentSystem()->DisplayName() << " v."
                      << deviceDescriptor->ParentInterface()->ParentSystem()->Version() << ")" << std::endl;
            ++i;
        }

        // select a camera to open
        size_t selectedDevice = 0;
        // select a camera to open via user input or remove these lines to always open the first available camera
        std::cout << std::endl << "Select camera to open: ";
        std::cin >> selectedDevice;

        // open the selected camera
        auto device = deviceManager.Devices().at(selectedDevice)->OpenDevice(peak::core::DeviceAccessType::Control);
        // get the remote device node map
        auto nodeMapRemoteDevice = device->RemoteDevice()->NodeMaps().at(0);

        std::shared_ptr<peak::core::DataStream> dataStream;
        try
        {
            // Open standard data stream
            dataStream = device->DataStreams().at(0)->OpenDataStream();
        }
        catch (const std::exception& e)
        {
            // Open data stream failed
            device.reset();
            std::cout << "Failed to open DataStream: " << e.what() << std::endl;
            std::cout << "Exiting program." << std::endl << std::endl;

            wait_for_enter();
            // close library before exiting program
            peak::Library::Close();
            return 0;
        }

        // general preparations for untriggered continuous image acquisition
        // load the default user set, if available, to reset the device to a defined parameter set
        try
        {
            nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector")
                ->SetCurrentEntry("Default");
            nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("UserSetLoad")->Execute();
	    //load default set

            // wait until the UserSetLoad command has been finished
            nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("UserSetLoad")->WaitUntilDone();

	    if(exposure_setting)
              nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->SetValue(exposureTime);
	    //set the exposure time

	    if(frame_setting)
              nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->SetValue(frameRate);
	    //set the framerate
        }
        catch (const std::exception&)
        {
            // UserSet is not available, try to disable ExposureStart or FrameStart trigger manually
            std::cout << "Failed to load UserSet Default. Manual freerun configuration." << std::endl;

            try
            {
                nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
                    ->SetCurrentEntry("ExposureStart");
                nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")
                    ->SetCurrentEntry("Off");
            }
            catch (const std::exception&)
            {
                try
                {
                    nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
                        ->SetCurrentEntry("FrameStart");
                    nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")
                        ->SetCurrentEntry("Off");
                }
                catch (const std::exception&)
                {
                    // There is no known trigger available, continue anyway.
                }
            }
        }

	double exposureTime_current = nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->Value();
	double frameRate_current = nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->Value();

	std::cout<<"Current Exposure Time: "<<exposureTime_current<<'\n'
		<<"Current FrameRate : "<<frameRate_current<<'\n';

        // allocate and announce image buffers
        auto payloadSize = nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PayloadSize")->Value();
        auto bufferCountMax = dataStream->NumBuffersAnnouncedMinRequired();
        for (uint64_t bufferCount = 0; bufferCount < bufferCountMax; ++bufferCount)
        {
            auto buffer = dataStream->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize), nullptr);
            dataStream->QueueBuffer(buffer);
        }

        // set a frame rate to 10fps (or max value) since some of the trigger cases require a defined frame rate
        auto frameRateMax = nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")
                                ->Maximum();
        //nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")
        //   ->SetValue(std::min(10.0, frameRateMax));

	//set frame rate to max value
        nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")
           ->SetValue(frameRateMax);

        // define the number of images to acquire

        // Lock critical features to prevent them from changing during acquisition
        nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(1);

        // start acquisition
        dataStream->StartAcquisition(peak::core::AcquisitionStartMode::Default, peak::core::DataStream::INFINITE_NUMBER);
        nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart")->Execute();

        // process the acquired images
        std::cout << std::endl << "Acquired Images shown by opencv, type q to quit" << std::endl;

	//create opencv image window
	//cv::namedWindow("display", cv::WINDOW_AUTOSIZE);

	bool acquisitionContinue = true;
        while (acquisitionContinue)
        {
            // get buffer from datastream and create IDS peak IPL image from it
            auto buffer = dataStream->WaitForFinishedBuffer(5000);
            auto image = peak::BufferTo<peak::ipl::Image>(buffer);

	    //show the image by opencv 
	    //
	    cv::Mat cvImage;
            cvImage = cv::Mat::zeros(image.Height(), image.Width(), CV_8UC1);

            int sizeBuffer = static_cast<int>(image.ByteCount());
            // Device buffer is being copied into cv::Mat array
            std::memcpy(cvImage.data, image.Data(), static_cast<size_t>(sizeBuffer));
						long epoch_time = std::system_clock::duration<std::chrono::milliseconds>((std::chrono::system_clock::now()).time_since_epoch());

						cv::imwrite(std::to_string(epoch_time)+".jpg", cvImage);

	    //cv::imshow("display", cvImage);

	    char c = cv::waitKey(5);
	    if(c =='q')
		 acquisitionContinue = false;

	    //opencv show end

            // queue buffer
            dataStream->QueueBuffer(buffer);
        }
        std::cout << std::endl << std::endl;

        // stop acquistion of camera
        try
        {
            dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
        }
        catch (const std::exception&)
        {
            // Some transport layers need no explicit acquisition stop of the datastream when starting its
            // acquisition with a finite number of images. Ignoring Errors due to that TL behavior.

            std::cout << "WARNING: Ignoring that TL failed to stop acquisition on datastream." << std::endl;
        }
        nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->Execute();

        // Unlock parameters after acquisition stop
        nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(0);

        // flush and revoke all buffers
        dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);
        for (const auto& buffer : dataStream->AnnouncedBuffers())
        {
            dataStream->RevokeBuffer(buffer);
        }
    }
    catch (const std::exception& e)
    {
        std::cout << "EXCEPTION: " << e.what() << std::endl;
    }

    wait_for_enter();
    // close library before exiting program
    peak::Library::Close();
    return 0;
}


void wait_for_enter()
{
    std::cout << std::endl;
#if defined(WIN32)
    system("pause");
#elif defined(__linux__)
    std::cout << "Press enter to exit." << std::endl;
    system("read _");
#else
#    warning("Operating system not implemented!")
#endif
}
