//The code for the water_linker DVL 50A 
//serial communication. The protocol offered by the manufactory
//can be obtained at https://waterlinked.github.io/dvl/dvl-protocol/#serial-protocol 
//author: Zhizun Xu
//The serial libraires are used.

#include <iostream>
#include <chrono>
#include <cstdlib>
#include <string>

#include "rOc_serial.h"



int main(int argc, char** argv){

	if(argc < 2){
		std::cout<<"Needed the device address \n for example: "<<argv[0]<<" /dev/ttyUSB0\n";
		return 1;
	}

	rOc_serial dvl_serial;
	
	dvl_serial.openDevice(argv[1], 115200);


	int N = 100;
  for(int i = 0; i<N; i++){

	  char receive_buffer[256];
	  dvl_serial.readString(receive_buffer,'\n', sizeof(receive_buffer));

		for( int c_i = 0; c_i < sizeof(receive_buffer); c_i++){
			  std::cout<<receive_buffer[c_i];
				if(receive_buffer[c_i] == '\n')
					break;
		}
		std::cout<<'\n';
	}

	dvl_serial.closeDevice();
	return 0;
}



	
