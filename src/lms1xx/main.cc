/*!
 * \file main.cc
 * \brief A simple application using the Sick LMS 1xx driver.
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2009, Jason C. Derenick and Christopher R. Mansley
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#include <string>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <math.h>
#include <lms1xx/SickLMS1xx.hh>

using namespace std;
using namespace SickToolbox;

SickLMS1xx* sick;
unsigned int status;
unsigned int num_measurements;
unsigned int range_1_vals[SickToolbox::SickLMS1xx::SICK_LMS_1XX_MAX_NUM_MEASUREMENTS];
unsigned int range_2_vals[SickToolbox::SickLMS1xx::SICK_LMS_1XX_MAX_NUM_MEASUREMENTS]; 
unsigned int reflect_1_vals[SickToolbox::SickLMS1xx::SICK_LMS_1XX_MAX_NUM_MEASUREMENTS];
unsigned int reflect_2_vals[SickToolbox::SickLMS1xx::SICK_LMS_1XX_MAX_NUM_MEASUREMENTS];
double resolution;
double start_angle;

int main(int argc, char* argv[])
{
    status = 1;
    num_measurements = 0; 
    resolution = 0.0;
    start_angle = 0.0;

    /*
    * Instantiate an instance
    */
    sick = new SickToolbox::SickLMS1xx(argv[1], atoi(argv[2]));

    /*
    * Initialize the Sick LMS 2xx
    */
    try {
	sick->Initialize();
    }

    catch(...) {
	cerr << "Initialize failed! Are you using the correct IP address?" << endl;
	return -1;
    }

    try{
	sick->SetSickScanDataFormat(SickToolbox::SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_DOUBLE_PULSE_REFLECT_16BIT);
    }
    catch(SickToolbox::SickConfigException sick_exception){
	std::cout << sick_exception.what() << std::endl;
	return -1;
    }

    if(sick->IntToSickScanFreq(50) == SickToolbox::SickLMS1xx::SICK_LMS_1XX_SCAN_FREQ_UNKNOWN)
	return -1;
    
    if(sick->DoubleToSickScanRes(0.5) == SickToolbox::SickLMS1xx::SICK_LMS_1XX_SCAN_RES_UNKNOWN)
	return -1;

    sick->SetSickScanFreqAndRes(sick->IntToSickScanFreq(50), sick->DoubleToSickScanRes(0.5));

    try{
	resolution = (sick->SickScanResToDouble(sick->GetSickScanRes()))/180.0*M_PI;
	//printf("Resolution: %f\n", sick->SickScanResToDouble(sick->GetSickScanRes()));
	start_angle = -(M_PI*0.5)+((sick->GetSickStartAngle())/180.0*M_PI);
	//printf("Frequency: %i\n", sick->SickScanFreqToInt(sick->GetSickScanFreq()));
    }
    catch(SickToolbox::SickIOException sick_exception) {
	std::cout << sick_exception.what() << std::endl;
	return -1;
    }

    vector<double> ranges, remission;
    int k = 0;
    while(1000){
	k++;
	try{
	    sick->GetSickMeasurements(range_1_vals,range_2_vals,reflect_1_vals,reflect_2_vals,num_measurements,&status);
	    for(int i=0; i<num_measurements; i++){
		//ranges.push_back(range_1_vals[i]);
		//std::cout << range_1_vals[i] << std::endl;
	    }
	    for(int i=0; i<num_measurements; i++){
		//remission.push_back(reflect_1_vals[i]);
		//std::cout << reflect_1_vals[i] << std::endl;
	    }
	}
	catch(SickToolbox::SickIOException sick_exception) {
	    std::cout << sick_exception.what() << std::endl;
	    exception();
	    return -1;
	}
	catch(SickToolbox::SickTimeoutException sick_exception) {
	    std::cout << sick_exception.what() << std::endl;
	    exception();
	    return -1;
	}
	catch(...) {
	    cerr << "An Error Occurred!" << endl;
	    exception();
	    return -1;
	}
    }

    if(sick)
        delete sick;
    sick = 0;

    /* Success! */
    return 0;
}
