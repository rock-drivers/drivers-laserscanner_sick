/*!
 * \file SickLMS1xxBufferMonitor.cc
 * \brief Implements a class for monitoring the receive
 *        buffer when interfacing w/ a Sick LMS 1xx LIDAR.
 *
 * Code by Jason C. Derenick and Christopher R. Mansley.
 * Contact jasonder(at)seas(dot)upenn(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2009, Jason C. Derenick and Christopher R. Mansley
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

/* Auto-generated header */
//#include "SickConfig.hh"

/* Implementation dependencies */
#include <iostream>
#include <sys/ioctl.h>

#include "SickLMS1xxBufferMonitor.hh"
#include "SickLMS1xxMessage.hh"
#include "SickLMS1xxUtility.hh"
#include "SickException.hh"

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \brief A standard constructor
   */
  SickLMS1xxBufferMonitor::SickLMS1xxBufferMonitor( ) : SickBufferMonitor< SickLMS1xxBufferMonitor, SickLMS1xxMessage >(this) { }

  /**
   * \brief Acquires the next message from the SickLMS1xx byte stream
   * \param &sick_message The returned message object
   */
  void SickLMS1xxBufferMonitor::GetNextMessageFromDataStream( SickLMS1xxMessage &sick_message ) throw( SickIOException ) {

    /* Flush the input buffer */
    uint8_t payload_buffer[SickLMS1xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    int payload_length = 0;

    try{  
	payload_length = readPacket(payload_buffer, SickLMS1xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH, 1000, 1000);
	
 
      /* Build the return message object based upon the received payload
       * NOTE: In constructing this message we ignore the header bytes
       *       buffered since the BuildMessage routine will insert the
       *       correct header automatically and verify the message size
       */
      sick_message.BuildMessage(payload_buffer+1, payload_length-1);

      /* Success */
      
    }
    
    catch(iodrivers_base::TimeoutError &sick_timeout) { /* This is ok! */ }

    /* Catch any serious IO buffer exceptions */
    catch(SickIOException &sick_io_exception) {
      throw;
    }
    catch(iodrivers_base::UnixError &e) {
      std::cout << e.what();
      throw;
    }
    
    /* A sanity check */
    catch (...) {
      throw;
    }
    
  }

  /**
   * \brief Flushes TCP receive buffer contents
   */
  void SickLMS1xxBufferMonitor::_flushTCPRecvBuffer( ) throw (SickIOException) {
   	
	//Simply calling clear in iodriver ald let the iodriver do the rest
	clear();
  }
  
  /**
   * \brief A standard destructor
   */
  SickLMS1xxBufferMonitor::~SickLMS1xxBufferMonitor( ) { }
  
  int SickLMS1xxBufferMonitor::extractPacket(uint8_t const* buffer, size_t buffer_size) const{
	unsigned int readPos = 0;

	//Searching for 0x02 for package start
	while(buffer[readPos] != 0x02 && readPos < buffer_size){
		readPos++;
	}

	//If package start was not 0, returning offset to get calles again
	if(readPos > 0){
		return -readPos;
	}
	
	//Package start found so search for end of packet
	while(buffer[readPos] != 0x03 && readPos < buffer_size){
		readPos ++;
	}

	if(readPos == buffer_size){ //End of stream not found start, so packed is not complete yet
		return 0;
	}

	//Found package end, so give package len
	return readPos;
  }
  
} /* namespace SickToolbox */
