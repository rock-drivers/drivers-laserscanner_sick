
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/base
		    ${CMAKE_CURRENT_SOURCE_DIR}
		    )

rock_library(laserscanner_sick
    SOURCES ld/SickLDBufferMonitor.cc 
	    ld/SickLD.cc 
            ld/SickLDMessage.cc
            lms1xx/SickLMS1xxBufferMonitor.cc
            lms1xx/SickLMS1xx.cc
            lms1xx/SickLMS1xxMessage.cc
            lms2xx/SickLMS2xxBufferMonitor.cc
            lms2xx/SickLMS2xx.cc
            lms2xx/SickLMS2xxMessage.cc
             
    HEADERS base/SickBufferMonitor.hh 
	    base/SickException.hh
            base/SickLIDAR.hh 
	    base/SickMessage.hh
            ld/SickLDBufferMonitor.hh 
	    ld/SickLDUtility.hh
            ld/SickLDMessage.hh 
	    ld/SickLD.hh
            lms1xx/SickLMS1xxUtility.hh
            lms1xx/SickLMS1xxBufferMonitor.hh
            lms1xx/SickLMS1xx.hh
            lms1xx/SickLMS1xxMessage.hh
            lms2xx/SickLMS2xxUtility.hh
            lms2xx/SickLMS2xxBufferMonitor.hh
            lms2xx/SickLMS2xx.hh
            lms2xx/SickLMS2xxMessage.hh)

rock_executable(laserscanner_sick_app lms1xx/main.cc
		DEPS laserscanner_sick)

target_link_libraries(laserscanner_sick pthread)


