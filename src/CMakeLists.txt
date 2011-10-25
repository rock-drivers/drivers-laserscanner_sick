rock_library(laserscanner_sick
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(laserscanner_sick_bin Main.cpp
    DEPS laserscanner_sick)

