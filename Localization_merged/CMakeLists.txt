CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(Location)


ADD_DEFINITIONS("-Wall -g")

SET(CMAKE_CXX_COMPILER "g++")  
if(CMAKE_COMPILER_IS_GNUCXX)
   execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
   if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
        message(STATUS "C++11 activated.")
        add_definitions("-std=gnu++11")
   elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
        message(WARNING "C++0x activated. If you get any errors update to a compiler which fully supports C++11")
        add_definitions("-std=gnu++0x")
   else ()
        message(FATAL_ERROR "C++11 needed. Therefore a gcc compiler with a version higher than 4.3 is needed.")   
   endif()
else(CMAKE_COMPILER_IS_GNUCXX)
   add_definitions("-std=c++11") 
endif(CMAKE_COMPILER_IS_GNUCXX)


FIND_PACKAGE(Threads)
FIND_PACKAGE(PCL 1.8 REQUIRED)
#FIND_PACKAGE(Pangolin REQUIRED)

INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)
MESSAGE(${PROJECT_SOURCE_DIR})
SET(INC_DIR /usr/include/dbus-1.0/dbus)
INCLUDE_DIRECTORIES(INC_DIR)
MESSAGE(${INC_DIR})


#LINK_DIRECTORIES({LINK_DIR})
#LINK_LIBRARIES(starlocation-ntrip)


AUX_SOURCE_DIRECTORY(src DIR_SRCS)

SET(LOCATION ${DIR_SRCS})

ADD_EXECUTABLE(${PROJECT_NAME} ${LOCATION})
TARGET_LINK_LIBRARIES (${PROJECT_NAME}  ${boost_LIBRARIES} ${Pangolin_LIBRARIES} ${PCL_LIBRARIES} -lboost_thread -lpthread -ldbus-1  -lqxwz-1.3.2 -g)

