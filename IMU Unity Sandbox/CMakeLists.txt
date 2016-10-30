cmake_minimum_required (VERSION 2.8)

project (IMUSandbox)

#add_subdirectory(src)

file(GLOB src src/*.cpp)
file(GLOB include include/*.h*)

include_directories(include)

#find_package(libbcm2835 REQUIRED)
#include_directories(${libbcm2835_INCLUDE_DIRS})
#set(LIBS ${LIBS} ${libbcm2835_LIBRARIES})

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
#find_package(Boost COMPONENTS asio)
find_package(Boost REQUIRED COMPONENTS asio)

#if(Boost_FOUND || true)
    include_directories(${Boost_INCLUDE_DIRS}) 
    #target_link_libraries(progname ${Boost_LIBRARIES})


add_executable(IMUSandbox ${src} ${include})

TARGET_LINK_LIBRARIES(IMUSandbox bcm2835 ${Boost_LIBRARIES})

#endif()
