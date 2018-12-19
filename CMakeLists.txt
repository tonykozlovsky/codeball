cmake_minimum_required(VERSION 3.13)
project(CodeBall)

#ADD_DEFINITIONS(-DLOCAL=1)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

add_executable(CodeBall
        Helper.cpp
        MyStrategy.cpp
        RemoteProcessClient.cpp
        Runner.cpp
        Strategy.cpp)

add_subdirectory(RewindClient/csimplesocket)
target_link_libraries(CodeBall csimplesocket)

