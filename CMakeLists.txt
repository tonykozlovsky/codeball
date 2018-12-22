cmake_minimum_required(VERSION 3.13)
project(CodeBall)

ADD_DEFINITIONS(-DLOCAL=1)
ADD_DEFINITIONS(-DDRAWLR=1)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

add_executable(CodeBall
        Helper.cpp
        MyStrategy.cpp
        RemoteProcessClient.cpp
        Runner.cpp
        Strategy.cpp
        model/Constants.cpp
        model/Painter.cpp)

add_subdirectory(RewindClient/csimplesocket)
target_link_libraries(CodeBall csimplesocket)

