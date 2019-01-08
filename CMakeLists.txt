cmake_minimum_required(VERSION 3.12)
project(CodeBall)

ADD_DEFINITIONS(-DLOCAL=1)
ADD_DEFINITIONS(-DDRAWLR=1)
ADD_DEFINITIONS(-DDEBUG=1)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

add_executable(CodeBall
        H.cpp
        MyStrategy.cpp
        RemoteProcessClient.cpp
        Runner.cpp
        Strategy.cpp
        model/C.cpp
        model/P.cpp)

add_subdirectory(RewindClient/csimplesocket)
target_link_libraries(CodeBall csimplesocket)

