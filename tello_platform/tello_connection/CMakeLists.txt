cmake_minimum_required(VERSION 3.13)  # CMake version check
set(PROJECT_NAME tello_connection)
project(${PROJECT_NAME})               # Create project "simple_example"
set(CMAKE_CXX_STANDARD 17)            # Enable c++17 standard
set(CMAKE_CXX_STANDARD_REQUIRED 17)

include_directories(
    include
)

find_package(OpenCV REQUIRED)
find_package (Threads)


# Add main.cpp file of project root directory as source file
set(CLIENT_FILES src/clientUdp.cpp src/socketUdp.cpp src/tello.cpp)
set(SERVER_FILES src/socketUdp.cpp src/serverUdp.cpp)
set(CLIENT_TELLO src/client_prueba.cpp)
# Add executable target with source files listed in SOURCE_FILES variable
add_executable(client ${CLIENT_FILES})
target_link_libraries(client ${OpenCV_LIBS})
target_link_libraries (client ${CMAKE_THREAD_LIBS_INIT})
#add_executable(server ${SERVER_FILES})
#add_executable(clientTello ${CLIENT_TELLO})
