project(camera_gmsl_publisher C CXX)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(PUBLIC_DOCS
    README.md
)

set(SOURCES
    main.cpp
)

message(${Driveworks_LIBRARIES})

set(LIBRARIES
    ${Driveworks_LIBRARIES}
    nvmedia_ipp
)

# Gerekli paketleri bul
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(OpenCV 2.4.9 REQUIRED)

# Include dizinleri
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${VIBRANTE_PDK}/targetfs/opt/ros/kinetic/include)
include_directories(${VIBRANTE_PDK}/targetfs/usr/include)
include_directories(${VIBRANTE_PDK}/targetfs/usr/include/aarch64-linux-gnu)

# Link dizinleri
link_directories(${VIBRANTE_PDK}/targetfs/opt/ros/kinetic/lib 
${VIBRANTE_PDK}/targetfs/lib/aarch64-linux-gnu
)

#-------------------------------------------------------------------------------
# Final target
#-------------------------------------------------------------------------------

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE 
                    ${LIBRARIES} 
                    roscpp 
                    rosconsole 
                    roscpp_serialization 
                    rostime 
                    ${OpenCV_LIBS}
                    X11 
                    ${Boost_LIBRARIES})
                    
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Samples")

# ------------------------------------------------------------------------------
# Install target
# ------------------------------------------------------------------------------
sdk_add_sample(${PROJECT_NAME})
