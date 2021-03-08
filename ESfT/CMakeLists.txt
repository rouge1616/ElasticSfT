cmake_minimum_required(VERSION 3.1)
project(GraphRegistration)


set(HEADER_FILES
    ProjectiveSetup.h
    WeightedRestShapeSpringsForceField.h
    WeightedRestShapeSpringsForceField.inl
    ObjectPoser.h
    MatchingEngine.h
    Rigid2Vec3d.h
    ComputeError.h
    CameraPoser.h
    config.h
    Recorded2Camera.h
    SolvePnP.h
)

set(SOURCE_FILES
    ProjectiveSetup.cpp
    WeightedRestShapeSpringsForceField.cpp
    initGraphRegistration.cpp
    ObjectPoser.cpp
    CameraPoser.cpp
    MatchingEngine.cpp
    Rigid2Vec3d.cpp
    ComputeError.cpp
    Recorded2Camera.cpp
    SolvePnP.cpp
)

find_package(SofaFramework REQUIRED)
find_package(SofaGeneral REQUIRED)
find_package(SofaOpenglVisual REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core highgui video features2d videoio imgproc flann photo)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_PLUGIN_GRAPHREGISTRATION")
target_link_libraries(${PROJECT_NAME} SofaDeformable SofaCore SofaOpenglVisual ${OpenCV_LIBS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/..")

install(TARGETS ${PROJECT_NAME}
        COMPONENT GraphRegistration_libraries
        EXPORT GraphRegistrationTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

