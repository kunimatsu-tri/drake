find_package(OpenCV)
include_directories(${OpenCV_DIRS})

if(lcm_FOUND)
  add_executable(rgbd_camera_sim rgbd_camera_sim.cc)
  target_link_libraries(rgbd_camera_sim
    drakeLcmSystem
    drakeMultibodyParsers
    drakeRigidBodyPlant
    drakeSensors
    drakeSystemAnalysis
    drakeSystemFramework
    drakeSystemPrimitives
    gflags
    lcm
    ${OpenCV_LIBS}
  )
endif()
