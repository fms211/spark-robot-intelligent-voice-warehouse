cmake_minimum_required(VERSION 3.0.2)
project(spark_course_mission)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
)

add_message_files(FILES InspectionResult.msg)
add_action_files(FILES PickPlace.action)
add_service_files(FILES CaptureWaypoint.srv)
catkin_python_setup()
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs message_runtime std_msgs)

catkin_install_python(PROGRAMS
  scripts/voice_intent.py
  scripts/status_speaker.py
  scripts/color_inspector.py
  scripts/swiftpro_adapter.py
  scripts/mission_manager.py
  scripts/waypoint_capture.py
  scripts/import_move2grasp_calibration.py
  scripts/course_voice_asr.py
  scripts/calibration_position.py
  scripts/calibration_grid_run.py
  scripts/voice_move2grasp_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/calibration_prompt.sh
  scripts/calibration_confirm.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/default_launch.test)
  add_rostest(test/warehouse_launch.test)
  add_rostest(test/mission_mock.test)
  add_rostest(test/mission_fault.test)
  add_rostest(test/arm_mock.test)
  add_rostest(test/voice_intent.test)
  add_rostest(test/voice_fms_bridge.test)
  add_rostest(test/waypoint_capture.test)
endif()
