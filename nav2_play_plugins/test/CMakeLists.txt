# Test costmap downsampler
ament_add_gtest(test_noti_at_waypoint
  test_noti_at_waypoint.cpp
)
ament_target_dependencies(test_noti_at_waypoint
  ${dependencies}
)
target_link_libraries(test_noti_at_waypoint
  ${library_name} noti_at_waypoint
)

# Install the test executables
install(TARGETS test_noti_at_waypoint
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)