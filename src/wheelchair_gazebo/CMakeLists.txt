cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_gazebo)

find_package(catkin REQUIRED)

catkin_package()
catkin_install_python(PROGRAMS
  scripts/sim_evidence_bridge.py
  scripts/rc_metrics_collector.py
  scripts/simulation_controller_adapter.py
  scripts/sim_sensor_canonicalizer.py
  scripts/rc_scenario_driver.py
  scripts/rc_fault_injector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test_rc_world_static.py)
  catkin_add_nosetests(tests/test_hanyang_gazebo_model.py)
  catkin_add_nosetests(tests/test_simulation_controller_adapter.py)
  catkin_add_nosetests(tests/test_sim_evidence_bridge.py)
  catkin_add_nosetests(tests/test_rc_metrics_collector.py)
  catkin_add_nosetests(tests/test_sim_sensor_canonicalizer.py)
  catkin_add_nosetests(tests/test_rc_scenario_driver.py)
  catkin_add_nosetests(tests/test_rc_fault_injector.py)
endif()

install(DIRECTORY config launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
