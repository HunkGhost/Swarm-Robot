add_library(my MODULE my.h my.cpp)
target_link_libraries(my
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
