add_executable(jacocpp src/jacocpp.cpp)
target_link_libraries(jacocpp ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS jacocpp DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
