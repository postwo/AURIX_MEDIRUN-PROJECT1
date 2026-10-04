# MissionController 단위 시험.
# ROS·DB·YAML 의존 클래스(MissionRosInterface, MissionStorageService, waypointfile)는
# 실제 구현 대신 fakes.cpp 의 대역을 링크하므로 Qt Core/Test 만 있으면 빌드·실행된다.
find_package(Qt6 REQUIRED COMPONENTS Core Test)

add_executable(test_missioncontroller
    test_missioncontroller.cpp
    fakes.cpp
    fakes.h
    ../src/services/missioncontroller.cpp
    ../src/services/missioncontroller_events.cpp
    ../src/services/missioncontroller_queue.cpp
    ../src/services/missioncontroller_returnflow.cpp
    ../src/services/missioncontroller_dispatch.cpp
    ../src/services/missioncontroller_state.cpp
    ../src/services/missioncontroller_helpers.h
    ../src/services/missioncontroller.h
    ../src/services/missionrosinterface.h
    ../src/services/missionstorageservice.h
    ../src/services/waypointrepository.cpp
    ../src/services/waypointrepository.h
    ../src/ros/navigationservice.h
)
set_target_properties(test_missioncontroller PROPERTIES AUTOMOC ON)
target_compile_features(test_missioncontroller PRIVATE cxx_std_17)
target_include_directories(test_missioncontroller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_missioncontroller PRIVATE Qt6::Core Qt6::Test)

add_test(NAME test_missioncontroller COMMAND test_missioncontroller)
