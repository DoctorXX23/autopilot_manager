add_library(
    mission-manager
    MissionManager.cpp
)

target_link_libraries(
    mission-manager
    autopilot-manager
)

target_include_directories(mission-manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/autopilot_manager/mission_manager>
    )

install(TARGETS mission-manager
    EXPORT mission-manager
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
