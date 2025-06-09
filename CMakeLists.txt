cmake_minimum_required(VERSION 3.27)
project(Autopilot)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MAIN
        src/defines.h
        src/config.h
        src/autopilot.h
)

set(MPC
        src/mpc/optimization/optimization.h
        src/mpc/optimization/mopso.h
        src/mpc/mpc.h
        src/util.h

        src/mpc/optimization/optimization.cc
        src/mpc/optimization/mopso.cc
        src/mpc/mpc.cc
        src/util.cc

)

set(CONTROL_MODE
        src/mpc/control_mode/control_mode.h
        src/mpc/control_mode/angle.h
        src/mpc/control_mode/position.h
        src/mpc/control_mode/acro.h
        src/mpc/control_mode/waypoint.h
        src/mpc/control_mode/loiter.h
        src/mpc/control_mode/alt_hold.h
        src/mpc/control_mode/cruise.h
        src/mpc/control_mode/idle.h

        src/mpc/control_mode/control_mode.cc
        src/mpc/control_mode/angle.cc
        src/mpc/control_mode/position.cc
        src/mpc/control_mode/acro.cc
        src/mpc/control_mode/waypoint.cc
        src/mpc/control_mode/loiter.cc
        src/mpc/control_mode/alt_hold.cc
        src/mpc/control_mode/cruise.cc
        src/mpc/control_mode/idle.cc
)

set(AIRCRAFT
        src/aircraft/aircraft.h
        src/aircraft/aircraft.cc
        src/aircraft/drone.h
        src/aircraft/drone.cc
)

set(GUIDANCE
        src/guidance/guidance.h
        src/navigation/navigation.h

        src/guidance/guidance.cc
        src/navigation/navigation.cc
)

set(RUN_CONFIG
        src/main.cc
        ${MAIN}
        ${AIRCRAFT}
        ${CONTROL_MODE}
        ${GUIDANCE}
        ${MPC}
)

add_executable(RUN  ${RUN_CONFIG})