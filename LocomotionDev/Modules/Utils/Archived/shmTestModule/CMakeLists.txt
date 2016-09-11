cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
project(shm)
find_package(qibuild)

set(_srcs
    main.cpp
    shmTestModule.h
    shmTestModule.cpp
)

qi_create_lib(shmTestModule SHARED ${_srcs} SUBFOLDER naoqi)

qi_use_lib(shmTestModule ALCOMMON)