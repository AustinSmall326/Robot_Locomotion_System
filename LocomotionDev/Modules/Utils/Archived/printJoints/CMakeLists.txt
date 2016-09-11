cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
project(printJoints)
find_package(qibuild)

set(_srcs
    main.cpp
    printJoints.h
    printJoints.cpp
)

qi_create_lib(printJoints SHARED ${_srcs} SUBFOLDER naoqi)

qi_use_lib(printJoints ALCOMMON ALMEMORYFASTACCESS)