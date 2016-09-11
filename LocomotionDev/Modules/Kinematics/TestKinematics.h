/** @file    TestKinematics.h
 *  @brief   Header file to Kinematics library.
 *  @author  Austin Small.
 */

#ifndef TestKinematics_h_DEFINED
#define TestKinematics_h_DEFINED

#include "KinematicsWrapper.h"
#include "../Representations/Transform.h"

#include <boost/lexical_cast.hpp>

#include <vector>
#include <iostream>
#include <math.h>

/** @brief		TestKinematics class stores tests for the NAO Kinematics library.
 */
class TestKinematics
{
    public:
        static std::string runTests(void);
};

#endif