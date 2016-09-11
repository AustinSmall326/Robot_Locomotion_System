#ifndef ROBOT_COMM_TEST_H
#define ROBOT_COMM_TEST_H

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include <qi/application.hpp>
#include <qi/eventloop.hpp>

#include <alerror/alerror.h>

namespace AL
{
    class ALBroker;
}

class RobotCommTest : public AL::ALModule
{
    public:
        // Methods.
        RobotCommTest(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName);
        virtual ~RobotCommTest();

    private:
        AL::ALProxy* proxy;
};

#endif