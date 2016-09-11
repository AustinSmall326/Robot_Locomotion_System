#ifndef SHM_TEST_MODULE_H
#define SHM_TEST_MODULE_H

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include <alerror/alerror.h>

// Output stream libraries included for testing only.
#include <iostream>
#include <fstream>

namespace AL
{
    class ALBroker;
}

class shmTestModule : public AL::ALModule
{
    public:
        // Methods.
        shmTestModule(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName);
        virtual ~shmTestModule();
    
    private:
        // Fields.
        AL::ALProxy* proxy;
    
        // Array to store a mapping from array indices to actuator names.
        std::string indexToStringMap[24];
};

#endif