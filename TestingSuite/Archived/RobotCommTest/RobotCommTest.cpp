#include "RobotCommTest.h"

RobotCommTest::RobotCommTest(boost::shared_ptr<AL::ALBroker> broker, const std::string &name) :
    AL::ALModule(broker, name)
{
    setModuleDescription("Test module for RobotComm.");
    
    proxy = new AL::ALProxy(broker, "TCPRobotComm", 0, 0);
    
    proxy->call<void>("spawnInfiniteBroadcast");
}

RobotCommTest::~RobotCommTest()
{ }

