/** @file    naoStatus.h
 *  @brief   Header file to naoStatus class, which allows for communicating sensor values to computer.
 *  @author  Austin Small.
 */

#ifndef NAO_STATUS_H
#define NAO_STATUS_H

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

namespace AL
{
    class ALBroker;
}

/** @brief		naoStatus class allows for communicating sensor values to computer.
 */
class naoStatus : public AL::ALModule
{
    public:
        // Methods.
        naoStatus(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName);
        virtual ~naoStatus();
    
        std::string communicate(std::string);
    
    private:
};

#endif