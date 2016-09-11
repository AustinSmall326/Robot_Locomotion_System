/** @file    keyframeGen.h
 *  @brief   Header file to keyframeGen class, which allows for easy creation of keyframes.
 *  @author  Austin Small.
 */

#ifndef KEYFRAME_GEN_H
#define KEYFRAME_GEN_H

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <vector>

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include "../../LocomotionDefines.h"
#include "../Iterp/Iterp.h"

namespace AL
{
    class ALBroker;
}

/** @brief		keyframeGen class allows for easy creation of keyframes.
 */
class keyframeGen : public AL::ALModule
{
    public:
        // Methods.
        keyframeGen(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName);
        virtual ~keyframeGen();
    
        std::string communicate(std::string);
    
    private:
        // Methods.
        void interpolationLinearThreadFunc(void);

        // Fields.
        AL::ALProxy* proxy;
        
        std::vector <std::vector <float> > positionQueue;
        std::vector <std::vector <float> > stiffnessQueue;
        std::vector <double> duration;
        std::vector <std::string> interpolationType;
    
        bool threadActive;
        boost::thread* interpolationThread;
};

#endif