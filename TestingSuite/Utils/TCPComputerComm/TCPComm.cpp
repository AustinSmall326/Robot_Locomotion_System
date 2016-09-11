/** @file    TCPComm.cpp
 *  @brief   cpp file to TCPComm class, which is a MEX file to send and receive TCP messages.
 *  @author  Austin Small.
 */

#include "TCPHeartbeatWrapper.h"

#include "mex.h"
#include <string.h>

#define PORT 12500
#define MAX_TRANSMISSION_LENGTH 300000 // Max length of a sent/received message.
#define MAX_PACKET_LENGTH 1400         // Max length of an individual packet.  Ethernet
                                       // limitation is 1500 bytes per packet.

enum failCause { TIMEOUT, ERROR, COLLISSION };

TCPHeartbeatWrapper* hbWrapper;

bool init = false;

std::string IP("");

/** @brief   Function to be called when MEX-function is cleared.
 *
 */

void mexExit(void)
{
    //std::cout << "deleting wrapper" <<std::endl;
    delete hbWrapper;
    hbWrapper = NULL;
    //std::cout << "deleted wrapper" << std::endl;
}

/** @brief   Function to handle errors.
 *
 *  @param   msg        Message to print to terminal.
 *  @param   *plhs[]    Pointer to MEX output array.
 *  @param   cause      Cause for failure.
 *
 */

void returnFail(std::string msg, mxArray *plhs[], failCause cause)
{
    // Effectively close the MEX module.
    if (cause == ERROR)
    {
        init = false;
        mexExit();
    }
    
    mexPrintf(msg.c_str());
    
    // Return an empty string.
    std::string emptyString("");
    plhs[0] = mxCreateString(emptyString.c_str());
}

/** @brief   This function provides an entry point for Matlab into this cpp file.
 *
 *  @param   nlhs   Number of output variables.
 *  @param   plhs   Array of mxArray pointers to the output variables.
 *  @param   nrhs   Number of input variables.
 *  @param   prhs   Array of mxArray pointers to the input variables.
 *
 */

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (init == false)
    {
        // Register mexExit() to run when MEX-function is cleared.
        mexAtExit(mexExit);
        
        // Store robot IP address.
        IP.append(mxArrayToString(prhs[1]));
        
        //std::cout << "instantiating new wrapper" << std::endl;
        
        // Make sure hbWrapper is null.
        delete hbWrapper;
        hbWrapper = NULL;
        
        // Instantiate heartbeat wrapper.
        hbWrapper = new TCPHeartbeatWrapper(PORT, MAX_PACKET_LENGTH);
        
        if (hbWrapper->connect(IP) < 0)
        {
            returnFail("Failed to connect to server.", plhs, ERROR);
            return;
        }
        
        init = true;
    }
    
    if (!hbWrapper->isActive())
    {
        if (hbWrapper->connect(IP) < 0)
        {
            returnFail("Failed to connect to server.", plhs, ERROR);
            return;
        }
    }
    
    char returnData[MAX_TRANSMISSION_LENGTH];
    
    // Check if an error occurred.
    int result = hbWrapper->communicate(mxArrayToString(prhs[0]), returnData);
    
    if (result < 0)
    {
        if (result == -1)
        {
            returnFail("Failed to send message due to an error.", plhs, ERROR);
            return;
        }
        else if (result == -2)
        {
            returnFail("Failed to send message due to collission.", plhs, COLLISSION);
            return;
        }
    }
    

    // Return message.
    plhs[0] = mxCreateString(returnData);
}