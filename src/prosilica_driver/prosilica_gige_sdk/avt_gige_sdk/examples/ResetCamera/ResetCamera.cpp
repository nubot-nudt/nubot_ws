/*
  ==============================================================================
  Copyright (C) 2006-2014 Allied Vision Technologies.  All Rights Reserved.
 
  This code may be used in part, or in whole for your application development.
 
 ==============================================================================
 
  ResetCamera
 
  Resets a camera specified by its IP address, using direct register write
  Camera IP address can be found with ListCameras example code, or DeviceIPAddress 
  attribute in SampleViewer.
 
      e.g. usage: ResetCamera 169.254.23.10
 
 ==============================================================================
 
  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 ==============================================================================
*/

#ifdef _WINDOWS
#include "StdAfx.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WINDOWS
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <Winsock2.h>
#endif

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#endif

#include <PvApi.h>
#include <PvRegIo.h>

#ifndef _WINDOWS
#define TRUE 0
#endif


// reset the camera
void CameraReset(tPvHandle Handle)
{
  tPvErr errCode;
  
  unsigned long Address = 0x10008;  // register @
  unsigned long Value   = 2;        // hard-reset value
  
  errCode = PvRegisterWrite(Handle,1,&Address,&Value,NULL);
  //in this case PvRegisterWrite should return with ePvErrUnplugged
  if(errCode == ePvErrUnplugged)
	  printf("Camera reset.\n");
  else
	  printf("PvRegisterWrite err: %u\n", errCode);
}


int main(int argc, char* argv[])
{
    tPvErr errCode;
	
	// initialize PvAPI
    errCode = PvInitialize();
	if(errCode != ePvErrSuccess)
    { 
		printf("PvInitialize err: %u\n", errCode);
	}
	else
	{
        // the only command line argument accepted is the IP@ of the camera to be open
        if(argc>1)
        {
            //windows call to convert user input to IP address
			unsigned long IP = inet_addr(argv[1]);
             
            if((IP == INADDR_NONE) || (IP == INADDR_ANY))
			{
				printf("A valid IP address must be entered\n");
			}
			else
			{           
                tPvHandle Handle; 
                
                // open the camera by IP
				errCode = PvCameraOpenByAddr(IP,ePvAccessMaster,&Handle);
				if(errCode != ePvErrSuccess)
				{ 
					//if err 7, ePvErrAccessDenied, camera already open as Master in other app
					//or camera wasn't properly closed, and still waiting to HeartbeatTimeout.
					printf("PvCameraOpenByAddr err: %u\n", errCode);
				}
				else
				{
                    // send reset command
                    CameraReset(Handle);
                    
					// close the camera
                    PvCameraClose(Handle);
                }
            }         
        }
        else
            printf("usage : ResetCamera <IP@>\n");

        // uninitialize the API
        PvUnInitialize();
	}
    
	return 0;
}
