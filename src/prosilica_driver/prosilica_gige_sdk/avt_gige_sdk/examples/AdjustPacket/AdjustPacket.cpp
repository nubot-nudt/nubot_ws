/*
  ==============================================================================
  Copyright (C) 2006-2014 Allied Vision Technologies.  All Rights Reserved.
 
  This code may be used in part, or in whole for your application development.
 
 ==============================================================================
 
  AdjustPacket
 
  Opens camera by IP address. Calls PvCaptureAdjustPacketSize -- recommended for
  minimizing dropped packets. This call sets camera PacketSize to largest sized
  test packet, up to MAXSIZE, that doesn't fail on network card. See AVT GigE 
  Installation Guide for optimizing network hardware.
 
  NOTE: Some MS VISTA network card drivers become unresponsive if test packet fails.
        Use PvUint32Set(handle, "PacketSize", MaxAllowablePacketSize) instead. 
        See network card properties for max allowable PacketSize/MTU/JumboFrameSize.
 					
 		e.g. usage: AdjustPacket 169.254.24.25
 
  Camera IP address can be found with ListCameras example code, 
  or DeviceIPAddress attribute in SampleViewer.
 
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

#ifndef _WINDOWS
#define TRUE 0
#endif

#define MAXSIZE 16000

int main(int argc, char* argv[])
{
	tPvErr errCode;

	// initialize the PvAPI
	if((errCode = PvInitialize()) != ePvErrSuccess)
    { 
		printf("PvInitialize err: %u\n", errCode);
	}
	else
    { 
        // the only command line argument accepted is the IP@ of the camera to be open
        if(argc>1)
        {
            unsigned long IP = inet_addr(argv[1]);
             
            if((IP == INADDR_NONE) || (IP == INADDR_ANY))
			{
				printf("A valid IP address must be entered\n");
			}
			else
            {           
                tPvHandle Handle; 
                
                // open the camera
                if((errCode = PvCameraOpenByAddr(IP,ePvAccessMaster,&Handle)) == ePvErrSuccess)
                {
					
	
					// NOTE: This call sets camera PacketSize to largest sized test packet, up to MaxSize, that doesn't fail
					// on network card. Some MS VISTA network card drivers become unresponsive if test packet fails. 
					// Use PvUint32Set(handle, "PacketSize", MaxAllowablePacketSize) instead. See network card properties
					// for max allowable PacketSize/MTU/JumboFrameSize. 
					if((errCode = PvCaptureAdjustPacketSize(Handle,MAXSIZE)) != ePvErrSuccess)
					{
						printf("PvCaptureAdjustPacketSize err: %u\n", errCode);
					}
					else
					{
						unsigned long Size;

						if((errCode = PvAttrUint32Get(Handle,"PacketSize",&Size)) != ePvErrSuccess)  
							printf("Attribute get error: %u\n", errCode);
						else
							printf("Packet size set to %lu bytes\n",Size);
					}

					
					// close the camera
                    PvCameraClose(Handle);


                }
                else if (errCode == ePvErrAccessDenied)
					printf("PvCameraOpenByAddr: ePvErrAccessDenied:\nCamera already open, or camera wasn't properly closed and waiting to HeartbeatTimeout.");
				else
					printf("PvCameraOpenByAddr err: %u\n", errCode);
			}
			
        }
        else
            printf("usage : AdjustPacket <IP@>\n");

        // uninitialise the API
        PvUnInitialize();
    }

	return 0;
}
