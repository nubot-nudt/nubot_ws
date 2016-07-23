/*
  ==============================================================================
  Copyright (C) 2013 Allied Vision Technologies.  All Rights Reserved.
 
  This code may be used in part, or in whole for your application development.
 
 ==============================================================================
 
  ForceCamera
 
  Force the IP setup of a camera specified by its MAC address
 
      e.g. usage: ForceCamera 00:0f:31:01:8f:27 169.254.6.66 255.255.0.0 0.0.0.0
 
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

int main(int argc, char* argv[])
{
    tPvErr errCode;
	
	// initialize PvAPI
    errCode = PvInitializeNoDiscovery();
	if(errCode != ePvErrSuccess)
    { 
		printf("PvInitializeNoDiscovery err: %u\n", errCode);
	}
	else
	{
        // the only command line argument accepted is the IP@ of the camera to be open
        if(argc>=5)
            PvCameraForceIP(argv[1],inet_addr(argv[2]),inet_addr(argv[3]),inet_addr(argv[4]));
        else
            printf("usage : ForceCamera <MAC> <IP@> <subnet> <gateway>\n");

        // uninitialize the API
        PvUnInitialize();
	}
    
	return 0;
}
