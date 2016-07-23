/*
  ==============================================================================
  Copyright (C) 2006-2011 Allied Vision Technologies.  All Rights Reserved.
 
  This code may be used in part, or in whole for your application development.
 
 ==============================================================================
 
  CamMemory
 
  Prosilica series cameras have 512 Bytes of non-volatile memory:
    MemoryUserDefined (17200h) 
 
  This sample code reads and writes to the user-defined memory using register
  access.
  
      e.g. usage: CamMemory -i 169.254.23.10 -s "factory camera 14"
      e.g. usage: CamMemory -i 169.254.23.10 -g
 
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
#include <ctype.h>

#ifdef _WINDOWS
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <Winsock2.h>
#include "XGetopt.h"
#endif

#ifndef _WINDOWS
#define strncpy_s(dest,len,src,count) strncpy(dest,src,count)
#define sprintf_s(dest,len,format,args...) sprintf(dest,format,args)
#define sscanf_s sscanf
#define strcpy_s(dst,len,src) strcpy(dst,src)
#define _strdup(src) strdup(src)
#define strtok_s(tok,del,ctx) strtok(tok,del)
#endif

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <arpa/inet.h>
#include <strings.h>
#endif

#include <PvApi.h>
#include <PvRegIo.h>  //register access

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
// put the calling thread to sleep for a given amount of millisecond
void Sleep(unsigned int time)
{
    struct timespec t,r;
    
    t.tv_sec    = time / 1000;
    t.tv_nsec   = (time % 1000) * 1000000;    
    
    while(nanosleep(&t,&r)==-1)
        t = r;
}
#endif

// display the usage details
void ShowUsage()
{
    printf("usage: CamMemory -u <camera unique ID>| -i <camera IP> -g|-s [<string>]\n");
    printf("-u\tcamera unique ID\n");
    printf("-i\tcamera IP address\n");
    printf("-g\tget string\n");
    printf("-s\tset string\n");
}

// Read and print MemoryUserDefined
tPvErr MemRead(tPvHandle aCamera)
{
    tPvErr lErr;
    unsigned char lBuffer[512];
    
    lErr = PvMemoryRead(aCamera,0x17200,512,lBuffer);
    
    if(lErr == ePvErrSuccess)
    {
        printf("value = '%s'\n",lBuffer);
        
        /*print as hex.
		for(int i=0;i<512;i++)
        {
            printf("0x%02X ",lBuffer[i]);
            if(!((i + 1) % 16))
                printf("\n");       
        }*/
        
        printf("\n");
    }
    
    return lErr;   
}

// Write the given string to MemoryUserDefined
tPvErr MemWrite(tPvHandle aCamera,const char* aString)
{ 
    if(strlen(aString) < 512)
    {
        unsigned char lBuffer[512];
        unsigned long Size = 0;
    
        memset(lBuffer,0,512);
    
        strcpy_s((char*)lBuffer, 512, aString);
    
        return PvMemoryWrite(aCamera,0x17200,512,lBuffer,&Size); 
    }
    else
	{
		printf("String length > 512\n");
		return ePvErrBadParameter;
	}
}


int main(int argc, char* argv[])
{
    tPvErr errCode;
	int err = 0;

	// initialize the PvAPI
	if((errCode = PvInitialize()) != ePvErrSuccess)
    { 
		printf("PvInitialize err: %u\n", errCode);
	}
	else
    {
        int c;
        unsigned long uid = 0;
        unsigned long addr = 0;
        bool bGet = false;
        bool bSet = false;
    
        while ((c = getopt (argc, argv, "u:i:gs:h?")) != -1)
        {
            switch(c)
            {
                case 'u':
                {
                    if(optarg)
                        uid = atol(optarg);
                    
                    break;    
                }
                case 'i':
                {
                    if(optarg)
                        addr = inet_addr(optarg);
                    
                    break;    
                }                
                case 'g':
                {
                    bGet = true;
                    break;
                }
                case 's':
                {
                    bSet = true;
                    break;
                }
                case '?':
                case 'h':
                {
                    ShowUsage();
                    break;    
                }
                default:
                    break;
            }
        }

		if(uid || ((addr != INADDR_NONE) && (addr != INADDR_ANY)))
        {
            tPvHandle       Camera;
            tPvAccessFlags  Flags = (bSet ? ePvAccessMaster : ePvAccessMonitor);
            
            if(uid)
            {
                // wait a bit to leave some time to the API to detect any camera
                Sleep(500);
                // and open the camera
                errCode = PvCameraOpen(uid,Flags,&Camera);
            }
            else
                errCode = PvCameraOpenByAddr(addr,Flags,&Camera);
                
            if(errCode == ePvErrSuccess)
            {   
                if(bGet) // get value
                    errCode = MemRead(Camera);
                else
                if(bSet) // set value
                    errCode = MemWrite(Camera,argv[argc-1]);          

                if(errCode != ePvErrSuccess)
					fprintf(stderr,"Error: %u\n",errCode);

                err = 1;

                // close the camera
                PvCameraClose(Camera);
            }
            else
            {
                if(errCode == ePvErrNotFound || errCode == ePvErrUnplugged)
                    fprintf(stderr,"No camera detected.\n");
                else
                if(errCode == ePvErrAccessDenied)
                    fprintf(stderr,"Camera already in use.\n");
                else
					fprintf(stderr,"PvCameraOpen fail: %u\n", errCode);

                err = 1;    
            }    
        }
        else
        {
            ShowUsage();
            err = 1;  
        }

        PvUnInitialize();
    }
    
	return err;
}

