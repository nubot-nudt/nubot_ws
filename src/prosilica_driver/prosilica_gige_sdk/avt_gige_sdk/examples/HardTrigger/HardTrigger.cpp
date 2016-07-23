/*
  ==============================================================================
  Copyright (C) 2006-2011 Allied Vision Technologies.  All Rights Reserved.
 
  This code may be used in part, or in whole for your application development.
 
 ==============================================================================
 
  HardTrigger
 
  Similar to SoftTrigger, except instead of:
    PvAttrEnumSet(Camera->Handle,"FrameStartTriggerMode","Software")
  this code sets:
    PvAttrEnumSet(GCamera.Handle,"FrameStartTriggerMode","SyncIn1")
    and waits for a trigger input on SyncIn1 instead of a FrameStartTriggerSoftware
    command.
 
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
#include <string.h>

#ifdef _WINDOWS
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
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
#endif

#include <PvApi.h>
#include <ImageLib.h>

#ifdef _WINDOWS
#define _STDCALL __stdcall
#else
#define _STDCALL
#define TRUE     0
#endif

// camera's data
typedef struct 
{
    unsigned long   UID;
    tPvHandle       Handle;
    tPvFrame        Frame;
    tPvUint32       Counter;
    char            Filename[20];
	bool            Abort;

} tCamera;

// global camera data
tCamera GCamera;

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
void Sleep(unsigned int time)
{
    struct timespec t,r;
    
    t.tv_sec    = time / 1000;
    t.tv_nsec   = (time % 1000) * 1000000;    
    
    while(nanosleep(&t,&r)==-1)
        t = r;
}

//Define function equivalent to Windows SetConsoleCtrlHandler 
void SetConsoleCtrlHandler(void (*func)(int), int junk)
{
    signal(SIGINT, func);
}
#endif

// CTRL+C handler
#ifdef _WINDOWS
BOOL WINAPI CtrlCHandler(DWORD dwCtrlType)
#else
void CtrlCHandler(int Signo)
#endif  
{
    printf("\nCtrl+C interrupt received, stopping camera.\n");    
    
	//Set flag to exit WaitForEver
    GCamera.Abort = true;    
        
    #ifndef _WINDOWS
    signal(SIGINT, CtrlCHandler);
    #else
    return true;
    #endif
}

// wait for camera to be plugged in
void WaitForCamera()
{
    printf("Waiting for a camera ");
    while((PvCameraCount() == 0) && !GCamera.Abort)
    {
        printf(".");
        Sleep(250);
    }
    printf("\n");
}

// get the first camera found
// return value: true == success, false == fail
bool CameraGet()
{
    tPvUint32 count,connected;
    tPvCameraInfoEx list;

    //regardless if connected > 1, we only set UID of first camera in list
	count = PvCameraListEx(&list,1,&connected, sizeof(tPvCameraInfoEx));
    if(count == 1)
    {
        GCamera.UID = list.UniqueId;
        printf("Got camera %s\n",list.SerialNumber);
        return true;
    }
    else
	{
		printf("CameraGet: Failed to find a camera\n");
		return false;
	}
}

// open camera, allocate memory
// return value: true == success, false == fail
bool CameraSetup()
{
    tPvErr errCode;
	unsigned long FrameSize = 0;

	//open camera
	if ((errCode = PvCameraOpen(GCamera.UID,ePvAccessMaster,&(GCamera.Handle))) != ePvErrSuccess)
	{
		if (errCode == ePvErrAccessDenied)
			printf("PvCameraOpen returned ePvErrAccessDenied:\nCamera already open as Master, or camera wasn't properly closed and still waiting to HeartbeatTimeout.");
		else
			printf("PvCameraOpen err: %u\n", errCode);
		return false;
	}

	// Calculate frame buffer size
    if((errCode = PvAttrUint32Get(GCamera.Handle,"TotalBytesPerFrame",&FrameSize)) != ePvErrSuccess)
	{
		printf("CameraSetup: Get TotalBytesPerFrame err: %u\n", errCode);
		return false;
	}

    // allocate image buffer
    GCamera.Frame.ImageBuffer = new char[FrameSize];
    if(!GCamera.Frame.ImageBuffer)
	{
		printf("CameraSetup: Failed to allocate buffers.\n");
		return false;
	}
	GCamera.Frame.ImageBufferSize = FrameSize;

	return true;
}

// close camera, free memory.
void CameraUnsetup()
{
    tPvErr errCode;
	
	// always close an opened camera even if it was unplugged before
    if((errCode = PvCameraClose(GCamera.Handle)) != ePvErrSuccess)
	{
		printf("CameraUnSetup: PvCameraClose err: %u\n", errCode);
	}
	else
	{
		printf("Camera closed.");
	}
	
	// free image buffer
    delete [] (char*)GCamera.Frame.ImageBuffer;
}

// setup and start streaming
// return value: true == success, false == fail
bool CameraStart()
{
    tPvErr errCode;

    // NOTE: This call sets camera PacketSize to largest sized test packet, up to 8228, that doesn't fail
	// on network card. Some MS VISTA network card drivers become unresponsive if test packet fails. 
	// Use PvUint32Set(handle, "PacketSize", MaxAllowablePacketSize) instead. See network card properties
	// for max allowable PacketSize/MTU/JumboFrameSize. 
	if((errCode = PvCaptureAdjustPacketSize(GCamera.Handle,8228)) != ePvErrSuccess)
	{
		printf("CameraStart: PvCaptureAdjustPacketSize err: %u\n", errCode);
		return false;
	}

    // start driver capture stream 
	if((errCode = PvCaptureStart(GCamera.Handle)) != ePvErrSuccess)
	{
		printf("CameraStart: PvCaptureStart err: %u\n", errCode);
		return false;
	}
	
    // queue frame
	if((errCode = PvCaptureQueueFrame(GCamera.Handle,&(GCamera.Frame),NULL)) != ePvErrSuccess)
	{
		printf("CameraStart: PvCaptureQueueFrame err: %u\n", errCode);
		// stop driver capture stream
		PvCaptureEnd(GCamera.Handle);
		return false;
	}
		
	// set the camera in hardware trigger, continuous mode, and start camera receiving triggers
	if((PvAttrEnumSet(GCamera.Handle,"FrameStartTriggerMode","SyncIn1") != ePvErrSuccess) ||
		(PvAttrEnumSet(GCamera.Handle,"AcquisitionMode","Continuous") != ePvErrSuccess) ||
		(PvCommandRun(GCamera.Handle,"AcquisitionStart") != ePvErrSuccess))
	{		
		printf("CameraStart: failed to set camera attributes\n");
		// clear queued frame
		PvCaptureQueueClear(GCamera.Handle);
		// stop driver capture stream
		PvCaptureEnd(GCamera.Handle);
		return false;
	}	

	return true;
}

// stop streaming
void CameraStop()
{
    tPvErr errCode;
	
	//stop camera receiving triggers
	if ((errCode = PvCommandRun(GCamera.Handle,"AcquisitionStop")) != ePvErrSuccess)
		printf("AcquisitionStop command err: %u\n", errCode);
	else
		printf("Camera stopped.\n");

    //clear queued frames. will block until all frames dequeued
	if ((errCode = PvCaptureQueueClear(GCamera.Handle)) != ePvErrSuccess)
		printf("PvCaptureQueueClear err: %u\n", errCode);
	else
		printf("Queue cleared.\n");  

	//stop driver stream
	if ((errCode = PvCaptureEnd(GCamera.Handle)) != ePvErrSuccess)
		printf("PvCaptureEnd err: %u\n", errCode);
	else
		printf("Driver stream stopped.\n");
}

// trigger and save a frame from the camera
// return value: true == success, false == fail
bool CameraSnap()
{
    tPvErr errCode = ePvErrSuccess;

    //wait for frame to return from camera to host
    printf("Waiting for SyncIn1 trigger input");
	while(!GCamera.Abort && ((errCode = PvCaptureWaitForFrameDone(GCamera.Handle,&(GCamera.Frame),800)) == ePvErrTimeout))
        printf(".");
	printf("\n");

	if(errCode != ePvErrSuccess  || GCamera.Abort)
    { 
		//likely camera unplugged
		GCamera.Abort = true;
		return false;
	}
	
	//check returned Frame.Status
	if(GCamera.Frame.Status == ePvErrSuccess)
    {
		sprintf_s(GCamera.Filename,sizeof(GCamera.Filename),"./snap%04lu.tiff",++GCamera.Counter);
		//save image
		if(!ImageWriteTiff(GCamera.Filename,&(GCamera.Frame)))
		{
			printf("ImageWriteTiff fail.\n");
			GCamera.Abort = true;
			return false;
		}
		printf("snap%04lu.tiff saved.\n", GCamera.Counter);
	}
    else
	{
		if (GCamera.Frame.Status == ePvErrDataMissing)
			printf("Dropped packets. Possible improper network card settings:\nSee GigE Installation Guide.");
		else
			printf("Frame.Status error: %u\n",GCamera.Frame.Status);
	}

	//requeue frame	
	if((errCode = PvCaptureQueueFrame(GCamera.Handle,&(GCamera.Frame),NULL)) != ePvErrSuccess)
	{
        printf("PvCaptureQueueFrame err: %u\n", errCode);
		GCamera.Abort = true;
		return false;
	}
	return true;
}

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

        //IMPORTANT: Initialize camera structure. See tPvFrame in PvApi.h for more info.
		memset(&GCamera,0,sizeof(tCamera));

		//Set function to handle ctrl+C
		SetConsoleCtrlHandler(CtrlCHandler, TRUE);
		
		printf("Press CTRL-C to terminate\n");

        // wait for a camera to be plugged in
        WaitForCamera();

        // get first camera found
        if(CameraGet())
        {
            // open camera
            if(CameraSetup())
            {
                // start camera streaming
                if(CameraStart())
                {
					
                   while(GCamera.Abort == false)
                        CameraSnap();
					
                    // stop camera streaming
                    CameraStop();
                }
                
                // close camera
                CameraUnsetup();
            }
        }        
       
        // uninitialize PvAPI
        PvUnInitialize();
    }
    
	return 0;
}
