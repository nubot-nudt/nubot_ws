/*
  ==============================================================================
  Copyright (C) 2006-2014 Allied Vision Technologies.  All Rights Reserved.
 
  This code may be used in part, or in whole for your application development.
 
 ==============================================================================
 
  BatchStream
 
  This example shows how to manage FRAMESCOUNT number of frames, which
  may be larger than the QUEUELIMIT frame limit. 
 
  QUEUELIMIT limit on PvAPI < 1.26 is 100. No QUEUELIMIT for PvAPI >= 1.26.
 
  -Opens the first camera found. 
  -Sets camera to freerun continuous mode. 
  -Queues QUEUELIMIT or FRAMESCOUNT frames, whichever is less. 
  -Frames queued with callbacks. Callbacks function returns Frame.Status.
  -Main thread call PvCaptureWaitForFrameDone on the 2nd last queued frame.
    When returned, queue the next QUEUELIMIT frames (or portion thereof).
  -Ctrl+C to abort.

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
#endif

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
#include <unistd.h>
#include <time.h>
#include <signal.h>
#endif

#include <PvApi.h>

#ifdef _WINDOWS
#define _STDCALL __stdcall
#else
#define _STDCALL
#define TRUE     0
#endif

#define FRAMESCOUNT 490
#define QUEUELIMIT 99

typedef struct 
{
    unsigned long   UID;
    tPvHandle       Handle;
    tPvFrame        Frames[FRAMESCOUNT];
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

// wait for a camera to be plugged in
void WaitForCamera()
{
    printf("Waiting for a camera");
    while((PvCameraCount() == 0) && !GCamera.Abort)
	{
		printf(".");
		Sleep(250);
	}
    printf("\n");
}

// CTRL+C handler
#ifdef _WINDOWS
BOOL WINAPI CtrlCHandler(DWORD dwCtrlType)
#else
void CtrlCHandler(int Signo)
#endif  
{
    printf("Ctrl+C interrupt received, stopping camera.\n");    
    
	//Set flag to exit WaitForEver
    GCamera.Abort = true;    
        
    #ifndef _WINDOWS
    signal(SIGINT, CtrlCHandler);
    #else
    return true;
    #endif
}

// Frame completed callback executes on seperate driver thread.
// One callback thread per camera. If a frame callback function has not 
// completed, and the next frame returns, the next frame's callback function is queued. 
// This situation is best avoided (camera running faster than host can process frames). 
// Spend as little time in this thread as possible and offload processing
// to other threads or save processing until later.
//
// Note: If a camera is unplugged, this callback will not get called until PvCaptureQueueClear.
// i.e. callback with pFrame->Status = ePvErrUnplugged doesn't happen -- so don't rely
// on this as a test for a missing camera. 
void _STDCALL FrameDoneCB(tPvFrame* pFrame)
{
	//report if any errors in returned frames
	switch (pFrame->Status) {
		case ePvErrSuccess:
			break;
		case ePvErrDataMissing:
			//Possible improper network card settings. See GigE Installation Guide.
			printf("FrameCallback [%lu]: Dropped packets.   \n", pFrame->FrameCount);
			break;
		case ePvErrResources:
			printf("FrameCallback [%lu]: Out of host memory!\n", pFrame->FrameCount);
			break;
		case ePvErrCancelled:
			printf("FrameCallback [%lu]: Frame cancelled.   \n", pFrame->FrameCount);
			break;
		case ePvErrInternalFault:
			printf("FrameCallback [%lu]: Internal fault. PvAPI.dll 1.24 w > 100 frames queued?\n", pFrame->FrameCount);
			break;
		default:
			printf("FrameCallback [%lu]: Error: %u          \n", pFrame->FrameCount, pFrame->Status);
			break;
	}

	//NOTE: No frame requeuing here. Done in main thread after PvCaptureWaitForFrameDone
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
	bool failed = false;
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

	// Allocating FRAMESCOUNT number of full sized images can be a LOT of memory!
	// I.e. more than your host has. Set camera image size to 100 x 100 (or less)
	if((PvAttrUint32Set(GCamera.Handle,"Width",100) != ePvErrSuccess) ||
		(PvAttrUint32Set(GCamera.Handle,"Height",100) != ePvErrSuccess))
	{		
		printf("Failed to set camera width x height\n");
		return false;
	}	

	// Calculate frame buffer size
    if((errCode = PvAttrUint32Get(GCamera.Handle,"TotalBytesPerFrame",&FrameSize)) != ePvErrSuccess)
	{
		printf("CameraSetup: Get TotalBytesPerFrame err: %u\n", errCode);
		return false;
	}

	// allocate the frame buffers
    printf("Allocating memory...\n");
	for(int i=0;i<FRAMESCOUNT && !failed;i++)
    {
        GCamera.Frames[i].ImageBuffer = new char[FrameSize];
        if(GCamera.Frames[i].ImageBuffer)
        {
			GCamera.Frames[i].ImageBufferSize = FrameSize;
		}
        else
		{
			printf("Failed to allocate buffers.\n");
			failed = true;
		}
    }

	if (!failed)
		printf("memory allocated\n");

	return !failed;
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
		printf("Camera closed.\n");
	}
	// delete image buffers
    for(int i=0;i<FRAMESCOUNT;i++)
        delete [] (char*)GCamera.Frames[i].ImageBuffer;

    GCamera.Handle = NULL;
}

// setup and start streaming
// return value: true == success, false == fail
bool CameraStart()
{
    int i = 0;
    int j = 0;
	tPvErr errCode;
	bool failed = false;

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

	// set the camera in freerun trigger, continuous mode, and start camera receiving triggers
	if((PvAttrEnumSet(GCamera.Handle,"FrameStartTriggerMode","Freerun") != ePvErrSuccess) ||
		(PvAttrEnumSet(GCamera.Handle,"AcquisitionMode","Continuous") != ePvErrSuccess) ||
		(PvCommandRun(GCamera.Handle,"AcquisitionStart") != ePvErrSuccess))
	{		
		printf("CameraStart: failed to set camera attributes\n");
		return false;
	}	

    // loop until failure or we have acquired all FRAMESCOUNT frames
	while(j<FRAMESCOUNT && !failed  && !GCamera.Abort)
    {
        int last;
			
		if ((j + QUEUELIMIT) < FRAMESCOUNT)
			last = j+QUEUELIMIT - 1;
		else
			last = FRAMESCOUNT - 1;
		
		printf("Queueing frames [%u - %u]\n", j, last);
		
		// enqueue QUEUELIMIT frames at a time.
        for(i=0;i<QUEUELIMIT && j<FRAMESCOUNT && !failed;i++,j++)
		{	
			if((errCode = PvCaptureQueueFrame(GCamera.Handle,&(GCamera.Frames[j]),FrameDoneCB)) != ePvErrSuccess)
			{
				printf("CameraStart: PvCaptureQueueFrame err: %u\n", errCode);
				failed = true;
			}
		}

        //Wait for the 2nd last enqueued frame to return to host before queuing the next batch
		//Always leave at least one frame on queue when requeuing, otherwise camera could return image 
		//with no frame queued, resulting in a dropped image.
		if (!failed)
		{
			printf("Waiting for frame [%u]", j-2);
			while((errCode = PvCaptureWaitForFrameDone(GCamera.Handle,&(GCamera.Frames[j-2]),2000)) == ePvErrTimeout)
				printf(".");
			printf("\n");
		}

		if(errCode == ePvErrSuccess)
        { 
			printf("Frame [%u] returned.\n", j-2);
		}
		else
		{
			//likely camera unplugged
			printf("PvCaptureWaitForFrameDone err: %u\n", errCode);
			failed = true;
		} 			
    }
	return !failed;
}


// stop streaming
void CameraStop()
{
    tPvErr errCode;
	
	//stop camera receiving triggers
	if ((errCode = PvCommandRun(GCamera.Handle,"AcquisitionStop")) != ePvErrSuccess)
		printf("AcquisitionStop command err: %u\n", errCode);
	else
		printf("AcquisitionStop success.\n");
    
	//PvCaptureQueueClear aborts any actively written frame with Frame.Status = ePvErrDataMissing
	//Further queued frames returned with Frame.Status = ePvErrCancelled
	
	//Add delay between AcquisitionStop and PvCaptureQueueClear
	//to give actively written frame time to complete
	Sleep(200);
	
	printf("Calling PvCaptureQueueClear...\n");
	if ((errCode = PvCaptureQueueClear(GCamera.Handle)) != ePvErrSuccess)
		printf("PvCaptureQueueClear err: %u\n", errCode);
	else
		printf("...Queue cleared.\n");  

	//stop driver stream
	if ((errCode = PvCaptureEnd(GCamera.Handle)) != ePvErrSuccess)
		printf("PvCaptureEnd err: %u\n", errCode);
	else
		printf("Driver stream stopped.\n");
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

        // grab a camera from the list
        if(CameraGet() && !GCamera.Abort)
        {
            // open camera
            if(CameraSetup())
            {
                // capture FRAMESCOUNT num frames
                CameraStart();
				// stop streaming
                CameraStop();
                // unsetup the camera
                CameraUnsetup();
            }
        }
        else
            printf("failed to find a camera\n");
       
        // uninitialise the API
        PvUnInitialize();
    }

	return 0;
}
