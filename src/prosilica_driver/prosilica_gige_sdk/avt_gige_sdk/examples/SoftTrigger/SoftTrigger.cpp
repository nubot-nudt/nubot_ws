/*
  ==============================================================================
  Copyright (C) 2006-2014 Allied Vision Technologies.  All Rights Reserved.
 
  This code may be used in part, or in whole for your application development.
 
 ==============================================================================
 
  SoftTrigger
 
  -Opens the first camera found. 
  -Frame queued. 
  -Sets camera to software trigger mode. 
  -Acquisition start readies camera to receive triggers. 
  -Once user presses 's' on keyboard, software trigger command is sent, 
  camera returns image to host. 
  -Host waits for frame to be return from queue, save images, and requeues frame.
 
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
#endif

#include <PvApi.h>
#include <ImageLib.h>

// camera's data
typedef struct 
{
    unsigned long   UID;
    tPvHandle       Handle;
    tPvFrame        Frame;
    tPvUint32       Counter;
    char            Filename[20];

} tCamera;

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
void Sleep(unsigned int time)
{
    struct timespec t,r;
    
    t.tv_sec    = time / 1000;
    t.tv_nsec   = (time % 1000) * 1000000;    
    
    while(nanosleep(&t,&r)==-1)
        t = r;
}
#endif

// wait for camera to be plugged in
void WaitForCamera()
{
    printf("Waiting for a camera ");
    while(PvCameraCount() == 0)
    {
        printf(".");
        Sleep(250);
    }
    printf("\n");
}

// wait for the user to press q
// return value: true == snap, false == quit
bool WaitForUserToQuitOrSnap()
{
    char c;

    do
    {
        c = getc(stdin);

    } while(c != 'q' && c != 's');

    return c == 's';
}

// get the first camera found
// return value: true == success, false == fail
bool CameraGet(tCamera* Camera)
{
    tPvUint32 count,connected;
    tPvCameraInfoEx list;

    //regardless if connected > 1, we only set UID of first camera in list
	count = PvCameraListEx(&list,1,&connected, sizeof(tPvCameraInfoEx));
    if(count == 1)
    {
        Camera->UID = list.UniqueId;
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
bool CameraSetup(tCamera* Camera)
{
    tPvErr errCode;
	unsigned long FrameSize = 0;

	//open camera
	if ((errCode = PvCameraOpen(Camera->UID,ePvAccessMaster,&(Camera->Handle))) != ePvErrSuccess)
	{
		if (errCode == ePvErrAccessDenied)
			printf("PvCameraOpen returned ePvErrAccessDenied:\nCamera already open as Master, or camera wasn't properly closed and still waiting to HeartbeatTimeout.");
		else
			printf("PvCameraOpen err: %u\n", errCode);
		return false;
	}

	// Calculate frame buffer size
    if((errCode = PvAttrUint32Get(Camera->Handle,"TotalBytesPerFrame",&FrameSize)) != ePvErrSuccess)
	{
		printf("CameraSetup: Get TotalBytesPerFrame err: %u\n", errCode);
		return false;
	}

    // allocate image buffer
    Camera->Frame.ImageBuffer = new char[FrameSize];
    if(!Camera->Frame.ImageBuffer)
	{
		printf("CameraSetup: Failed to allocate buffers.\n");
		return false;
	}
	Camera->Frame.ImageBufferSize = FrameSize;

	return true;
}

// close camera, free memory.
void CameraUnsetup(tCamera* Camera)
{
    tPvErr errCode;
	
	// always close an opened camera even if it was unplugged before
    if((errCode = PvCameraClose(Camera->Handle)) != ePvErrSuccess)
	{
		printf("CameraUnSetup: PvCameraClose err: %u\n", errCode);
	}
	else
	{
		printf("Camera closed.");
	}
	
	// free image buffer
    delete [] (char*)Camera->Frame.ImageBuffer;
}

// setup and start streaming
// return value: true == success, false == fail
bool CameraStart(tCamera* Camera)
{
    tPvErr errCode;

    // NOTE: This call sets camera PacketSize to largest sized test packet, up to 8228, that doesn't fail
	// on network card. Some MS VISTA network card drivers become unresponsive if test packet fails. 
	// Use PvUint32Set(handle, "PacketSize", MaxAllowablePacketSize) instead. See network card properties
	// for max allowable PacketSize/MTU/JumboFrameSize. 
	if((errCode = PvCaptureAdjustPacketSize(Camera->Handle,8228)) != ePvErrSuccess)
	{
		printf("CameraStart: PvCaptureAdjustPacketSize err: %u\n", errCode);
		return false;
	}

    // start driver capture stream 
	if((errCode = PvCaptureStart(Camera->Handle)) != ePvErrSuccess)
	{
		printf("CameraStart: PvCaptureStart err: %u\n", errCode);
		return false;
	}
	
    // queue frame
	if((errCode = PvCaptureQueueFrame(Camera->Handle,&(Camera->Frame),NULL)) != ePvErrSuccess)
	{
		printf("CameraStart: PvCaptureQueueFrame err: %u\n", errCode);
		// stop driver capture stream
		PvCaptureEnd(Camera->Handle);
		return false;
	}
		
	// set the camera in software trigger, continuous mode, and start camera receiving triggers
	if((PvAttrEnumSet(Camera->Handle,"FrameStartTriggerMode","Software") != ePvErrSuccess) ||
		(PvAttrEnumSet(Camera->Handle,"AcquisitionMode","Continuous") != ePvErrSuccess) ||
		(PvCommandRun(Camera->Handle,"AcquisitionStart") != ePvErrSuccess))
	{		
		printf("CameraStart: failed to set camera attributes\n");
		// clear queued frame
		PvCaptureQueueClear(Camera->Handle);
		// stop driver capture stream
		PvCaptureEnd(Camera->Handle);
		return false;
	}	

	return true;
}

// stop streaming
void CameraStop(tCamera* Camera)
{
    tPvErr errCode;
	
	//stop camera receiving triggers
	if ((errCode = PvCommandRun(Camera->Handle,"AcquisitionStop")) != ePvErrSuccess)
		printf("AcquisitionStop command err: %u\n", errCode);
	else
		printf("Camera stopped.\n");

    //clear queued frames. will block until all frames dequeued
	if ((errCode = PvCaptureQueueClear(Camera->Handle)) != ePvErrSuccess)
		printf("PvCaptureQueueClear err: %u\n", errCode);
	else
		printf("Queue cleared.\n");  

	//stop driver stream
	if ((errCode = PvCaptureEnd(Camera->Handle)) != ePvErrSuccess)
		printf("PvCaptureEnd err: %u\n", errCode);
	else
		printf("Driver stream stopped.\n");
}

// trigger and save a frame from the camera
// return value: true == success, false == fail
bool CameraSnap(tCamera* Camera)
{
    tPvErr errCode;
	
	//software trigger camera
	printf("Triggering camera.\n");  
	if((errCode = PvCommandRun(Camera->Handle,"FrameStartTriggerSoftware")) != ePvErrSuccess)
	{
		printf("CameraSnap: FrameStartTriggerSoftware err: %u\n", errCode);
		return false;
	}

    //wait for frame to return from camera to host. short timeout here to show ~time for return.
    while(PvCaptureWaitForFrameDone(Camera->Handle,&(Camera->Frame),10) == ePvErrTimeout)
        printf("Waiting for frame to return to host...\n");

    //check returned Frame.Status
	if(Camera->Frame.Status == ePvErrSuccess)
    {
		sprintf_s(Camera->Filename,sizeof(Camera->Filename),"./snap%04lu.tiff",++Camera->Counter);
		//save image
		if(!ImageWriteTiff(Camera->Filename,&(Camera->Frame)))
		{
			printf("ImageWriteTiff fail.\n");
			return false;
		}

		printf("snap%04lu.tiff saved.\n", Camera->Counter);
	}
    else
	{
		if (Camera->Frame.Status == ePvErrDataMissing)
			printf("Dropped packets. Possible improper network card settings:\nSee GigE Installation Guide.");
		else
			printf("Frame.Status error: %u\n",Camera->Frame.Status);
	}

	//requeue frame	
	if((errCode = PvCaptureQueueFrame(Camera->Handle,&(Camera->Frame),NULL)) != ePvErrSuccess)
	{
        printf("CameraSnap: PvCaptureQueueFrame err: %u\n", errCode);
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
		tCamera Camera;

        //IMPORTANT: Initialize camera structure. See tPvFrame in PvApi.h for more info.
		memset(&Camera,0,sizeof(tCamera));

        // wait for a camera to be plugged in
        WaitForCamera();

        // get first camera found
        if(CameraGet(&Camera))
        {
            // open camera
            if(CameraSetup(&Camera))
            {
                // start camera streaming
                if(CameraStart(&Camera))
                {
					printf("\nPress q to quit or s to take a picture.\n");
                    // snap or quit
                    while(WaitForUserToQuitOrSnap() &&
						CameraSnap(&Camera))
                    {
                        printf("\nPress q to quit or s to take a picture.\n");
                    }
                    // stop camera streaming
                    CameraStop(&Camera);
                }
                
                // close camera
                CameraUnsetup(&Camera);
            }
        }        
       
        // uninitialize PvAPI
        PvUnInitialize();
    }
    
	return 0;
}
