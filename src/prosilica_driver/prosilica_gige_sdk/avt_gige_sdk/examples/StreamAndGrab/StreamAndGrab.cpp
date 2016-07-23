/*
| ==============================================================================
| Copyright (C) 2006-2011 Allied Vision Technologies.  All Rights Reserved.
|
| This code may be used in part, or in whole for your application development.
|
|==============================================================================
|
| StreamAndGrab 
|
| Based on Stream. Main thread waits for user input
| 's' to save next queued frame as TIFF in frame callback, or 'q' to exit
|
| -Opens the first camera found. 
| -Queues FRAMESCOUNT number of frames, each with a frame callback function. 
| -Sets camera to freerun continuous mode. 
| -Main waits for 's' or 'q' + enter input
| -Frame callback function displays info, saves frame if 's', requeues frame. 
|
|==============================================================================
|
| THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
| WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
| NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
| DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
| INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
| LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
| OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  AND ON ANY THEORY OF
| LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
| NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
| EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
|
|==============================================================================
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

#define FRAMESCOUNT 15

typedef struct 
{
    unsigned long   UID;
    tPvHandle       Handle;
    tPvFrame        Frames[FRAMESCOUNT];
	bool			SaveFrame;
	char            Filename[20];
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
#endif


// wait for a camera to be plugged in
void WaitForCamera()
{
    printf("Waiting for a camera");
    while(PvCameraCount() == 0)
	{
		printf(".");
		Sleep(250);
	}
    printf("\n");
}

// wait for the user to press q or s, followed by enter.
// or camera unplugged.
// return value: true == snap, false == quit
bool WaitForUserToQuitOrSnap()
{
    char c;
	tPvUint32 exposureVal;
	tPvErr errCode;

    do
    {	
		//halts program until user input
		c = getc(stdin);
		
		//Lazy/poor way to check for unplugged camera. See StreamPnp for 
		//registering a ePvLinkRemove callback.
		if ((errCode = PvAttrUint32Get(GCamera.Handle, "ExposureValue", &exposureVal)) == ePvErrUnplugged)
		{
			printf("Camera Unplugged.\n");
			c = 'q';
		}

	} while(c != 'q' && c != 's');
	
	return (c == 's');
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
	//Display FrameCount and Status
	if (pFrame->Status == ePvErrSuccess)
	{
		printf("Frame: %lu returned successfully.\r", pFrame->FrameCount);

		//User input 's'? Save image
		if (GCamera.SaveFrame)
		{
			sprintf_s(GCamera.Filename,sizeof(GCamera.Filename),"./Frame%04lu.tiff",pFrame->FrameCount);
			//save image
			if(!ImageWriteTiff(GCamera.Filename,pFrame))
			{
				printf("ImageWriteTiff fail.             \n");
			}
			else
				printf("Frame: %lu Saved to disk.         \n", pFrame->FrameCount);

			GCamera.SaveFrame = false;
		}
	}
	else if (pFrame->Status == ePvErrDataMissing)
	{
		//Possible improper network card settings. See GigE Installation Guide.
		printf("Frame: %lu dropped packets\n", pFrame->FrameCount);
	}
	else if (pFrame->Status == ePvErrCancelled)
	{
		printf("Frame Cancelled\n");
	}
	else
	{
		printf("Frame: %lu Error: %u\n", pFrame->FrameCount, pFrame->Status);
	}	
	
	//Requeue frame
	if (pFrame->Status != ePvErrCancelled)
	{
		tPvErr errCode;
		if ((errCode = PvCaptureQueueFrame(GCamera.Handle,pFrame,FrameDoneCB)) != ePvErrSuccess)
		{
			printf("PvCaptureQueueFrame err: %u           \n", errCode);
		}
	}
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

	// Calculate frame buffer size
    if((errCode = PvAttrUint32Get(GCamera.Handle,"TotalBytesPerFrame",&FrameSize)) != ePvErrSuccess)
	{
		printf("CameraSetup: Get TotalBytesPerFrame err: %u\n", errCode);
		return false;
	}

	// allocate the frame buffers
    for(int i=0;i<FRAMESCOUNT && !failed;i++)
    {
        GCamera.Frames[i].ImageBuffer = new char[FrameSize];
        if(GCamera.Frames[i].ImageBuffer)
        {
			GCamera.Frames[i].ImageBufferSize = FrameSize;
		}
        else
		{
			printf("CameraSetup: Failed to allocate buffers.\n");
			failed = true;
		}
    }

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
		printf("Camera closed.");
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
	
    // queue frames with FrameDoneCB callback function. Each frame can use a unique callback function
	// or, as in this case, the same callback function.
	for(int i=0;i<FRAMESCOUNT && !failed;i++)
	{           
		if((errCode = PvCaptureQueueFrame(GCamera.Handle,&(GCamera.Frames[i]),FrameDoneCB)) != ePvErrSuccess)
		{
			printf("CameraStart: PvCaptureQueueFrame err: %u\n", errCode);
			// stop driver capture stream
			PvCaptureEnd(GCamera.Handle);
			failed = true;
		}
	}

	if (failed)
		return false;
		
	// set the camera in freerun trigger, continuous mode, and start camera receiving triggers
	if((PvAttrEnumSet(GCamera.Handle,"FrameStartTriggerMode","Freerun") != ePvErrSuccess) ||
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
		printf("AcquisitionStop success.\n");
    
	//PvCaptureQueueClear aborts any actively written frame with Frame.Status = ePvErrDataMissing
	//Further queued frames returned with Frame.Status = ePvErrCancelled
	
	//Add delay between AcquisitionStop and PvCaptureQueueClear
	//to give actively written frame time to complete
	Sleep(200);
	
	printf("\nCalling PvCaptureQueueClear...\n");
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

        // wait for a camera to be plugged in
        WaitForCamera();

        // get first camera found
        if(CameraGet())
        {
            // open camera
            if(CameraSetup())
            {
                // start streaming from the camera
                if(CameraStart())
                {
                    printf("Camera is streaming.\n");
                    printf("Press 'q' (quit) or 's' (save), followed by 'enter'\n");
                    // snap or quit
                    while(WaitForUserToQuitOrSnap())
                    {
                        //Flag callback function to save next returning frame 
						GCamera.SaveFrame = true;
					}
                    // stop the camera
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
