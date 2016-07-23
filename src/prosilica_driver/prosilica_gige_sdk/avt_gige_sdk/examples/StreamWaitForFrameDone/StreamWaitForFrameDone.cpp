/*
| ==============================================================================
| Copyright (C) 2006-2011 Allied Vision Technologies.  All Rights Reserved.
|
| This code may be used in part, or in whole for your application development.
|
|==============================================================================
|
| StreamWaitForFrameDone
|
| -Open first camera found.
| -Set camera streaming, using a circular buffer queue with no frame callbacks.
| -Uses PvCaptureWaitForFrameDone to block main thread until frame returned,
|  camera unplugged, or ctrl+C.
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
#include <Winsock2.h>
#endif

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/times.h>
#include <arpa/inet.h>
#endif

#include <PvApi.h>

#ifdef _WINDOWS
#define _STDCALL __stdcall
#else
#define _STDCALL
#define TRUE     0
#endif

// number of frames to be used 
#define FRAMESCOUNT 3

typedef struct 
{
    unsigned long   UID;
    tPvHandle       Handle;
    tPvFrame        Frames[FRAMESCOUNT];
    bool            Abort;
    unsigned long   Discarded; //Count of missing frames.
#ifdef _WINDOWS
    HANDLE          ThHandle;
    DWORD           ThId;
#else
    pthread_t       ThHandle;
#endif    
    
} tCamera;


// global camera data
tCamera         GCamera;

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
struct tms      gTMS;
unsigned long   gT00 = times(&gTMS);

void Sleep(unsigned int time)
{
    struct timespec t,r;
    
    t.tv_sec    = time / 1000;
    t.tv_nsec   = (time % 1000) * 1000000;    
    
    while(nanosleep(&t,&r)==-1)
        t = r;
}

void SetConsoleCtrlHandler(void (*func)(int), int junk)
{
    signal(SIGINT, func);    
}
#endif

//Thread start function. Displays camera stats.
#ifdef _WINDOWS
unsigned long __stdcall ThreadFunc(void *pContext)
#else
void *ThreadFunc(void *pContext)
#endif
{
    unsigned long FrameCompleted, FrameDropped, PacketReceived, PacketMissed;
    float FrameRate;
    tPvErr Err;
    
    //StatFramesCompleted increments when a queued frame returns with tPvFrame.Status = ePvErrSuccess
	//StatFramesDropped increments when a queued frame returns with tPvFrame.Status = ePvErrDataMissing
	//In a situation where a camera returns a frame, but there is no frame queued for it, THESE
	//STATS DO NOT INCREMENT. tPvFrame.FrameCount increments regardless of host queuing, and is a better measure
	//for what frame is being returned from the camera. See CameraStart, where we check this parameter,
	//for the case where a frame is returned from camera with no frame queued on host.
	while(!GCamera.Abort &&
          ((Err = PvAttrUint32Get(GCamera.Handle,"StatFramesCompleted",&FrameCompleted)) == ePvErrSuccess) &&
          ((Err = PvAttrUint32Get(GCamera.Handle,"StatFramesDropped",&FrameDropped)) == ePvErrSuccess) &&
		  ((Err = PvAttrUint32Get(GCamera.Handle,"StatPacketsMissed",&PacketMissed)) == ePvErrSuccess) &&
		  ((Err = PvAttrUint32Get(GCamera.Handle,"StatPacketsReceived",&PacketReceived)) == ePvErrSuccess) &&
          ((Err = PvAttrFloat32Get(GCamera.Handle,"StatFrameRate",&FrameRate)) == ePvErrSuccess))
    {
        printf("FrmCmp : %5lu  FrmDrp : %5lu PckCmp : %5lu PckMss : %5lu FrmRt : %5.2f\r", FrameCompleted, FrameDropped, PacketReceived, PacketMissed, FrameRate);
		Sleep(20);
	}

    return 0;
}

// Create a thread to display camera stats.
// By spawning a new thread, we're able to display stats independent of frames returning. 
void SpawnThread()
{
#ifdef _WINDOWS	
    GCamera.ThHandle = CreateThread(NULL,NULL,ThreadFunc,&GCamera,NULL,&(GCamera.ThId));
#else
    pthread_create(&GCamera.ThHandle,NULL,ThreadFunc,(void *)&GCamera);
#endif    
}

// wait for the thread to be over
void WaitThread()
{
    #ifdef _WINDOWS		
    WaitForSingleObject(GCamera.ThHandle,INFINITE);
    #else
    pthread_join(GCamera.ThHandle,NULL);
    #endif
}

// wait for camera to be plugged in
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
    printf("\nCtrl+C interrupt received.\n");    
    
    GCamera.Abort = true;    
        
    #ifndef _WINDOWS
    signal(SIGINT, CtrlCHandler);
    #else
    return true;
    #endif
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
		printf("Camera closed.\n");
	}
	// delete image buffers
    for(int i=0;i<FRAMESCOUNT;i++)
        delete [] (char*)GCamera.Frames[i].ImageBuffer;

    GCamera.Handle = NULL;
}

// setup and start streaming with PvCaptureWaitForFrameDone
// Note this function doesn't return until Ctrl+C abort.
// return value: true == success (code exits on Ctrl+C abort), false == fail (other failure)
bool CameraStart()
{
    tPvErr errCode;
	bool failed = false;
	int Index = 0;
    unsigned long Last = 0;

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
	
	// queue frames. No FrameDoneCB callback function. 
	for(int i=0;i<FRAMESCOUNT && !failed;i++)
	{           
		if((errCode = PvCaptureQueueFrame(GCamera.Handle,&(GCamera.Frames[i]),NULL)) != ePvErrSuccess)
		{
			printf("CameraStart: PvCaptureQueueFrame err: %u\n", errCode);
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
		return false;
	}	 
    
	//Loop until failure or CRTL+C abort
    while(!failed && !GCamera.Abort)
    {
		//Wait for [Index] frame of FRAMESCOUNT num frames
		//wait for frame to return from camera to host
		while((errCode = PvCaptureWaitForFrameDone(GCamera.Handle,&GCamera.Frames[Index],2000)) == ePvErrTimeout)
			printf("\nWaiting for frame to return to host...\n");
	
		if(errCode != ePvErrSuccess)
        { 
			//likely camera unplugged
			printf("PvCaptureWaitForFrameDone err: %u\n", errCode);
			failed = true;
		}
		else
		{
			if (GCamera.Frames[Index].Status != ePvErrSuccess)
				printf("Frame: %lu Error: %u\n", GCamera.Frames[Index].FrameCount, GCamera.Frames[Index].Status);	

			// if frame hasn't been cancelled, requeue frame
			if(GCamera.Frames[Index].Status != ePvErrCancelled)
			{
				//Check for gaps in FrameCount due to image returning from camera with no frame queued.
				//This should never happen, as we use a multiple frame circular buffer. 
				if(Last + 1 != GCamera.Frames[Index].FrameCount)
				{
					//Note missing frame
					GCamera.Discarded++;
				}
                    
				Last = GCamera.Frames[Index].FrameCount;
                
				//Requeue [Index] frame of FRAMESCOUNT num frames
				if ((errCode = PvCaptureQueueFrame(GCamera.Handle,&GCamera.Frames[Index],NULL)) != ePvErrSuccess)
				{
					printf("PvCaptureQueueFrame err %u\n", errCode);
					failed = true;
				}
				
				//Increment [Index]
				Index++;
				if(Index==FRAMESCOUNT)
					Index = 0;                   
			}
			else
			{
				//Cancelled
				failed = true;
			}
		}
    }

	return !failed;
}


// stop streaming, clear queue.
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
		printf("PvInitialize err: %u\n", errCode);
	else
	{
		//IMPORTANT: Initialize camera structure. See tPvFrame in PvApi.h for more info.
		memset(&GCamera,0,sizeof(tCamera));

		// set the CTRL+C handler
		SetConsoleCtrlHandler(&CtrlCHandler, TRUE);
      
		// wait for a camera to be plugged
		WaitForCamera();

		// grab first camera found
		if(CameraGet())
		{
			// setup the camera
			if(CameraSetup())
			{                    
				// spawn a thread to display stats
				SpawnThread();
				
				printf("Press CTRL+C to terminate\n");
				// start streaming from the camera. 
				// blocks main until Ctrl+c or failure
				CameraStart();
				
				//signal spawned thread to complete
				//if not already done
				GCamera.Abort = true;
                                                             
				// stop the streaming
				CameraStop();                        
                
				// then wait for the thread to finish
				if(GCamera.ThHandle)
					WaitThread();                    
                
				// unsetup the camera
				CameraUnsetup();

				printf("%lu frames missed by host frame queue\n", GCamera.Discarded);
			}
		}
		PvUnInitialize();
	}   

	return 0;
}
