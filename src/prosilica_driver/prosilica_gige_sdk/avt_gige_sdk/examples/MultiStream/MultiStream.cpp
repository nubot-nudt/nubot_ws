/*
  ==============================================================================
  Copyright (C) 2006-2014 Allied Vision Technologies.  All Rights Reserved.
 
  This code may be used in part, or in whole for your application development.
 
 ==============================================================================
 
  MultiStream
 
  Based on StreamWaitForFrameDone example
 
  Streams from the camera(s) which IP address(es) are given
  in the command line arguments. Camera IP address(es) can be found with ListCameras
  example code, or DeviceIPAddress attribute in SampleViewer.
 
      e.g. usage: MultiStream 169.254.23.10 169.254.17.21 169.254.33.1
 
  Each camera is opened and streamed in a separate thread -- if 3 cameras are
  given, there will be 3 threads. Each camera uses a circular buffer queue,
  with it's thread calling PvCaptureWaitForFrameDone. One camera dropping out
  won't affect the other cameras. 
 
  See the note on dividing bandwidth amongst cameras in CameraStart.
 
  Ctrl+C required to exit main thread.
 
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

// camera data
typedef struct 
{
    int             ID;
    unsigned long   IP;
    tPvHandle       Handle;
    tPvFrame        Frames[FRAMESCOUNT];
	unsigned long   Discarded;
#ifdef _WINDOWS
    HANDLE          ThHandle;
    DWORD           ThId;
#else
    pthread_t       ThHandle;
#endif    
    
} tCamera;

// session data
typedef struct
{
    int      Count; 
    tCamera* Cameras;
    bool     Abort;

} tSession;

///////////////

// global data
tSession GSession;

///////////////

#if defined(_LINUX) || defined(_QNX) || defined(_OSX)

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

// Print cam stats
void CameraStats(tCamera& Camera)
{
    unsigned long FrameCompleted, FrameDropped, PacketMissed;
    
    //StatFramesCompleted increments when a queued frame returns with tPvFrame.Status = ePvErrSuccess
	//StatFramesDropped increments when a queued frame returns with tPvFrame.Status = ePvErrDataMissing
	//In a situation where a camera returns a frame, but there is no frame queued for it, THESE
	//STATS DO NOT INCREMENT. tPvFrame.FrameCount increments regardless of host queuing, and is a better measure
	//for what frame is being returned from the camera. 
	if((PvAttrUint32Get(Camera.Handle,"StatFramesCompleted",&FrameCompleted) == ePvErrSuccess) &&
          (PvAttrUint32Get(Camera.Handle,"StatFramesDropped",&FrameDropped) == ePvErrSuccess) &&
		  (PvAttrUint32Get(Camera.Handle,"StatPacketsMissed",&PacketMissed) == ePvErrSuccess))
	{
		printf("C%u: FCmp %5lu  FDrp %4lu PDrp %6lu ", Camera.ID, FrameCompleted, FrameDropped, PacketMissed);
	}	 
}

// allocate memory
// return value: true == success, false == fail
bool CameraSetup(tCamera& Camera)
{
    tPvErr errCode;
	bool failed = false;
	unsigned long FrameSize = 0;

	// Calculate frame buffer size
    if((errCode = PvAttrUint32Get(Camera.Handle,"TotalBytesPerFrame",&FrameSize)) != ePvErrSuccess)
	{
		printf("Cam[%u]: Get TotalBytesPerFrame err: %u\n", Camera.ID, errCode);
		return false;
	}

	// allocate the frame buffers
    for(int i=0;i<FRAMESCOUNT && !failed;i++)
    {
        Camera.Frames[i].ImageBuffer = new char[FrameSize];
        if(Camera.Frames[i].ImageBuffer)
        {
			Camera.Frames[i].ImageBufferSize = FrameSize;
		}
        else
		{
			printf("Cam[%u]: Failed to allocate buffers.\n", Camera.ID);
			failed = true;
		}
    }

	return !failed;
}

// close camera, free memory.
void CameraUnsetup(tCamera& Camera)
{
    tPvErr errCode;
	
	// always close an opened camera even if it was unplugged before
    if((errCode = PvCameraClose(Camera.Handle)) != ePvErrSuccess)
	{
		printf("Cam[%u]: PvCameraClose err: %u\n", Camera.ID, errCode);
	}
	else
	{
		printf("Cam[%u]: Closed.\n", Camera.ID);
	}
	// delete image buffers
    for(int i=0;i<FRAMESCOUNT;i++)
        delete [] (char*)Camera.Frames[i].ImageBuffer;

    Camera.Handle = NULL;
}

// setup and start streaming with PvCaptureWaitForFrameDone
// return value: true == success (code exits on Ctrl+C abort), false == fail (other failure)
bool CameraStart(tCamera& Camera)
{
    tPvErr errCode;
	bool failed = false;
	int Index = 0;
    unsigned long Last = 0;

    // NOTE: This call sets camera PacketSize to largest sized test packet, up to 8228, that doesn't fail
	// on network card. Some MS VISTA network card drivers become unresponsive if test packet fails. 
	// Use PvUint32Set(handle, "PacketSize", MaxAllowablePacketSize) instead. See network card properties
	// for max allowable PacketSize/MTU/JumboFrameSize. 
	if((errCode = PvCaptureAdjustPacketSize(Camera.Handle,8228)) != ePvErrSuccess)
	{
		printf("Cam[%u]: PvCaptureAdjustPacketSize err: %u\n", Camera.ID, errCode);
		return false;
	}

	// Assuming hardware setup is multiple cameras on a switch, with a single GigE NIC
	// Therefore total available bandwidth is ~ 115MB. Divide bandwidth amongst cameras
	// or get packet collisions, i.e. frames returning with ePvErrDataMissing.
	if((PvAttrUint32Set(Camera.Handle,"StreamBytesPerSecond", 115000000 / GSession.Count) != ePvErrSuccess))
	{
		printf("Cam[%u]: Set StreamBytesPerSecond err: %u\n", Camera.ID, errCode);
		return false;
	}

    // start driver capture stream 
	if((errCode = PvCaptureStart(Camera.Handle)) != ePvErrSuccess)
	{
		printf("Cam[%u]: PvCaptureStart err: %u\n", Camera.ID, errCode);
		return false;
	}
	
	// queue frames. No FrameDoneCB callback function. 
	for(int i=0;i<FRAMESCOUNT && !failed;i++)
	{           
		if((errCode = PvCaptureQueueFrame(Camera.Handle,&(Camera.Frames[i]),NULL)) != ePvErrSuccess)
		{
			printf("Cam[%u]: PvCaptureQueueFrame err: %u\n", Camera.ID, errCode);
			failed = true;
		}
	}

	if (failed)
		return false;
		
	// set the camera in freerun trigger, continuous mode, and start camera receiving triggers
	if((PvAttrEnumSet(Camera.Handle,"FrameStartTriggerMode","Freerun") != ePvErrSuccess) ||
		(PvAttrEnumSet(Camera.Handle,"AcquisitionMode","Continuous") != ePvErrSuccess) ||
		(PvCommandRun(Camera.Handle,"AcquisitionStart") != ePvErrSuccess))
	{		
		printf("Cam[%u]: failed to set camera attributes\n", Camera.ID);
		return false;
	}	 
    
	//Loop until failure or CRTL+C abort
    while(!failed && !GSession.Abort)
    {
		//Wait for [Index] frame of FRAMESCOUNT num frames
		//wait for frame to return from camera to host
		while((errCode = PvCaptureWaitForFrameDone(Camera.Handle,&Camera.Frames[Index],2000)) == ePvErrTimeout)
			printf("\nCam[%u]: Waiting for frame to return to host...\n", Camera.ID);
	
		if(errCode != ePvErrSuccess)
        { 
			//likely camera unplugged
			printf("Cam[%u]: PvCaptureWaitForFrameDone err: %u\n", Camera.ID, errCode);
			failed = true;
		}
		else
		{
			// if frame hasn't been cancelled, requeue frame
			if(Camera.Frames[Index].Status != ePvErrCancelled)
			{
				//Check for gaps in FrameCount due to image returning from camera with no frame queued.
				//This should never happen, as we use a multiple frame circular buffer. 
				if(Last + 1 != Camera.Frames[Index].FrameCount)
				{
					//Note missing frame
					Camera.Discarded++;
				}
                    
				Last = Camera.Frames[Index].FrameCount;
                
				//Requeue [Index] frame of FRAMESCOUNT num frames
				if ((errCode = PvCaptureQueueFrame(Camera.Handle,&Camera.Frames[Index],NULL)) != ePvErrSuccess)
				{
					printf("Cam[%u]: PvCaptureQueueFrame err %u\n", Camera.ID, errCode);
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
void CameraStop(tCamera& Camera)
{
    tPvErr errCode;
	
	//stop camera receiving triggers
	if ((errCode = PvCommandRun(Camera.Handle,"AcquisitionStop")) != ePvErrSuccess)
		printf("Cam[%u]: AcquisitionStop err: %u\n", Camera.ID, errCode);
	else
		printf("Cam[%u]: AcquisitionStop success.\n", Camera.ID);
    
	//PvCaptureQueueClear aborts any actively written frame with Frame.Status = ePvErrDataMissing
	//Further queued frames returned with Frame.Status = ePvErrCancelled
	
	//Add delay between AcquisitionStop and PvCaptureQueueClear
	//to give actively written frame time to complete
	Sleep(200);
	
	printf("Cam[%u]: Calling PvCaptureQueueClear...\n", Camera.ID);
	if ((errCode = PvCaptureQueueClear(Camera.Handle)) != ePvErrSuccess)
		printf("Cam[%u]: PvCaptureQueueClear err: %u\n", Camera.ID, errCode);
	else
		printf("Cam[%u]: ...Queue cleared.\n", Camera.ID);  

	//stop driver stream
	if ((errCode = PvCaptureEnd(Camera.Handle)) != ePvErrSuccess)
		printf("Cam[%u]: PvCaptureEnd err: %u\n", Camera.ID, errCode);
	else
		printf("Cam[%u]: Driver stream stopped.\n", Camera.ID);
}


#ifdef _WINDOWS
unsigned long __stdcall ThreadFunc(void *pContext)
#else
void *ThreadFunc(void *pContext)
#endif
{
    tCamera* Camera = (tCamera*)pContext;
	tPvErr errCode;

    if((errCode = PvCameraOpenByAddr(Camera->IP,ePvAccessMaster,&(Camera->Handle))) == ePvErrSuccess)
    {
        char IP[128];
        char Name[128];

        // Read the IP and Name strings
        if(((errCode = PvAttrStringGet(Camera->Handle,"DeviceIPAddress",IP,128,NULL)) == ePvErrSuccess) &&
           ((errCode = PvAttrStringGet(Camera->Handle,"CameraName",Name,128,NULL)) == ePvErrSuccess))
        {
            printf("Cam[%u]: %s (%s) opened\n",Camera->ID,IP,Name);

            if(CameraSetup(*Camera))
            {
                CameraStart(*Camera);
                CameraStop(*Camera);
                CameraUnsetup(*Camera);
				printf("Cam[%u]: %lu frames missed by host frame queue\n", Camera->ID, Camera->Discarded);
            }
        }
        else
            printf("Cam[%u]: PvAttrStringGet err: %u\n",Camera->ID, errCode);

        PvCameraClose(Camera->Handle);
        Camera->Handle = 0;
    }
    else
		printf("Cam[%u]: PvCameraOpenByAddr err: %u\n",Camera->ID, errCode);

    return 0;
}

// CTRL-C handler
#ifdef _WINDOWS
BOOL WINAPI CtrlCHandler(DWORD dwCtrlType)
#else
void CtrlCHandler(int Signo)
#endif	
{
    // set the flag
    GSession.Abort = true;  
    
    #ifndef _WINDOWS
    signal(SIGINT, CtrlCHandler);
    #else
    return true;
    #endif
}

// main
int main(int argc, char* argv[])
{
	tPvErr errCode;
	 
	// initialize the PvAPI
	if((errCode = PvInitialize()) != ePvErrSuccess)
		printf("PvInitialize err: %u\n", errCode);
	else
	{ 
        //IMPORTANT: Initialize camera structure. See tPvFrame in PvApi.h for more info.
		memset(&GSession,0,sizeof(tSession));

        //Set Ctrl+C handler
		SetConsoleCtrlHandler(&CtrlCHandler, TRUE);
      
        if(argc>1)
        {
			GSession.Count = argc - 1;
            GSession.Cameras = new tCamera[GSession.Count];
            if(GSession.Cameras)
            {
                int i;
               
                memset(GSession.Cameras,0,sizeof(tCamera) * GSession.Count);

                for(i=1;i<argc;i++)
                {
                    GSession.Cameras[i-1].IP = inet_addr(argv[i]);
                    GSession.Cameras[i-1].ID = i;
					if((GSession.Cameras[i-1].IP == INADDR_NONE) || (GSession.Cameras[i-1].IP == INADDR_ANY))
					{
						printf("A valid IP address must be entered\n");
						GSession.Abort = true;
					}
                }

				if (GSession.Abort)
					return 0;

				printf("Press CTRL+C to terminate\n");
                printf("%d cameras to be opened\n",GSession.Count);
             
                //Spawn a thread for each camera
                for(i=0;i<GSession.Count;i++)
                {
                    if(GSession.Cameras[i].IP)
                    {
                        #ifdef _WINDOWS	
                        GSession.Cameras[i].ThHandle = CreateThread(NULL,NULL,ThreadFunc,&GSession.Cameras[i],NULL,&(GSession.Cameras[i].ThId));
                        #else
                        pthread_create(&GSession.Cameras[i].ThHandle,NULL,ThreadFunc,&GSession.Cameras[i]);
                        #endif    
                    }
                }

                // Until CTRL+C we will display simple streaming statistics
                while(GSession.Abort == false)
                {
                    for(i=0;i<GSession.Count;i++)
                    {
                        if(GSession.Cameras[i].Handle)
                            CameraStats(GSession.Cameras[i]);
                        if(i<GSession.Count - 1)
                            printf(" ");
                    }

                    printf("\r");

                    Sleep(30);
                }

                //wait for spawned threads to terminate
                for(i=0;i<GSession.Count;i++)
                {
                    if(GSession.Cameras[i].ThHandle)
                    {
                        #ifdef _WINDOWS		
                        WaitForSingleObject(GSession.Cameras[i].ThHandle,INFINITE);
                        #else
                        pthread_join(GSession.Cameras[i].ThHandle,NULL);
                        #endif   
                    }
                }

                delete [] GSession.Cameras;
            }
        }
        else
            printf("usage: MultiStream <Camera IP #1> <Camera IP#2> ...\n");

        // uninitialize the API
        PvUnInitialize();
	}

    return 0;
}
