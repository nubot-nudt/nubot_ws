/*
  ==============================================================================
  Copyright (C) 2006-2012 Allied Vision Technologies.  All Rights Reserved.
 
  This code may be used in part, or in whole for your application development.
 
 ==============================================================================
 
  Based on Multistream and EventCallback to test PTP mode. 
 
  See Event Controls section of the "Camera and Driver" attributes document
  online at http://www.alliedvisiontec.com for the latest listing of supported
  events.
 
  Streams from multiple cameras which IP addresses are given
  in the command line arguments. Enables and prints PTP related events, and frame
  timestamps.
 
  Camera IP address(es) can be found with ListCameras
  example code, or DeviceIPAddress attribute in SampleViewer.
 
      e.g. usage: EventCallbackPTP 169.254.23.10 169.254.17.21
                  Press 'q' to quit or 't' to set PtpAquisitionGateTime
 
  See the note on dividing bandwidth amongst cameras in CameraStart.
 
  
 
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
	bool			PTPSynced;   //Are we in sync?
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

// allocate memory
// return value: true == success, false == fail
bool CameraSetup(tCamera& Camera)
{
    char lValue[128];
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

	//Reset PTP Mode
    if (PvAttrEnumSet(Camera.Handle,"PtpMode","Off") != ePvErrSuccess)
	{
		printf("Cam[%u]: failed to set PtpMode = Off\n", Camera.ID);
		return false;
	}
	else
	{
		printf("Cam[%u]: PtpMode = Off.\n", Camera.ID);
	}
	
    if(PvAttrEnumGet(Camera.Handle,"PtpStatus",lValue,128,NULL) == ePvErrSuccess)
        printf("Cam[%u]: PtpStatus = %s\n", Camera.ID, lValue); 
    else
        printf("Cam[%u]: EnumGet fail\n", Camera.ID);

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
// return value: true == success (code exits on abort), false == fail (other failure)
bool CameraStart(tCamera& Camera)
{
    char lValue[128];
	tPvErr errCode;
	bool failed = false;
	int Index = 0;

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
	if((PvAttrUint32Set(Camera.Handle,"StreamBytesPerSecond", 50000000 / GSession.Count) != ePvErrSuccess))
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

	// set the camera in fixed rate, 1 fps, continuous mode, but NOT AcquisitionStart
	if((PvAttrFloat32Set(Camera.Handle,"FrameRate",1) != ePvErrSuccess) ||
		(PvAttrEnumSet(Camera.Handle,"FrameStartTriggerMode","FixedRate") != ePvErrSuccess) ||
		(PvAttrEnumSet(Camera.Handle,"AcquisitionMode","Continuous") != ePvErrSuccess))
	{		
		printf("Cam[%u]: failed to set camera attributes\n", Camera.ID);
		return false;
	}	 
    
	
	//Set one camera to Master, others to Slave.
    if (Camera.ID == 1)
	{
		if (PvAttrEnumSet(Camera.Handle,"PtpMode","Master") != ePvErrSuccess)
		{
			printf("Cam[%u]: failed to set PtpMode = Master\n", Camera.ID);
			return false;
		}
		else
		{
			printf("Cam[%u] Set PtpMode = Master\n", Camera.ID);
		}
	}
	else
	{
		if (PvAttrEnumSet(Camera.Handle,"PtpMode","Slave") != ePvErrSuccess)
		{
			printf("Cam[%u]: failed to set PtpMode = Slave\n", Camera.ID);
			return false;
		}
		else
		{
			printf("Cam[%u]: Set PtpMode = Slave\n", Camera.ID);
		}
	}
	
	/*
	//Auto test: Set PTP Mode on all cameras to Auto
	if (PvAttrEnumSet(Camera.Handle,"PtpMode","Auto") != ePvErrSuccess)
	{
		printf("Cam[%u]: failed to set PtpMode = Auto\n", Camera.ID);
		return false;
	}
	else
	{
		printf("Cam[%u]: PtpMode = Auto.\n", Camera.ID);
	}
	*/

	//Wait for cameras to Sync. Global var set in F_CameraEventCallback
	printf("Cam[%u]: Waiting for EventPtpSyncLocked\n", Camera.ID);
	while (!Camera.PTPSynced && !GSession.Abort) {
		lValue[0] = '\0';

		//Test PtpStatus and print
		if(PvAttrEnumGet(Camera.Handle,"PtpStatus",lValue,128,NULL) != ePvErrSuccess)
			printf("Cam[%u]: EnumGet fail\n", Camera.ID);
		
		//if (!strcmp(lValue, "Master") || !strcmp(lValue, "Slave"))
			printf("Cam[%u]: PtpStatus = %s\n", Camera.ID, lValue); 		

		Sleep(600);
	}
	
	if (GSession.Abort)
		return false;

	//AcquisitionStart must be called after EventPtpSyncLocked occurs, and not before.
	if (PvCommandRun(Camera.Handle,"AcquisitionStart") != ePvErrSuccess)
	{
		printf("Cam[%u]: AcquisitionStart failed\n", Camera.ID);
	}
	else
	{
		printf("Cam[%u]: AcquisitionStart success\n", Camera.ID);
	}

	//Loop until failure or 'q' abort in main user thread
    while(!failed && !GSession.Abort)
    {
			
		//wait for frame to return from camera to host. Timeout = 5 sec.
		while(((errCode = PvCaptureWaitForFrameDone(Camera.Handle,&Camera.Frames[Index],5000)) == ePvErrTimeout) && !GSession.Abort)
			printf("Cam[%u]: Frame: [%lu] Timeout\n", Camera.ID, Camera.Frames[Index].FrameCount);
	
		if(errCode != ePvErrSuccess)
        { 
			printf("Cam[%u]: PvCaptureWaitForFrameDone err: %u\n", Camera.ID, errCode);
			failed = true;
		}
		else
		{							
			//print frame timestamp
			printf("Cam[%u]: Frame: [%lu] TimestampHi: [%lu] TimestampLo: [%lu]\n", Camera.ID, Camera.Frames[Index].FrameCount , Camera.Frames[Index].TimestampHi, Camera.Frames[Index].TimestampLo);
			
			// if frame hasn't been cancelled, requeue frame
			if(Camera.Frames[Index].Status != ePvErrCancelled)
			{       
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
    
	//Set PTP Mode to Off
    if (PvAttrEnumSet(Camera.Handle,"PtpMode","Off") != ePvErrSuccess)
		printf("Cam[%u]: failed to set PtpMode = Off\n", Camera.ID);
	else
		printf("Cam[%u]: PtpMode = Off\n", Camera.ID);
	
	//PvCaptureQueueClear aborts any actively written frame with Frame.Status = ePvErrDataMissing
	//Further queued frames returned with Frame.Status = ePvErrCancelled
	
	//Add delay between AcquisitionStop and PvCaptureQueueClear
	//to give actively written frame time to complete
	Sleep(1100);
	
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

// wait for the user to press q or t
// return value: true == snap, false == quit
bool WaitForUserToQuitOrTrigger()
{
    char c;

    do
    {
        c = getc(stdin);

    } while(c != 'q' && c != 't');

    return c == 'q';
}     

// Event callback.  This is called by PvApi when camera event(s) occur.
void _STDCALL F_CameraEventCallback(void*                   pContext,
                                    tPvHandle               CamHandle,
                                    const tPvCameraEvent*	EventList,
                                    unsigned long			EventListLength)
{
	tCamera* Camera = (tCamera*)pContext;
	
	//multiple events may have occurred for this one callback
	for (unsigned long i = 0; i < EventListLength; i++)
	{
		switch (EventList[i].EventId) {
			case 40000:
				printf("Cam[%u]: ***EventAcquisitionStart\n", Camera->ID);
				break;
			case 40001:
				printf("Cam[%u]: ***EventAcquisitionEnd\n", Camera->ID);
				break;
			case 40002:
				printf("Cam[%u]: ***EventFrameTrigger\n", Camera->ID);
				break;
			case 40005:
				Camera->PTPSynced = false;
				printf("Cam[%u]: ***EventPtpSyncLost\n", Camera->ID);
				break;
			case 40006:
				Camera->PTPSynced = true;
				printf("Cam[%u]: ***EventPtpSyncLocked\n", Camera->ID);
				break;
			case 65534:
				printf("Cam[%u]: ***EventOverflow error\n", Camera->ID);
				break;
			default:
				printf("Cam[%u]: ***Event %lu\n", Camera->ID, EventList[i].EventId);
				break;
		}
	}
}

// setup event channel
// return value: true == success, false == fail
bool EventSetup(tCamera& Camera)
{
	tPvErr errCode;
	
	// check if events supported with this camera firmware
	if (PvAttrExists(Camera.Handle,"EventsEnable1") == ePvErrNotFound)
	{
        printf("This camera does not support event notifications.\n");
        return false;
	}
	
	//Clear all events
	//EventsEnable1 is a bitmask of all events. Bits correspond to last two digits of EventId.
	// e.g: Bit 1 is EventAcquisitionStart, Bit 2 is EventAcquisitionEnd, Bit 10 is EventSyncIn1Rise. 
    if ((errCode = PvAttrUint32Set(Camera.Handle,"EventsEnable1",0)) != ePvErrSuccess)
	{
		printf("Set EventsEnable1 err: %u\n", errCode);
		return false;
	}
            
	//Set events: AcquisitionStart, AcquisitionEnd, PtpSyncLost, PtpSyncLocked
    //In binary this is:  0001100011. In hex: 0x63
	if ((errCode = PvAttrUint32Set(Camera.Handle,"EventsEnable1",0x63)) != ePvErrSuccess)
	{
		printf("Set EventsEnable1 err: %u\n", errCode);
		return false;
	}

    //register callback function
	if ((errCode = PvCameraEventCallbackRegister(Camera.Handle,F_CameraEventCallback,&Camera)) != ePvErrSuccess)
    {
		printf("PvCameraEventCallbackRegister err: %u\n", errCode);
        return false;
    }     
	return true;
}

// unsetup event channel
void EventUnsetup(tCamera& Camera)
{
    // wait so that the "AcquisitionEnd" [from CameraStop()] can be received on the event channel
    Sleep(1000);
	// clear all events
	PvAttrUint32Set(Camera.Handle,"EventsEnable1",0);
    // unregister callback function
	PvCameraEventCallbackUnRegister(Camera.Handle,F_CameraEventCallback);
}

//main camera thread 
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
            printf("Cam[%u]: %s [%s] opened\n",Camera->ID,IP,Name);

            if(CameraSetup(*Camera))
            {
                if (EventSetup(*Camera))
				{
					CameraStart(*Camera);
					CameraStop(*Camera);
					EventUnsetup(*Camera);
					CameraUnsetup(*Camera);
				}
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

// main user thread
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
                
				printf("======================================================\n");
				printf("Press 'q' to quit or 't' to set PtpAquisitionGateTime.\n");
				printf("======================================================\n");
				while(GSession.Abort == false)
                {
                    GSession.Abort = WaitForUserToQuitOrTrigger();
					if (!GSession.Abort)
					{	
						//'t' = set future software trigger, i.e. PtpAcquisitionGateTime
						
						//TimeStampValueLo units: 1 nanosecond
						//TimeStampValueHi units: ~ 4.295 seconds
						unsigned long TSVHi1, TSVLo1, TSVHi2, TSVLo2, TSVHtemp, TSVLtemp;
						
						/*#ifdef _WINDOWS
							DWORD SystemTime1, SystemTime2;
						#else
							int SystemTime1, SystemTime2;
						#endif
						*/

						//Get TimeStampValue of one camera. There will be less than < 2us variation in this value between all cameras				
						//Note: must TimeStampValueLatch before read
						PvCommandRun(GSession.Cameras[0].Handle, "TimeStampValueLatch");

						if (PvAttrUint32Get(GSession.Cameras[0].Handle,"TimeStampValueHi",&TSVHi1) != ePvErrSuccess) 
						{
							printf("Failed to get TimeStampValueHi\n");
						}
						if (PvAttrUint32Get(GSession.Cameras[0].Handle,"TimeStampValueLo",&TSVLo1) != ePvErrSuccess)
						{
							printf("Failed to get TimeStampValueLo\n");
						}

						TSVHtemp = TSVHi1;
						TSVLtemp = TSVLo1;

						printf("Setting PtpAquisitionGateTime 4.295 seconds from now.\n\n");
						//set PtpAcquisitionGateTime 4.295 seconds (resolution of PtpAcquisitionGateTimeHi) in the future for all cameras
						for(i=0;i<GSession.Count;i++)
						{		
							if (PvAttrUint32Set(GSession.Cameras[i].Handle, "PtpAcquisitionGateTimeHi", TSVHi1 + 1) != ePvErrSuccess)
							{
								printf("Failed to set PtpAcquisitionGateTimeHi\n");
							}				
							
							if (PvAttrUint32Set(GSession.Cameras[i].Handle, "PtpAcquisitionGateTimeLo", TSVLo1) != ePvErrSuccess)
							{
								printf("Failed to set PtpAcquisitionGateTimeLo\n");
							}
							
							//Get TimeStampValue again, to see how long the above calls took. 
							//Use these to optimize a system for an arbitrary number of cameras. 
							//You may wish to set PtpAcquisitionGateTime as near into
							//the future as possible, without the camera TimeStampValue incrementing past the value before it is set.
							//A good measure for this is to combine the time it takes to set PtpAcquisitionGateTimeHi+Lo on all cameras. 
							//This code does not optimize PtpAcquisitionGateTime, it simply informs user of optimum values.

							PvCommandRun(GSession.Cameras[i].Handle, "TimeStampValueLatch");
							if (PvAttrUint32Get(GSession.Cameras[i].Handle,"TimeStampValueHi",&TSVHi2) != ePvErrSuccess) 
							{
								printf("Failed to get TimeStampValueHi\n");
							}
							if (PvAttrUint32Get(GSession.Cameras[i].Handle,"TimeStampValueLo",&TSVLo2) != ePvErrSuccess)
							{
								printf("Failed to get TimeStampValueLo\n");
							}
							
							printf("Elapsed time to set PtpAcquisitionGateTime for Cam[%u]: : %lu ns\n\n", i, ((TSVHi2 - TSVHtemp)<<31) + TSVLo2 - TSVLtemp);
							TSVHtemp = TSVHi2;
							TSVLtemp = TSVLo2;
						}
						printf("Total these times plus some offset to determine the safe minimum time to set \n");
						printf("PtpAcquisitionTime for all cameras, rather than the arbitrary 4.295 seconds.\n\n");
						printf("======================================================\n");
						printf("Press 'q' to quit or 't' to set PtpAquisitionGateTime.\n");
						printf("======================================================\n");
					}
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
            printf("usage: EventCallbackPTP <Camera IP #1> <Camera IP#2> ...\n");

        // uninitialize the API
        PvUnInitialize();
	}

    return 0;
}




