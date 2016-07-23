/*
  ==============================================================================
  Copyright (C) 2010-2014 Allied Vision Technologies.  All Rights Reserved.
 
  This code may be used in part, or in whole for your application development.
 
 ==============================================================================
 
  Saves the image stream from a specified camera to disk,
  each frame being saved as a JPEG file.
 
  Code uses sprintf. Disable "Treat Warnings As Errors" in project properties.
 
  To use this sample code on Windows, you will need to download & build libjpeg
  from http://www.ijg.org/. On other systems (such as Linux) you will need to
  install the header files for the built-in jpeg library.
 
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

#include "Jpeg.h"
#include "Lock.h"
#include "Thread.h"

#include <queue>


#ifdef _WINDOWS
#define _STDCALL __stdcall
#else
#define _STDCALL
#define TRUE     0
#endif

#define kFRAME_MAX 100

#ifndef min 
#define min(a,b) (a < b ? a : b)
#endif

typedef unsigned long ulong;
typedef unsigned int  uint;

typedef std::queue<tPvFrame*> tQueue;

typedef struct {

    tPvFrame*       mArray;
    uint            mCount;

} tFrameArray;

typedef struct {

    tPvHandle       mCamera;
    tFrameArray     mFrames;
  
    tLock           mLock;
    tQueue          mTodo;
    tQueue          mDone; 
       
    bool            mAbort;  
    uint            mCount;
    uint            mLimit; 
    uint            mCReal;
    
    const char*     mFileRoot;
    uint            mQuality;


} tStreamContext;

typedef struct {

    tThread         mThread;
    tStreamContext* mStream;
    char            mBuffer[256];
    tJPEGContext    mWriter;

} tWriterContext;

///

#ifndef _WINDOWS
void Sleep(unsigned int time)
{
    struct timespec t,r;
    
    t.tv_sec    = time / 1000;
    t.tv_nsec   = (time % 1000) * 1000000;    
    
    while(nanosleep(&t,&r)==-1)
        t = r;
}
#endif

///

// write the frame to disk
void doSaveFrame(tWriterContext* aContext,tPvFrame* aFrame)
{
    sprintf(aContext->mBuffer,"%s/%lu.jpg",aContext->mStream->mFileRoot,aFrame->FrameCount);

    if(JPEGAdapt(&aContext->mWriter,aFrame->Format,aFrame->Width,aFrame->Height))
        JPEGWrite(&aContext->mWriter,aFrame,aContext->mBuffer,aContext->mStream->mQuality);
}

///

// framedone callback
void _STDCALL frameDoneCB(tPvFrame* aFrame)
{
    if(aFrame->Status != ePvErrUnplugged && aFrame->Status != ePvErrCancelled)
    {
        tStreamContext* lContext = (tStreamContext*)aFrame->Context[0];
    
        if(aFrame->Status == ePvErrSuccess)
        {
            acquireLock(&lContext->mLock);
            
            if(!lContext->mAbort)
            {
                lContext->mCount++;
                
                // replace the framecount by our own counter
                lContext->mCReal   = aFrame->FrameCount;
                aFrame->FrameCount = lContext->mCount;
            
                // push the completed frame for processing
                lContext->mDone.push(aFrame);
                
                // try to grab a frame from the extra-queue
                if(!lContext->mTodo.empty())
                    if(PvCaptureQueueFrame(lContext->mCamera,lContext->mTodo.front(),frameDoneCB) == ePvErrSuccess) 
                        lContext->mTodo.pop();  
                        
                if(lContext->mCount == lContext->mLimit)
                    lContext->mAbort = true;            
            }
            
            releaseLock(&lContext->mLock);
        }
        else
        {
            // when the frame is missing data, we just re-enqueue it right away
            if(PvCaptureQueueFrame(lContext->mCamera,aFrame,frameDoneCB))
            {
                // or, if the driver queue is maxed-out, we put it in the extra-queue
                acquireLock(&lContext->mLock); 
                lContext->mTodo.push(aFrame);
                releaseLock(&lContext->mLock);    
            } 
        }
    }    
}

#ifdef _WINDOWS
unsigned long __stdcall writerThreadCB(void *aContext)
#else
void *writerThreadCB(void *aContext)
#endif
{
    tWriterContext* lContext = (tWriterContext*)aContext;
    tPvFrame*       lFrame;
    
    acquireLock(&lContext->mStream->mLock);

    // loop until we're done
    while(!lContext->mStream->mAbort)
    {
        while(!lContext->mStream->mDone.empty())
        { 
            lFrame = lContext->mStream->mDone.front();
            lContext->mStream->mDone.pop();
        
            releaseLock(&lContext->mStream->mLock);  
            
            doSaveFrame(lContext,lFrame);
            
            acquireLock(&lContext->mStream->mLock);
            
            // re-enqueue it
            if(!lContext->mStream->mAbort)
                if(PvCaptureQueueFrame(lContext->mStream->mCamera,lFrame,frameDoneCB))
                    lContext->mStream->mTodo.push(lFrame);                                   
        }
        
        releaseLock(&lContext->mStream->mLock);  
        
        Sleep(25);
        
        acquireLock(&lContext->mStream->mLock);
        
    }   
    
    releaseLock(&lContext->mStream->mLock); 
    
    return NULL;
}

#ifdef _WINDOWS
unsigned long __stdcall watcherThreadCB(void *aContext)
#else
void *watcherThreadCB(void *aContext)
#endif
{
    tStreamContext* lContext = (tStreamContext*)aContext;
    uint            lDoneSize,lTodoSize,lCount,lCReal;
      
    acquireLock(&lContext->mLock);

    // loop until we're done
    while(!lContext->mAbort)
    {
        lCount      = lContext->mCount;
        lCReal      = lContext->mCReal;
        lDoneSize   = lContext->mDone.size();
        lTodoSize   = lContext->mTodo.size();
        
        releaseLock(&lContext->mLock);  
        
        fprintf(stdout,"%06u|%06u %03u|%03u\r",lCount,lCReal,lDoneSize,lTodoSize);
        fflush(stdout);
        
        Sleep(100);
        
        acquireLock(&lContext->mLock);
        
    } 
    
    releaseLock(&lContext->mLock);  
    
    fprintf(stdout,"\n");
    
    return NULL;
}
 

///

bool openAndSetupCamera(tPvHandle* aHandle,ulong aCameraIP,float aFrameRate)
{
    if(PvCameraOpenByAddr(aCameraIP,ePvAccessMaster,aHandle) == ePvErrSuccess)
    {
        if((PvAttrEnumSet(*aHandle,"FrameStartTriggerMode","FixedRate") == ePvErrSuccess) &&
           (PvAttrFloat32Set(*aHandle,"FrameRate",aFrameRate) == ePvErrSuccess))
           return true;
        else
        {
            PvCameraClose(*aHandle);
            return false;
        }
    }
    else
        return false;
}

void closeCamera(tPvHandle* aHandle)
{
    // dequeue all the frame still queued (this will block until they all have been dequeued)
    PvCaptureQueueClear(*aHandle);
    // then close the camera
    PvCameraClose(*aHandle);
}

bool startCamera(tPvHandle* aHandle)
{
    return PvCommandRun(*aHandle,"AcquisitionStart") == ePvErrSuccess;   
}

bool stopCamera(tPvHandle* aHandle)
{
    return PvCommandRun(*aHandle,"AcquisitionStop") == ePvErrSuccess;   
}

bool allocateFrames(tStreamContext* aContext,uint aCount)
{
    tPvUint32 lSize = 0;

    if(PvAttrUint32Get(aContext->mCamera,"TotalBytesPerFrame",&lSize) == ePvErrSuccess)
    {
        aContext->mFrames.mArray = (tPvFrame*)malloc(sizeof(tPvFrame) * aCount);
        if(aContext->mFrames.mArray)
        {
            bool lFailed = false;
            
            memset(aContext->mFrames.mArray,0,sizeof(tPvFrame) * aCount);
            
            aContext->mFrames.mCount = aCount;
        
            for(uint i=0;i<aCount && !lFailed;i++)
            {
                aContext->mFrames.mArray[i].ImageBuffer = malloc(lSize); 
                if(!aContext->mFrames.mArray[i].ImageBuffer)
                    lFailed = true;
                else
                {                
                    aContext->mFrames.mArray[i].ImageBufferSize = lSize;
                    aContext->mFrames.mArray[i].Context[0] = aContext;
                }
            }
            
            if(lFailed)
            {
                for(uint i=0;i<aCount;i++)
                    if(aContext->mFrames.mArray[i].ImageBuffer)
                    {
                        free(aContext->mFrames.mArray[i].ImageBuffer);
                        
                        aContext->mFrames.mArray[i].ImageBuffer     = NULL;
                        aContext->mFrames.mArray[i].ImageBufferSize = 0;
                    } 
                    
                free(aContext->mFrames.mArray);
                aContext->mFrames.mArray = NULL;   
                aContext->mFrames.mCount = 0;  
            }
           
            return !lFailed;
        }
        else
            return false;
    }
    else
        return false;
}

void freeFrames(tStreamContext* aContext)
{
    if(aContext->mFrames.mArray)
    {
        for(uint i=0;i<aContext->mFrames.mCount;i++)
            if(aContext->mFrames.mArray[i].ImageBuffer)
            {
                free(aContext->mFrames.mArray[i].ImageBuffer);
                
                aContext->mFrames.mArray[i].ImageBuffer     = NULL;
                aContext->mFrames.mArray[i].ImageBufferSize = 0;
            } 
            
        free(aContext->mFrames.mArray);
        aContext->mFrames.mArray = NULL;
        aContext->mFrames.mCount = 0;     
    }
}

bool enqueueFrames(tStreamContext* aContext)
{    
    for(uint i=0;i<aContext->mFrames.mCount;i++)
        if(PvCaptureQueueFrame(aContext->mCamera,&aContext->mFrames.mArray[i],frameDoneCB))
            aContext->mTodo.push(&aContext->mFrames.mArray[i]);
   
    return true;   
}

void doStream(ulong aCameraIP,float aFrameRate,uint aCount,uint aQuality,uint aFrames,const char* aFileRoot,uint aWriters)
{
    tStreamContext  lContext;
    tWriterContext* lWriters;
    
    memset(&lContext.mFrames,0,sizeof(tFrameArray));
    
    lContext.mAbort      = false;
    lContext.mCount      = 0;
    lContext.mLimit      = aCount;
    lContext.mFileRoot   = aFileRoot;
    lContext.mQuality    = aQuality;
    
    initLock(&lContext.mLock);
    
    lWriters = (tWriterContext*)malloc(sizeof(tWriterContext) * aWriters);
    if(lWriters)
    {
        for(uint i=0;i<aWriters;i++)
        {
            lWriters[i].mBuffer[0] = '\0';
            lWriters[i].mStream = &lContext;
            JPEGSetup(&lWriters[i].mWriter);
        }
        
        // open & setup the camera
        if(openAndSetupCamera(&lContext.mCamera,aCameraIP,aFrameRate))
        {
            // allocate our frames
            if(allocateFrames(&lContext,aFrames))
            {
                PvCaptureStart(lContext.mCamera);
            
                // enqueue all frames
                if(enqueueFrames(&lContext))
                {
                    tThread lWatcher;
                
                    // spawn watcher thread
                    if(spawnThread(&lWatcher,watcherThreadCB,&lContext))
                    {                
                        // spawn the writers
                        for(uint i=0;i<aWriters;i++)
                            spawnThread(&lWriters[i].mThread,writerThreadCB,&lWriters[i]);
                    
                        // get camera stream
                        if(startCamera(&lContext.mCamera))
                        {
                            fprintf(stdout,"Camera is up&running ...\n");    
                        }
                        else
                            lContext.mAbort = true;
                    
                        // wait 4 it to end    
                        wait4Thread(&lWatcher);
                        
                        // and then for the writers
                        for(uint i=0;i<aWriters;i++)
                            wait4Thread(&lWriters[i].mThread);                       
                        
                        stopCamera(&lContext.mCamera);
                    }
                    else
                        fprintf(stderr,"Failed to spawn worker thread\n");
                }
                else
                    fprintf(stderr,"Failed to enqueue frames\n");    
                
                PvCaptureQueueClear(lContext.mCamera);
                PvCaptureEnd(lContext.mCamera);
                                
                // free the frames
                freeFrames(&lContext);
            }
            else
                fprintf(stderr,"Failed to setup the frames\n");
            
            closeCamera(&lContext.mCamera);
        } 
        else
            fprintf(stderr,"Failed to open the camera\n");        
        
        for(uint i=0;i<aWriters;i++)
            JPEGClear(&lWriters[i].mWriter);
       
        free(lWriters);
    }
    
    deleteLock(&lContext.mLock);
}

///

int main(int argc, char* argv[])
{
    if(argc == 8)
    {
        // initialise the PvAPI
        if(PvInitialize() == ePvErrSuccess)
        { 
            doStream(inet_addr(argv[1]),atof(argv[2]),atol(argv[3]),atol(argv[4]),atol(argv[5]),argv[7],atol(argv[6]));
        
            // uninitialise the API
            PvUnInitialize();        
        }
        else
            fprintf(stderr,"Failed to initialize the API\n");
    }
    else
        fprintf(stdout,"Usage: <IP@> <fps> <frame count> <quality (1-100)> <buffer count> <writer count> <filename root>\n");
        
    return 0;
}

