/*
  ==============================================================================
  Copyright (C) 2010-2014 Allied Vision Technologies.  All Rights Reserved.
 
  This code may be used in part, or in whole for your application development.
 
 ==============================================================================
 
  Simple multi-platform lock C-wrapper.
 
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

#include "Lock.h"

#ifdef _WINDOWS

void initLock(tLock* aLock)
{
    InitializeCriticalSection(aLock);
}

void deleteLock(tLock* aLock)
{
    DeleteCriticalSection(aLock);
}

void acquireLock(tLock* aLock)
{
    EnterCriticalSection(aLock);
}

void releaseLock(tLock* aLock)
{
    LeaveCriticalSection(aLock);
}


#else

void initLock(tLock* aLock)
{
    pthread_mutexattr_t lAttr;

    pthread_mutexattr_init(&lAttr);
#ifdef _OSX
    pthread_mutexattr_settype(&lAttr,PTHREAD_MUTEX_RECURSIVE);
#else
	pthread_mutexattr_settype(&lAttr,PTHREAD_MUTEX_RECURSIVE_NP);
#endif
    pthread_mutex_init(aLock,&lAttr);
}

void deleteLock(tLock* aLock)
{
    pthread_mutex_destroy(aLock);
}

void acquireLock(tLock* aLock)
{
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
    pthread_mutex_lock(aLock); 
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL);
}

void releaseLock(tLock* aLock)
{
    pthread_mutex_unlock(aLock);
}

#endif
