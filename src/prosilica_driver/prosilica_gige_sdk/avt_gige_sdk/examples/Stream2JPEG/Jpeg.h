/*
  ==============================================================================
  Copyright (C) 2010-2014 Allied Vision Technologies.  All Rights Reserved.
 
  This code may be used in part, or in whole for your application development.
 
 ==============================================================================
 
  This sample code, provides a frame to jpeg file function (based on IJG's library)
 
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

#include <PvApi.h>

struct jpeg_compress_struct;

typedef struct {

    unsigned char*               Buffer;
    unsigned int                 Length;
    struct jpeg_compress_struct* Info;

} tJPEGContext;

/*
 * Function:  JPEGSetup()
 *
 * Purpose:   Setup the provided JPEG context for usage
 *
 * Arguments:
 *
 * [ IN] tJPEGContext* pContext, context
 *
 * Results: false if failed
 *
 */
bool JPEGSetup(tJPEGContext* pContext);

/*
 * Function:  JPEGClear()
 *
 * Purpose:   clear the provided JPEG context after usage
 *
 * Arguments:
 *
 * [ IN] tJPEGContext* pContext, context
 *
 */
void JPEGClear(tJPEGContext* pContext);

/*
 * Function:  JPEGAdapt()
 *
 * Purpose:   adapt the provided JPEG context for usage with the given ROI & pixel format
 *
 * Arguments:
 *
 * [ IN] tJPEGContext* pContext,    context
 * [ IN] tPvImageFormat aFormat,    pixel format
 * [ IN] unsigned int aWidth,       max width of the frame
 * [ IN] unsigned int aHeight,      max height of the frame
 * 
 * Results: false if failed
 */
bool JPEGAdapt(tJPEGContext* pContext,tPvImageFormat aFormat,unsigned int aWidth,unsigned int aHeight);

/*
 * Function:  JPEGWrite()
 *
 * Purpose:   write a frame into a file
 *
 * Arguments:
 *
 * [ IN] tJPEGContext* pContext,    context
 * [ IN] const tPvFrame* pFrame,    frame
 * [ IN] const char* pFilePath,     path&name of the file
 * [ IN] unsigned char aQuality,    quality [0..100]
 * 
 * Results: false if failed
 */
bool JPEGWrite(tJPEGContext* pContext,const tPvFrame* pFrame,const char* pFilePath,unsigned char aQuality = 75);

