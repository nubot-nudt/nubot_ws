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

#include <malloc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "Jpeg.h"
#include "jpeglib.h"

typedef struct {

    unsigned char red;
    unsigned char green;
    unsigned char blue;

} tRGB;

typedef struct {

    unsigned short red;
    unsigned short green;
    unsigned short blue;

} tRGB16;

typedef struct {

    unsigned char blue;
    unsigned char green;
    unsigned char red;
   
} tBGR;

typedef struct {

    unsigned char red;
    unsigned char green;
    unsigned char blue;
    unsigned char alpha;

} tRGBA;

typedef struct {

    unsigned char blue;
    unsigned char green;
    unsigned char red;
    unsigned char alpha;
   
} tBGRA;

typedef struct {

    unsigned char u;
    unsigned char y1;
    unsigned char y2;
    unsigned char v;
    unsigned char y3;
    unsigned char y4;

} tYUV411;

typedef struct {

    unsigned char u;
    unsigned char y1;
    unsigned char v;
    unsigned char y2;

} tYUV422;

typedef struct {

    unsigned char u;
    unsigned char y1;
    unsigned char v;
    unsigned char _pad1;
    unsigned char y2;
    unsigned char _pad2;

} tYUV444;

typedef struct {

    unsigned char lbyte;
    unsigned char mbyte;
    unsigned char ubyte;

} tPackedPixel;

#ifndef max
#define max(a,b) (a > b ? a : b)
#endif

#ifndef min
#define min(a,b) (a < b ? a : b)
#endif

// convert YUV to RGB
inline void YUV2RGB(int y,int u,int v,unsigned char& r,unsigned char& g,unsigned char& b)
{
   // u and v are +-0.5
   u -= 128;
   v -= 128;

   // Conversion (clamped to 0..255)
   r = min(max(0,(int)(y + 1.370705 * (float)v)),255);
   g = min(max(0,(int)(y - 0.698001 * (float)v - 0.337633 * (float)u)),255);
   b = min(max(0,(int)(y + 1.732446 * (float)u)),255);
}

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
bool JPEGSetup(tJPEGContext* pContext)
{
    memset(pContext,0,sizeof(tJPEGContext));
    
    pContext->Info = (struct jpeg_compress_struct*)malloc(sizeof(struct jpeg_compress_struct));  
    
    return pContext->Info != NULL;
}

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
void JPEGClear(tJPEGContext* pContext)
{
    if(pContext->Buffer)
    {
        free(pContext->Buffer);
        pContext->Buffer = NULL;   
    }   
    
    if(pContext->Info)
    {
        free(pContext->Info);
        pContext->Info = NULL;
    }
}

/*
 * Function:  JPEGAdapt()
 *
 * Purpose:   adapt the provided JPEG context for usage with the given ROI
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
bool JPEGAdapt(tJPEGContext* pContext,tPvImageFormat aFormat,unsigned int aWidth,unsigned int aHeight)
{
    bool lOutcome = true;

    switch(aFormat)
    {
        case ePvFmtMono8:
        case ePvFmtRgb24:
        {
            if(pContext->Buffer)
            {
                free(pContext->Buffer);
            
                pContext->Buffer = NULL;
                pContext->Length = 0;
            }
                    
            break;
        }
        case ePvFmtMono16:
        {
            if(!pContext->Buffer)
            {
                pContext->Length = aWidth;
                pContext->Buffer = (unsigned char*)malloc(pContext->Length);
               
                lOutcome = pContext->Buffer != NULL;
            }
            else
            if(aWidth > pContext->Length)
            {
                unsigned char* lBuffer = (unsigned char*)realloc(pContext->Buffer,aWidth);
                
                if(lBuffer)
                {
                    pContext->Buffer = lBuffer;
                    pContext->Length = aWidth;       
                }
                else
                    lOutcome = false;
            }            
        
            break;
        }
        case ePvFmtBayer16:
        case ePvFmtBayer12Packed:
        {
            unsigned long lLength = aWidth * aHeight * 4; 
        
            if(!pContext->Buffer)
            {
                pContext->Length = lLength;
                pContext->Buffer = (unsigned char*)malloc(pContext->Length);
                
                lOutcome = pContext->Buffer != NULL;
            }
            else          
            if(lLength > pContext->Length)
            {
                unsigned char* lBuffer = (unsigned char*)realloc(pContext->Buffer,lLength);
                
                if(lBuffer)
                {
                    pContext->Buffer = lBuffer;
                    pContext->Length = lLength;       
                }
                else
                    lOutcome = false;
            } 
                    
            break;
        }
        case ePvFmtMono12Packed:
        {
            unsigned long lLength = aWidth * aHeight; 
        
            if(!pContext->Buffer)
            {
                pContext->Length = lLength;
                pContext->Buffer = (unsigned char*)malloc(pContext->Length);
                
                lOutcome = pContext->Buffer != NULL;
            }
            else          
            if(lLength > pContext->Length)
            {
                unsigned char* lBuffer = (unsigned char*)realloc(pContext->Buffer,lLength);
                
                if(lBuffer)
                {
                    pContext->Buffer = lBuffer;
                    pContext->Length = lLength;       
                }
                else
                    lOutcome = false;
            } 
                    
            break;
        }           
        case ePvFmtBgr24:
        case ePvFmtRgba32:
        case ePvFmtBgra32:
        case ePvFmtRgb48:
        {
            unsigned long lLength = aWidth * 3; 
        
            if(!pContext->Buffer)
            {
                pContext->Length = lLength;
                pContext->Buffer = (unsigned char*)malloc(pContext->Length);
                
                lOutcome = pContext->Buffer != NULL;
            }
            else          
            if(lLength > pContext->Length)
            {
                unsigned char* lBuffer = (unsigned char*)realloc(pContext->Buffer,lLength);
                
                if(lBuffer)
                {
                    pContext->Buffer = lBuffer;
                    pContext->Length = lLength;       
                }
                else
                    lOutcome = false;
            } 
                    
            break;
        }        
        default:
        {
            if(!pContext->Buffer)
            {
                pContext->Length = aWidth * aHeight * 3;
                pContext->Buffer = (unsigned char*)malloc(pContext->Length);
                
                lOutcome = pContext->Buffer != NULL;
            }
            else
            {
                unsigned int lLength = aWidth * aHeight * 3;
                
                if(lLength > pContext->Length)
                {
                    unsigned char* lBuffer = (unsigned char*)realloc(pContext->Buffer,lLength);
                    
                    if(lBuffer)
                    {
                        pContext->Buffer = lBuffer;
                        pContext->Length = lLength;       
                    }
                    else
                        lOutcome = false;
                }
            }         
        
            break;
        }        
    }
    
    return lOutcome;   
}

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
 * 
 * Results: false if failed
 */
bool JPEGWrite(tJPEGContext* pContext,const tPvFrame* pFrame,const char* pFilePath,unsigned char aQuality)
{
    FILE* lFile;    
    
    if((lFile = fopen(pFilePath,"wb")))
    {
        JSAMPROW                lRow;
        struct jpeg_error_mgr   jerr;
        
        pContext->Info->err = jpeg_std_error(&jerr);
        jpeg_create_compress(pContext->Info);
        jpeg_stdio_dest(pContext->Info,lFile);
        
        pContext->Info->image_width      = pFrame->Width;      
        pContext->Info->image_height     = pFrame->Height;    
        
        if(pFrame->Format == ePvFmtMono8 || pFrame->Format == ePvFmtMono16 || pFrame->Format == ePvFmtMono12Packed)
        {
            pContext->Info->input_components = 1;
            pContext->Info->in_color_space   = JCS_GRAYSCALE;      
        }             
        else
        {
            pContext->Info->input_components = 3;
            pContext->Info->in_color_space   = JCS_RGB;  
        }
        
        jpeg_set_defaults(pContext->Info);
        jpeg_set_quality(pContext->Info,aQuality,true);      //set the quality [0..100]
        jpeg_start_compress(pContext->Info,true);                  
        
        switch(pFrame->Format)
        {
            case ePvFmtMono8:
            
            {
                const unsigned char* lBuffer = (const unsigned char*)pFrame->ImageBuffer;
            
                for(unsigned int j=0;j<pFrame->Height;j++)
                {
                    lRow = (JSAMPROW)&lBuffer[j * pFrame->Width];
                
                    jpeg_write_scanlines(pContext->Info,&lRow,1);    
                }        
            
                break;
            }
            case ePvFmtMono16:
            {
                const unsigned char     lBitshift   = (unsigned char)pFrame->BitDepth - 8;
                const unsigned short*   lBuffer     = (const unsigned short*)pFrame->ImageBuffer;
                 
                lRow = (JSAMPROW)pContext->Buffer;
                            
                for(unsigned int j=0;j<pFrame->Height;j++)
                {               
                    for(unsigned int i=0;i<pFrame->Width;i++)
                    #ifdef _ppc
                        pContext->Buffer[i] = lBuffer[j * pFrame->Width + i] << lBitshift;
                    #else
                        pContext->Buffer[i] = lBuffer[j * pFrame->Width + i] >> lBitshift;
                    #endif
                                       
                    jpeg_write_scanlines(pContext->Info,&lRow,1);   
                }   
                          
                break;                    
            }
            case ePvFmtMono12Packed:
            {
                // tPackedPixel
                const unsigned char lBitshift   = (unsigned char)pFrame->BitDepth - 8;
                const tPackedPixel* lSource     = (const tPackedPixel*)pFrame->ImageBuffer;
                unsigned char*      lTarget     = (unsigned char*)pContext->Buffer;
                unsigned short      lPixel;
                unsigned long       lOffset     = 0;
                unsigned long       lCount      = pFrame->ImageSize / sizeof(tPackedPixel);
                
                for(unsigned long i = 0;i<lCount;i++)
                {
                    lPixel = (unsigned short)lSource[i].lbyte << 4;
                    lPixel += ((unsigned short)lSource[i].mbyte & 0xF0) >> 4;
                    lTarget[lOffset++] = lPixel >> lBitshift;
                    
                    lPixel = (unsigned short)lSource[i].ubyte << 4;
                    lPixel += ((unsigned short)lSource[i].mbyte & 0x0F) >> 4;                    
                    lTarget[lOffset++] = lPixel >> lBitshift;
                }               
                
                // send to JPEG lib
                for(unsigned int j=0;j<pFrame->Height;j++)
                {
                    lRow = (JSAMPROW)&lTarget[j * pFrame->Width];
                    jpeg_write_scanlines(pContext->Info,&lRow,1);    
                }                 
            
                break;
            }
            case ePvFmtBayer8:
            {
                PvUtilityColorInterpolate(pFrame,&pContext->Buffer[0],&pContext->Buffer[1],&pContext->Buffer[2],2,0);
                  
                for(unsigned int j=0;j<pFrame->Height;j++)
                {
                    lRow = (JSAMPROW)&pContext->Buffer[j * pFrame->Width * 3];
                
                    jpeg_write_scanlines(pContext->Info,&lRow,1);    
                }     
                
                break;                             
            }
            case ePvFmtBayer16:
            {
                const unsigned char     lBitshift   = (unsigned char)pFrame->BitDepth - 8;
                const unsigned short*   lImageData  = (const unsigned short*)pFrame->ImageBuffer;
                unsigned char*          lBayerData  = pContext->Buffer;
                unsigned char*          lColorData  = pContext->Buffer + (pFrame->Width * pFrame->Height);
                unsigned long           lOffset;
                tPvFrame                lFrame      = *pFrame;
                
                lFrame.Format           = ePvFmtBayer8;
                lFrame.ImageBuffer      = lBayerData;
                lFrame.ImageBufferSize  = lFrame.Width * lFrame.Height;
                
                // convert to 8bit
                for(unsigned int j=0;j<pFrame->Height;j++)
                    for(unsigned int i=0;i<pFrame->Width;i++)
                    {
                        lOffset = j * pFrame->Width + i;
                    
                        #ifdef _ppc
                            lBayerData[lOffset] = lImageData[lOffset] << lBitshift;
                        #else
                            lBayerData[lOffset] = lImageData[lOffset] >> lBitshift;
                        #endif
                    }                                            
            
                // interpolate
                PvUtilityColorInterpolate(&lFrame,&lColorData[0],&lColorData[1],&lColorData[2],2,0);
                  
                // send to JPEG lib
                for(unsigned int j=0;j<pFrame->Height;j++)
                {
                    lRow = (JSAMPROW)&lColorData[j * pFrame->Width * 3];
                    jpeg_write_scanlines(pContext->Info,&lRow,1);    
                }                
            
                break;
            }  
            case ePvFmtBayer12Packed:
            {
                const tPackedPixel* lSource     = (const tPackedPixel*)pFrame->ImageBuffer;
                unsigned char*      lTarget     = (unsigned char*)pContext->Buffer;
                unsigned short      lPixel;
                unsigned long       lOffset     = 0;
                unsigned long       lCount      = pFrame->ImageSize / sizeof(tPackedPixel);
                const unsigned char lBitshift   = (unsigned char)pFrame->BitDepth - 8;
                unsigned char*      lColorData  = pContext->Buffer + (pFrame->Width * pFrame->Height);
                tPvFrame            lFrame      = *pFrame;
                
                lFrame.Format           = ePvFmtBayer8;
                lFrame.ImageBuffer      = lTarget;
                lFrame.ImageBufferSize  = lFrame.Width * lFrame.Height;
                
                for(unsigned long i = 0;i<lCount;i++)
                {
                    lPixel = (unsigned short)lSource[i].lbyte << 4;
                    lPixel += ((unsigned short)lSource[i].mbyte & 0xF0) >> 4;
                    lTarget[lOffset++] = lPixel >> lBitshift;
                    
                    lPixel = (unsigned short)lSource[i].ubyte << 4;
                    lPixel += ((unsigned short)lSource[i].mbyte & 0x0F) >> 4;                    
                    lTarget[lOffset++] = lPixel >> lBitshift;
                }
                                            
                // interpolate
                PvUtilityColorInterpolate(&lFrame,&lColorData[0],&lColorData[1],&lColorData[2],2,0);
                  
                // send to JPEG lib
                for(unsigned int j=0;j<pFrame->Height;j++)
                {
                    lRow = (JSAMPROW)&lColorData[j * pFrame->Width * 3];
                    jpeg_write_scanlines(pContext->Info,&lRow,1);    
                }
                            
                break;
            }
            case ePvFmtRgb24:
            {
                const unsigned char* lBuffer = (const unsigned char*)pFrame->ImageBuffer;
            
                for(unsigned int j=0;j<pFrame->Height;j++)
                {
                    lRow = (JSAMPROW)&lBuffer[j * pFrame->Width * 3];
                    jpeg_write_scanlines(pContext->Info,&lRow,1);    
                }                 
            
                break;
            } 
            case ePvFmtBgr24:    
            {
                const tBGR*     lSource = (const tBGR*)pFrame->ImageBuffer;
                tRGB*           lTarget = (tRGB*)pContext->Buffer;
                unsigned long   lOffset;
                
                lRow = (JSAMPROW)lTarget;
                
                for(unsigned int j=0;j<pFrame->Height;j++)
                {
                    for(unsigned int i=0;i<pFrame->Width;i++)
                    {
                        lOffset = j * pFrame->Width + i;
                    
                        lTarget[i].red   = lSource[lOffset].red;
                        lTarget[i].green = lSource[lOffset].green;
                        lTarget[i].blue  = lSource[lOffset].blue;
                    }
                    
                    jpeg_write_scanlines(pContext->Info,&lRow,1);    
                }                  
            
                break;
            }     
            case ePvFmtRgba32:
            {
                const tRGBA*    lSource = (const tRGBA*)pFrame->ImageBuffer;
                tRGB*           lTarget = (tRGB*)pContext->Buffer;
                unsigned long   lOffset;
                
                lRow = (JSAMPROW)lTarget;
                
                for(unsigned int j=0;j<pFrame->Height;j++)
                {
                    for(unsigned int i=0;i<pFrame->Width;i++)
                    {
                        lOffset = j * pFrame->Width + i;
                    
                        lTarget[i].red   = lSource[lOffset].red;
                        lTarget[i].green = lSource[lOffset].green;
                        lTarget[i].blue  = lSource[lOffset].blue;
                    }
                    
                    jpeg_write_scanlines(pContext->Info,&lRow,1);    
                }                
            
                break;
            }
            case ePvFmtBgra32:
            {
                const tBGRA*    lSource = (const tBGRA*)pFrame->ImageBuffer;
                tRGB*           lTarget = (tRGB*)pContext->Buffer;
                unsigned long   lOffset;
                
                lRow = (JSAMPROW)lTarget;
                
                for(unsigned int j=0;j<pFrame->Height;j++)
                {
                    for(unsigned int i=0;i<pFrame->Width;i++)
                    {
                        lOffset = j * pFrame->Width + i;
                    
                        lTarget[i].red   = lSource[lOffset].red;
                        lTarget[i].green = lSource[lOffset].green;
                        lTarget[i].blue  = lSource[lOffset].blue;
                    }
                    
                    jpeg_write_scanlines(pContext->Info,&lRow,1);    
                }                
            
                break;            
            }
            case ePvFmtRgb48:
            {
                const tRGB16*       lSource   = (const tRGB16*)pFrame->ImageBuffer;
                tRGB*               lTarget   = (tRGB*)pContext->Buffer;
                unsigned long       lOffset;
                const unsigned char lBitshift = (unsigned char)pFrame->BitDepth - 8;
                
                lRow = (JSAMPROW)lTarget;
                
                for(unsigned int j=0;j<pFrame->Height;j++)
                {
                    for(unsigned int i=0;i<pFrame->Width;i++)
                    {
                        lOffset = j * pFrame->Width + i;
                    
                        #ifdef _ppc
                        lTarget[i].red   = lSource[lOffset].red << lBitshift;
                        lTarget[i].green = lSource[lOffset].green << lBitshift;
                        lTarget[i].blue  = lSource[lOffset].blue << lBitshift;
                        #else
                        lTarget[i].red   = lSource[lOffset].red >> lBitshift;
                        lTarget[i].green = lSource[lOffset].green >> lBitshift;
                        lTarget[i].blue  = lSource[lOffset].blue >> lBitshift;                        
                        #endif
                    }
                    
                    jpeg_write_scanlines(pContext->Info,&lRow,1);    
                }  
            
                break;
            }
            case ePvFmtYuv411:
            {
                const tYUV411* lSource = (const tYUV411*)pFrame->ImageBuffer;
                tRGB*          lTarget = (tRGB*)pContext->Buffer;
                unsigned long  lOffset;
                unsigned long  k = 0;
                
                lRow = (JSAMPROW)lTarget;
                
                for(unsigned int j=0;j<pFrame->Height;j++)
                    for(unsigned int i=0;i<pFrame->Width;i+=4,k++)
                    {
                        lOffset = j * pFrame->Width + i;
                    
                        YUV2RGB(lSource[k].y1,lSource[k].u,lSource[k].v,lTarget[lOffset].red,lTarget[lOffset].green,lTarget[lOffset].blue);
                        YUV2RGB(lSource[k].y2,lSource[k].u,lSource[k].v,lTarget[lOffset+1].red,lTarget[lOffset+1].green,lTarget[lOffset+1].blue);
                        YUV2RGB(lSource[k].y3,lSource[k].u,lSource[k].v,lTarget[lOffset+2].red,lTarget[lOffset+2].green,lTarget[lOffset+2].blue);
                        YUV2RGB(lSource[k].y4,lSource[k].u,lSource[k].v,lTarget[lOffset+3].red,lTarget[lOffset+3].green,lTarget[lOffset+3].blue);   
                    }                                            
            
                 // send to JPEG lib
                for(unsigned int j=0;j<pFrame->Height;j++)
                {
                    lRow = (JSAMPROW)&pContext->Buffer[j * pFrame->Width * 3];
                    jpeg_write_scanlines(pContext->Info,&lRow,1);    
                }                              
            
                break;
            }
            case ePvFmtYuv422:
            {
                const tYUV422* lSource = (const tYUV422*)pFrame->ImageBuffer;
                tRGB*          lTarget = (tRGB*)pContext->Buffer;
                unsigned long  lOffset;
                unsigned long  k = 0;
                
                lRow = (JSAMPROW)lTarget;
                
                for(unsigned int j=0;j<pFrame->Height;j++)
                    for(unsigned int i=0;i<pFrame->Width;i+=2,k++)
                    {
                        lOffset = j * pFrame->Width + i;
                    
                        YUV2RGB(lSource[k].y1,lSource[k].u,lSource[k].v,lTarget[lOffset].red,lTarget[lOffset].green,lTarget[lOffset].blue);
                        YUV2RGB(lSource[k].y2,lSource[k].u,lSource[k].v,lTarget[lOffset+1].red,lTarget[lOffset+1].green,lTarget[lOffset+1].blue);
                    }                                            
            
                 // send to JPEG lib
                for(unsigned int j=0;j<pFrame->Height;j++)
                {
                    lRow = (JSAMPROW)&pContext->Buffer[j * pFrame->Width * 3];
                    jpeg_write_scanlines(pContext->Info,&lRow,1);    
                }                
            
                break;
            }
            case ePvFmtYuv444:
            {
                const tYUV444* lSource = (const tYUV444*)pFrame->ImageBuffer;
                tRGB*          lTarget = (tRGB*)pContext->Buffer;
                unsigned long  lOffset;
                unsigned long  k = 0;
                
                lRow = (JSAMPROW)lTarget;
                
                for(unsigned int j=0;j<pFrame->Height;j++)
                    for(unsigned int i=0;i<pFrame->Width;i+=2,k++)
                    {
                        lOffset = j * pFrame->Width + i;
                    
                        YUV2RGB(lSource[k].y1,lSource[k].u,lSource[k].v,lTarget[lOffset].red,lTarget[lOffset].green,lTarget[lOffset].blue);
                        YUV2RGB(lSource[k].y2,lSource[k].u,lSource[k].v,lTarget[lOffset+1].red,lTarget[lOffset+1].green,lTarget[lOffset+1].blue);
                    }                                            
            
                 // send to JPEG lib
                for(unsigned int j=0;j<pFrame->Height;j++)
                {
                    lRow = (JSAMPROW)&pContext->Buffer[j * pFrame->Width * 3];
                    jpeg_write_scanlines(pContext->Info,&lRow,1);    
                }                
            
                break;
            }            
            default:
                break;
        }
   
        jpeg_finish_compress(pContext->Info);
    
        fclose(lFile);
        
        return true;
    }
    else
        return false;
}
