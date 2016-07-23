/*
  ==============================================================================
  Copyright (C) 2006-2014 Allied Vision Technologies.  All Rights Reserved.
 
  Reproduction or disclosure of this file or its contents without the prior
  written consent of AVT is prohibited.
 ==============================================================================
 
  File:			    ImageLibTiff.cpp
 
  Project/lib:	    ImageLib
 
  Target:		    any     
 
  Description:	    implement a function that convert an existing tPvFrame to
                    a TIFF file using the TIFF library
 
  Notes:		    To recompile this, you will need to either have the tiff
                    library already installed on your system. On Windows, you
                    will have to grab it from http://www.libtiff.org and
                    integrate the needed files with this file.

 ==============================================================================
  dd/mon/yy  Author		Notes
 ------------------------------------------------------------------------------
  01/01/06	 JLV		Original.
  24/03/11   AKA		Updated to LibTIFF 3.9.4. Now builds with VC++ 2008, 2010.
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

//===== INCLUDE FILES =========================================================


#include <ImageLib.h>
#include <PvApi.h>
#include <assert.h>
#include <memory.h>
#include "tiffio.h"

//===== #DEFINES ==============================================================


#define ASSERT(x)	        assert(x)
#define ULONG_PADDING(x)    (((x+3) & ~3) - x)

//#if defined(_LINUX) || defined(_QNX) || defined(_OSX)
#define USHORT unsigned short
#define ULONG  unsigned long
#define UCHAR  unsigned char
//#endif

typedef struct {

    unsigned char LByte;
    unsigned char MByte;
    unsigned char UByte;

} Packed12BitsPixel_t;

//===== TYPE DEFINITIONS ======================================================

//===== FUNCTION PROTOTYPES ===================================================

void F_WriteStandardTags(TIFF* tiff, const tPvFrame* pFrame);
void F_WriteFrameBuffer(TIFF* tiff, const tPvFrame* pFrame);

//===== DATA (PUBLIC) =========================================================

//===== DATA (PRIVATE) ========================================================

//===== IMPLEMENTATION ========================================================

bool ImageWriteTiff(const char* filename, const tPvFrame* pFrame)
{
	TIFF* tiff = TIFFOpen(filename, "w");

	if (!tiff)
		return false;

	F_WriteStandardTags(tiff, pFrame);
	F_WriteFrameBuffer(tiff, pFrame);

	TIFFClose(tiff);

	return true;
}


void F_WriteStandardTags(TIFF* tiff, const tPvFrame* pFrame)
{
	USHORT				bitsPerSample;
	int					result;

	bitsPerSample = (pFrame->BitDepth > 8) ? 16 : 8;

	result = TIFFSetField(tiff, TIFFTAG_IMAGEWIDTH, (ULONG) pFrame->Width);
	ASSERT(result);
	result = TIFFSetField(tiff, TIFFTAG_IMAGELENGTH, (ULONG) pFrame->Height);
	ASSERT(result);
	result = TIFFSetField(tiff, TIFFTAG_COMPRESSION, (USHORT) 1); // No compression
	ASSERT(result);
	result = TIFFSetField(tiff, TIFFTAG_XRESOLUTION, (float) 1.0F);
	ASSERT(result);
	result = TIFFSetField(tiff, TIFFTAG_YRESOLUTION, (float) 1.0F);
	ASSERT(result);
	result = TIFFSetField(tiff, TIFFTAG_RESOLUTIONUNIT, (USHORT) 1); // No units
	ASSERT(result);
	result = TIFFSetField(tiff, TIFFTAG_PLANARCONFIG, (USHORT) 1);
	ASSERT(result);

	if ((pFrame->Format == ePvFmtMono8) || (pFrame->Format == ePvFmtMono16) || (pFrame->Format == ePvFmtMono12Packed))
	{
		// Monochrome

		result = TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, (USHORT) bitsPerSample);
		ASSERT(result);
		result = TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, (USHORT) 1); // Monochrome, black=0
		ASSERT(result);
	}
    else if ((pFrame->Format == ePvFmtRgba32) || (pFrame->Format == ePvFmtBgra32))
	{
		// Color

		result = TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, (USHORT) bitsPerSample,
							  (USHORT) bitsPerSample, (USHORT) bitsPerSample);
		ASSERT(result);
		result = TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL, (USHORT) 4);
		ASSERT(result);
		result = TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, (USHORT) 2); // Rgb color
		ASSERT(result);
	}
	else if ((pFrame->Format != ePvFmtYuv411) && (pFrame->Format != ePvFmtYuv422) && (pFrame->Format != ePvFmtYuv444))
	{
		// Color

		result = TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, (USHORT) bitsPerSample,
							  (USHORT) bitsPerSample, (USHORT) bitsPerSample);
		ASSERT(result);
		result = TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL, (USHORT) 3);
		ASSERT(result);
		result = TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, (USHORT) 2); // Rgb color
		ASSERT(result);
	}
	else
	{
		ASSERT(false);
	}

}


void F_WriteFrameBuffer(TIFF* tiff, const tPvFrame* pFrame)
{
	int					pixelSize;			// In bytes
	int					result;

	switch (pFrame->Format)
	{
	case ePvFmtMono8:		pixelSize = 1;		break;
    case ePvFmtMono12Packed:
	case ePvFmtMono16:		pixelSize = 2;		break;
	case ePvFmtRgb24:
    case ePvFmtBgr24:       pixelSize = 3;		break;
	case ePvFmtRgb48:		pixelSize = 6;		break;
	case ePvFmtBayer8:		pixelSize = 3;		break;
    case ePvFmtBayer12Packed:
	case ePvFmtBayer16:		pixelSize = 6;		break;
    case ePvFmtRgba32:
    case ePvFmtBgra32:      pixelSize = 4;		break;
    
	default:
		ASSERT(false);
		return;
	}

	const int destLineSize = pixelSize * pFrame->Width;			// In bytes

	UCHAR* const tmpBuffer     = new UCHAR[destLineSize * pFrame->Height];
    UCHAR* const tmpBufferEnd  = tmpBuffer + destLineSize * pFrame->Height;


    if(tmpBuffer)
    {

	    //
	    // Copy or manipulate data into temporary buffer.
	    //

	    if (pFrame->Format == ePvFmtMono8)
	    {
		    memcpy(tmpBuffer, pFrame->ImageBuffer, pFrame->ImageSize);
	    }
	    else if (pFrame->Format == ePvFmtMono16)
	    {
		    const USHORT*		pSrc = (USHORT*)pFrame->ImageBuffer;
		    const USHORT* const	pSrcEnd = pSrc + (pFrame->Width * pFrame->Height);
		    USHORT*				pDest = (USHORT*)tmpBuffer;
		    const ULONG			bitshift = 16 - pFrame->BitDepth;

		    while (pSrc < pSrcEnd)
			    *(pDest++) = *(pSrc++) << bitshift;
	    }
	    else if (pFrame->Format == ePvFmtMono12Packed)
        {
	        const Packed12BitsPixel_t*  pSrc = (const Packed12BitsPixel_t*)pFrame->ImageBuffer;
            const Packed12BitsPixel_t*  pSrcEnd = (const Packed12BitsPixel_t*)((unsigned char*)pFrame->ImageBuffer + pFrame->ImageSize);
	        USHORT*				        pDest = (USHORT*)tmpBuffer;
            USHORT*                     pDestEnd = (USHORT*)tmpBufferEnd;
	        const ULONG			        bitshift = 16 - pFrame->BitDepth;
            USHORT                      pixel;

	        while (pSrc < pSrcEnd)
	        {
		        for (ULONG i = 0; i < pFrame->Width && pSrc < pSrcEnd; i+=2)
                {
                    if(pDest < pDestEnd)
                    {
                        pixel = (USHORT)pSrc->LByte << 4;
                        pixel += ((USHORT)pSrc->MByte & 0xF0) >> 4;
                        *(pDest++) = pixel << bitshift;
                        
                        if(pDest < pDestEnd)
                        {
                            pixel = (USHORT)pSrc->UByte << 4;
                            pixel += ((USHORT)pSrc->MByte & 0x0F) >> 4;
                            *(pDest++) = pixel << bitshift;
                        }
                    }

                    pSrc++;
                }
	        }  
        }
	    else if (pFrame->Format == ePvFmtBayer8)
	    {
		    UCHAR* const		pDest = tmpBuffer;

		    PvUtilityColorInterpolate(pFrame, &pDest[0], &pDest[1], &pDest[2], 2, 0);
	    }
	    else if (pFrame->Format == ePvFmtBayer16)
	    {
		    USHORT*				pDest = (USHORT*)tmpBuffer;
		    const USHORT* const	pDestEnd = pDest + (pFrame->Width * pFrame->Height * 3);
		    const ULONG			bitshift = 16 - pFrame->BitDepth;

		    PvUtilityColorInterpolate(pFrame, &pDest[0], &pDest[1], &pDest[2], 2, 0);

		    while (pDest < pDestEnd)
			    *(pDest++) <<= bitshift;
	    }
        else if (pFrame->Format == ePvFmtBayer12Packed)
        {
            tPvFrame lFrame = *pFrame;

            lFrame.Format           = ePvFmtBayer16;
            lFrame.ImageBufferSize  = pFrame->Width * pFrame->Height * 2;
            lFrame.ImageBuffer      = new UCHAR[lFrame.ImageBufferSize];

            if(lFrame.ImageBuffer)
            {
	            const Packed12BitsPixel_t*  pSrc = (const Packed12BitsPixel_t*)pFrame->ImageBuffer;
                const Packed12BitsPixel_t*  pSrcEnd = (const Packed12BitsPixel_t*)((unsigned char*)pFrame->ImageBuffer + pFrame->ImageSize);
                USHORT*				        pDest = (USHORT*)lFrame.ImageBuffer;
                USHORT*                     pDestEnd = (USHORT*)lFrame.ImageBuffer + pFrame->Width * pFrame->Height;
                const ULONG			        bitshift = 16 - pFrame->BitDepth;
                USHORT                      pixel1,pixel2;

	            while (pSrc < pSrcEnd && pDest < pDestEnd)
	            {
		            for (ULONG i = 0; i < pFrame->Width && pSrc < pSrcEnd; i+=2)
                    {
                        pixel1 = (USHORT)pSrc->LByte << 4;
                        pixel1 += ((USHORT)pSrc->MByte & 0xF0) >> 4;
                        
                        pixel2 = (USHORT)pSrc->UByte << 4;
                        pixel2 += ((USHORT)pSrc->MByte & 0x0F) >> 4;

                        if(pDest < pDestEnd)
                        {
                            *(pDest++) = pixel1;
                            if(pDest < pDestEnd)
                                *(pDest++) = pixel2;
                        }

                        pSrc++;
                    }
	            }

	            pDest = (USHORT*)tmpBuffer;
	            pDestEnd = pDest + (pFrame->Width * pFrame->Height * 3);

	            PvUtilityColorInterpolate(&lFrame, &pDest[0], &pDest[1], &pDest[2], 2, 0);

	            while (pDest < pDestEnd)
		            *(pDest++) <<= bitshift;

                delete (UCHAR*)lFrame.ImageBuffer;
            }
        }
	    else if (pFrame->Format == ePvFmtRgb24)
	    {
		    memcpy(tmpBuffer, pFrame->ImageBuffer, pFrame->ImageSize);
	    }
	    else if (pFrame->Format == ePvFmtBgr24)
	    {
		    memcpy(tmpBuffer, pFrame->ImageBuffer, pFrame->ImageSize);
	    }
	    else if (pFrame->Format == ePvFmtRgba32)
	    {
		    memcpy(tmpBuffer, pFrame->ImageBuffer, pFrame->ImageSize);
	    }
	    else if (pFrame->Format == ePvFmtBgra32)
	    {
		    memcpy(tmpBuffer, pFrame->ImageBuffer, pFrame->ImageSize);
	    }
	    else if (pFrame->Format == ePvFmtRgb48)
	    {
		    const USHORT*		pSrc = (USHORT*)pFrame->ImageBuffer;
		    const USHORT* const	pSrcEnd = pSrc + (pFrame->Width * pFrame->Height * 3);
		    USHORT*				pDest = (USHORT*)tmpBuffer;
		    const ULONG			bitshift = 16 - pFrame->BitDepth;

		    while (pSrc < pSrcEnd)
			    *(pDest++) = *(pSrc++) << bitshift;
	    }


	    //
	    // Write temporary buffer into TIFF file.
	    //

	    UCHAR* pScanLine = tmpBuffer;

	    for (ULONG i = 0; i < pFrame->Height; i++)
	    {
		    result = TIFFWriteScanline(tiff, pScanLine, i, 0);
		    ASSERT(result);

		    pScanLine += destLineSize; 
	    }


	    delete [] tmpBuffer;

    }
}
