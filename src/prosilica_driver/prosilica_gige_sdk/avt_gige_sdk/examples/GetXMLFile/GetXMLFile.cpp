/*
  ==============================================================================
  Copyright (C) 2011 Allied Vision Technologies.  All Rights Reserved.
 
  Redistribution of this header file, in original or modified form, without
  prior written consent of AVT is prohibited.
 
 ==============================================================================
 
  GetXMLFile
 
  Cameras have an onboard Genicam compliant XML file. Applications not using
  PvAPI use this to access camera features. This example code saves this XML 
  file to a txt file.
      
      e.g. usage: GetXMLFile 169.254.23.10
 
  Camera IP address can be found with ListCameras example code, 
  or DeviceIPAddress attribute in SampleViewer.
 
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
#include <vector>
#include <string>

#ifdef _WINDOWS
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <Winsock2.h>
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
#include <arpa/inet.h>
#endif

#include <PvApi.h>
#include <PvRegIo.h> //register access



void DownloadXmlFile(tPvHandle Handle)
{
    std::vector<unsigned char>   url;
    std::vector<unsigned char>   xmlData;
    unsigned long                xmlDataLength;
    unsigned long                xmlDataAddr;
    std::string                  fileName;
    char*                        token;

    //
    // We parse the URL to get the file location and length.  We use the
    // filename too, for saving to disk.
    //

    // The URL, from the camera, describes the location of the XML file.
    url.resize(512);
    if (PvMemoryRead(Handle, 0x200, url.size(), &url[0]) != ePvErrSuccess)
    {
        printf("Error reading XML file from camera\n");
        return;
    }

    try
    {
        if ((token = strtok_s((char*)&url[0], ";", &context)) == 0)
            throw false;

        fileName = token;
        fileName = fileName.substr(6, fileName.size()-6);    // Strip "Local:"
        
        if ((token = strtok_s(NULL, ";", &context)) == 0)
            throw false;

        if ((xmlDataAddr = strtol(token, NULL, 16)) == 0)
            throw false;

        if ((token = strtok_s(NULL, ";", &context)) == 0)
            throw false;

        if ((xmlDataLength = strtol(token, NULL, 16)) == 0)
            throw false;
    }
    catch (bool)
    {
        printf("Error reading XML file from camera\n");
        return;
    }
               

    //
    // Download the XML file.  We need to break this into multiple pieces.
    //

    printf("Downloading XML file from camera...\n");

    // Memory read must be a multiple of 4.  Round up.
    xmlData.resize((xmlDataLength + 3) & ~3);

    // Download in 512 byte pieces.
    for (unsigned long offset = 0; offset < xmlData.size(); offset += 512)
    {
        unsigned long size = 512;

        // The last piece is smaller.
        if (size > xmlData.size() - offset)
            size = xmlData.size() - offset;

        if (PvMemoryRead(Handle, xmlDataAddr + offset,
                         size, &xmlData[offset]) != ePvErrSuccess)
        {
            printf("Error reading XML file from camera\n");
            return;
        }
    }


    //
    // Write the file.
    //
	FILE* lFile = NULL;

	#ifdef _WINDOWS
		fopen_s(&lFile, fileName.c_str(), "wb");
	#else
		lFile = fopen(fileName.c_str(), "wb");
	#endif

    if ((lFile) && (fwrite(&xmlData[0], 1, xmlDataLength, lFile) == xmlDataLength))
        printf("Done!\n");
    else
        printf("Error writing file to disk.\n");

    fclose(lFile);

}


int main(int argc, char* argv[])
{
    unsigned long        IpAddress;
    tPvHandle            Handle; 


    // We require the IP address as a command line argument.
    if ((argc != 2) || ((IpAddress = inet_addr(argv[1])) == 0))
    {
        printf("Camera IP address is missing or invalid.\n");
        printf("  usage: GetXMLFile <IP@>\n");
        return -1;
    }

    // Initialize PvAPI
    if (PvInitialize() != ePvErrSuccess)
    {
        printf("Failed to initialize PvAPI.\n");
        return -1;
    }

    // Open camera, by IP address.
    if (PvCameraOpenByAddr(IpAddress, ePvAccessMonitor, &Handle) != ePvErrSuccess)
    {
        printf("Failed to open the camera.  Wrong IP address?\n");
        return -1;
    }

    // Read and save the XML file.
    DownloadXmlFile(Handle);

    // Close the camera.
    PvCameraClose(Handle);

    // Stop PvAPI
    PvUnInitialize();

    return 0;
}

