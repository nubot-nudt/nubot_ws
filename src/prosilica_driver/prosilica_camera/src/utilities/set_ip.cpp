#include <cstdio>
#include "prosilica/prosilica.h"
#include <arpa/inet.h>

void printSettings(const tPvCameraInfo& info, const tPvIpSettings& conf)
{
  printf("\t%s - %7s - Unique ID = %lu\n", info.SerialString, info.DisplayName, info.UniqueId);
  printf("\tMode:\t\t");
  if (conf.ConfigMode & ePvIpConfigPersistent)
    printf("FIXED\n");
  else if (conf.ConfigMode & ePvIpConfigDhcp)
    printf("DHCP&AutoIP\n");
  else if (conf.ConfigMode & ePvIpConfigAutoIp)
    printf("AutoIP\n");
  else
    printf("None\n");

  struct in_addr addr;
  addr.s_addr = conf.CurrentIpAddress;
  printf("\tAddress:\t%s\n", inet_ntoa(addr));
  addr.s_addr = conf.CurrentIpSubnet;
  printf("\tSubnet:\t\t%s\n", inet_ntoa(addr));
  addr.s_addr = conf.CurrentIpGateway;
  printf("\tGateway:\t%s\n", inet_ntoa(addr));
}

int main(int argc, char** argv)
{
  if (argc < 2) {
    printf("Usage: %s 10.68.0.20 [255.255.255.0] [0.0.0.0]\n", argv[0]);
    return 0;
  }

  char* netmask = "255.255.255.0";
  if (argc >= 3) {
    netmask = argv[2];
  }

  char* gateway = "0.0.0.0";
  if (argc >= 4) {
    netmask = argv[3];
  }

  prosilica::init();
  // Make sure we call prosilica::fini() on exit.
  boost::shared_ptr<void> guard(static_cast<void*>(0), boost::bind(prosilica::fini));

  // Check if camera IP is already set
  char* ip_address = argv[1];
  unsigned long IP = inet_addr(ip_address);
  tPvCameraInfo info;
  tPvIpSettings conf;
  tPvErr err = PvCameraInfoByAddr(IP, &info, &conf);
  if (!err) {
    printf("Camera found at requested IP address:\n");
    printSettings(info, conf);
  }
  else {
    printf("No camera found at %s, trying to change settings of a local camera...\n", ip_address);
    size_t num_cams = prosilica::numCameras();
    if (num_cams == 0) {
      printf("ERROR: No camera detected. Is it plugged in?\n");
      return 1;
    }
    if (num_cams == 2) {
      printf("ERROR: Multiple cameras (%u) found. Do you have more than one plugged in?\n", (unsigned)num_cams);
      return 1;
    }
    
    printf("Detected camera.\n");
    unsigned long uid = prosilica::getGuid(0);
    
    if (PvCameraInfo(uid, &info)) {
      printf("ERROR: could not retrieve camera info.\n");
      return 1;
    }
    if (PvCameraIpSettingsGet(uid, &conf)) {
      printf("ERROR: could not retrieve camera IP settings.\n");
      return 1;
    }
    printf("Original settings:\n");
    printSettings(info, conf);

    printf("Applying new settings...\n");
    conf.ConfigMode = ePvIpConfigPersistent;
    //conf.ConfigMode = ePvIpConfigDhcp;
    conf.CurrentIpAddress = conf.PersistentIpAddr = IP;
    conf.CurrentIpSubnet = conf.PersistentIpSubnet = inet_addr(netmask);
    conf.CurrentIpGateway = conf.PersistentIpGateway = inet_addr(gateway);
    if (PvCameraIpSettingsChange(uid, &conf)) {
      printf("ERROR: Failed to apply the new settings\n");
      return 1;
    }
    printf("New settings:\n");
    printSettings(info, conf);
  }

  return 0;
}
