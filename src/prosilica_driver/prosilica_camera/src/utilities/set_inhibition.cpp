#include "prosilica/prosilica.h"
#include <cstdio>

int main(int argc, char** argv)
{
  if (argc < 2) {
    printf("Usage: %s <IP address>\n", argv[0]);
    return 1;
  }

  prosilica::init();

  try {
    // Open camera at specified IP address
    prosilica::Camera cam(argv[1]);
    // Load factory settings
    cam.setAttributeEnum("ConfigFileIndex", "Factory");
    cam.runCommand("ConfigFileLoad");
    // Write settings for signaling exposure
    cam.setAttributeEnum("SyncOut1Invert", "Off");
    cam.setAttributeEnum("SyncOut1Mode", "Exposing");
    // Save settings to config file 1
    cam.setAttributeEnum("ConfigFileIndex", "1");
    cam.runCommand("ConfigFileSave");
    // Always load config file 1 on power up
    cam.setAttributeEnum("ConfigFilePowerUp", "1");
  }
  catch (const prosilica::ProsilicaException& e) {
    printf("CONFIGURATION FAILED:\n%s\n", e.what());
    return 1;
  }
  printf("Configured camera successfully.\n");

  prosilica::fini();
  return 0;
}
