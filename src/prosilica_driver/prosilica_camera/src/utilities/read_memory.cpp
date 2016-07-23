#include "prosilica/prosilica.h"
#include <cstdio>
#include <cassert>

int main(int argc, char** argv)
{
  if (argc < 2) {
    printf("Usage: %s <IP address>\n", argv[0]);
    return 0;
  }

  prosilica::init();
  char buffer[prosilica::Camera::USER_MEMORY_SIZE] = {0};
  {
    prosilica::Camera cam(argv[1]);
    cam.readUserMemory(buffer, sizeof(buffer));
  }
  prosilica::fini();

  fwrite(buffer, 1, sizeof(buffer), stdout);

  return 0;
}
