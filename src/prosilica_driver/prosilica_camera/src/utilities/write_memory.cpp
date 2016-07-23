#include "prosilica/prosilica.h"
#include <cstdio>
#include <cassert>

int main(int argc, char** argv)
{
  if (argc < 3) {
    printf("Usage: %s <IP address> <file>\n", argv[0]);
    return 0;
  }

  char buffer[prosilica::Camera::USER_MEMORY_SIZE];
  FILE* file = fopen(argv[2], "rb");
  assert(file);
  fseek(file, 0, SEEK_END);
  size_t size = ftell(file);
  if (size > prosilica::Camera::USER_MEMORY_SIZE) {
    printf("File is too big!\n");
    return -1;
  }
  fseek(file, 0, SEEK_SET);
  size_t items_read = fread(buffer, 1, size, file);
  fclose(file);

  if (items_read != size)
  {
    printf("An error occurred reading the file\n");
    return -1;
  }
  
  prosilica::init();

  // Make sure cam's destructor is called before fini
  {
    prosilica::Camera cam(argv[1]);
    cam.writeUserMemory(buffer, size);
  }

  prosilica::fini();
  
  return 0;
}
