# makefile of GigE SDK Sample code
#

include ../ARCH

## Executable
EXE	= Stream2JPEG
    
clean: 
	rm $(EXE)

sample : $(EXE).cpp Jpeg.h Jpeg.c Lock.h Lock.c
	$(CC) $(RPATH) $(TARGET) $(CFLAGS) $(EXE).cpp Jpeg.c Lock.c Thread.c -o $(EXE) $(OBJ) $(SOLIB) $(PVLIB) $(LJPEG)

sample-static : $(EXE).cpp
	$(CC) $(RPATH) $(TARGET) $(CFLAGS) $(EXE).cpp Jpeg.c Lock.c Thread.c $(SALIB) -o $(EXE) $(OBJ) $(SOLIB) $(LJPEG)
	
install:
	cp -f $(EXE) $(BIN_DIR)
