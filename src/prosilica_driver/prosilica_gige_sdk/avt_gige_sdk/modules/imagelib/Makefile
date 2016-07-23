# makefile of GigE SDK
#

include ../ARCH

NAME	= libImagelib.a

## some flags
FLAGS	= -fPIC -fexceptions -fno-stack-protector -I/usr/include $(DFLAGS)
#FLAGS	= -fPIC -fexceptions -I/usr/include $(DFLAGS)

## few locations
SRC_DIR = ./tiff-src
IST_DIR = ../../lib-pc/$(CPU)/$(CVER)

##
LIB	= -Bstatic -ltiff -Bdynamic $(SOLIB) 
CFLAGS	= $(OPT) $(FLAGS) -Wall -I$(INC_DIR) -D_REENTRANT $(EXTRA)

## OBJS
OBJS	= $(OBJ_DIR)/ImageLibTiff.o

##
$(OBJ_DIR)/%.o : $(SRC_DIR)/%.cpp
	$(CC) $(TARGET) $(CFLAGS) $(VERSION) -c $< -o $@

$(OBJ_DIR)/%.o : $(SRC_DIR)/%.c
	$(CC) $(TARGET) $(CFLAGS) $(VERSION) -c $< -o $@

clean: 
	rm $(OBJS)

lib : $(OBJS)
	rm -f $(NAME)
	$(AR) -r $(NAME) $(OBJS)

install:
	cp -f $(NAME) $(IST_DIR)
