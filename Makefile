#####################################################################
#
#  Name:         Makefile
#  Originally by:   Stefan Ritt
#  Modified by Alexandre XXX and Thomas Lindner
#  
#  Contents:     Makefile for Keithley picoammeter readout
#
#
#####################################################################
#
#--------------------------------------------------------------------
# The MIDASSYS should be defined prior the use of this Makefile
ifndef MIDASSYS
missmidas::
	@echo "...";
	@echo "Missing definition of environment variable 'MIDASSYS' !";
	@echo "...";
endif

#--------------------------------------------------------------------
# The following lines contain specific switches for different UNIX
# systems. Find the one which matches your OS and outcomment the 
# lines below.

#-----------------------------------------
# This is for Linux
ifeq ($(OSTYPE),Linux)
OSTYPE = linux
endif

ifeq ($(OSTYPE),linux)

OS_DIR = linux
OSFLAGS = -DOS_LINUX -DHAVE_LIBUSB -DHAVE_USB -Dextname
CFLAGS = -g -O2 -Wall
LIBS = -lusb -lutil -lnsl -lpthread -lz -lrt
endif

#-----------------------
# MacOSX/Darwin is just a funny Linux
#
ifeq ($(OSTYPE),Darwin)
OSTYPE = darwin
endif

ifeq ($(OSTYPE),darwin)
OS_DIR = darwin
FF = cc
OSFLAGS = -DOS_LINUX -DOS_DARWIN -DHAVE_STRLCPY -DAbsoftUNIXFortran -fPIC -Wno-unused-function
LIBS = -lpthread
SPECIFIC_OS_PRG = $(BIN_DIR)/mlxspeaker
NEED_STRLCPY=
NEED_RANLIB=1
NEED_SHLIB=
NEED_RPATH=

endif

#-------------------------------------------------------------------
# The following lines define directories. Adjust if necessary
#                 
DRV_DIR    = $(MIDASSYS)/drivers
MSCB_DIR   = $(MIDASSYS)/../mscb
INC_DIR    = $(MIDASSYS)/include
LIB_DIR    = $(MIDASSYS)/lib
SRC_DIR    = $(MIDASSYS)/src
#-------------------------------------------------------------------
# Frontend code name defaulted to frontend in this example.
# comment out the line and run your own frontend as follow:
# gmake UFE=my_frontend
#
UFE = scpico

####################################################################
# Lines below here should not be edited
####################################################################

# MIDAS library
LIB = $(LIB_DIR)/libmidas.a 

# compiler
CC = gcc
CXX = g++
CFLAGS += -g -I. -I$(INC_DIR) -I$(MIDASSYS)/../mxml -I$(DRV_DIR) -I$(DRV_DIR)/devices -I$(DRV_DIR)/class -I$(MSCB_DIR)/include 
LDFLAGS +=

all: $(UFE) 

$(UFE): $(UFE).cxx $(LIB) $(LIB_DIR)/mfe.o mscbrpc.o mscb.o mxml.o strlcpy.o musbstd.o $(UFE).cxx 
	$(CXX) $(CFLAGS) $(OSFLAGS) -o $(UFE) $(UFE).cxx mscbrpc.o mscb.o musbstd.o mxml.o strlcpy.o \
	$(LIB_DIR)/mfe.o $(LIB) $(LDFEFLAGS) $(LIBS)

mscbdev.o: $(DRV_DIR)/device/mscbdev.c
	$(CC) $(CFLAGS) $(OSFLAGS) -c -o $@ -c $<
musbstd.o: $(DRV_DIR)/usb/musbstd.c
	$(CC) $(CFLAGS) $(OSFLAGS) -c -o $@ -c $<
mscbrpc.o: $(MSCB_DIR)/src/mscbrpc.c
	$(CC) $(CFLAGS) $(OSFLAGS) -c -o $@ -c $<
mscb.o: $(MSCB_DIR)/src/mscb.c
	$(CC) $(CFLAGS) $(OSFLAGS) -c -o $@ -c $<
mxml.o: $(MIDASSYS)/../mxml/mxml.c
	$(CC) $(CFLAGS) $(OSFLAGS) -c -o $@ -c $<
strlcpy.o: $(MIDASSYS)/../mxml/strlcpy.c
	$(CC) $(CFLAGS) $(OSFLAGS) -c -o $@ -c $<

%.o: %.c experim.h
	$(CXX) $(USERFLAGS) $(CFLAGS) $(OSFLAGS) -o $@ -c $<

clean::
	rm -f *.o *~ \#*

#end file
