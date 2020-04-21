#
# Makefile for MIP SDK Example for the GX3_35_TEST (linux version)
#
# Written by: Greggory P. Carpenter
#
# Copyright: LORD MicroStrain 2014
#
# This Makefile configured to work under GNU Make in linux with the gcc 
# compiler, but should be compatible with ANY C compiler on a system which 
# supports GNU Make or compatible Make alternative given appropriate 
# configuration. 
#
#
# THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING            
# CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER          
# FOR THEM TO SAVE TIME. AS A RESULT, MICROSTRAIN SHALL NOT BE HELD LIABLE     
# FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY        
# CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY      
# CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH      
# THEIR PRODUCTS.
#
##############################################################################  

#Name of project executable binary
PROJNAME = GX3_35_TEST

#Name of Project Main source file
PROJ_SRC = GX3_35_Test.c

#Relative paths to include headers 
INCLUDE_DIRECTORIES = ../../../../Utilities/Include \
                      ../../../../Library/Include ./ \

#Realtive path to Utility source files
UTIL_DIR = ../../../../Utilities/Source

#Relative path to Library source files
LIB_DIR = ../../../../Library/Source

#Generate compiler inlcude flags from specified include directories
INCS += $(addprefix -I,$(INCLUDE_DIRECTORIES))

#Specify Library source files
LIB_SRCS := mip_sdk_system.c mip_sdk_base.c mip_sdk_gps.c mip.c \
            mip_sdk_3dm.c ring_buffer.c mip_sdk_inteface.c mip_sdk_filter.c \
            mip_sdk_ahrs.c

#Generate object file names from specified Library source files
LIB_OBJS := $(LIB_SRCS:.c=.o)

#Local source files
LOCAL_SRCS := mip_sdk_user_functions.c 

#Generate object file names from local source files
LOCAL_OBJS := $(LOCAL_SRCS:.c=.o)

#Utility source files
UTIL_SRCS := byteswap_utilities.c 

#Generate object file names from specified Utility source files
UTIL_OBJS := $(UTIL_SRCS:.c=.o)

#C compiler used for compiling project
CC=gcc

#Compile flags for Project binary
PROJ_CFLAGS = -w

#Compile flags for object files
CFLAGS = -c -w

#To ENABLE Serial Port communication, comment out this line
CFLAGS += -DUSE_USB_CONNECTION

#linking flags for linking compiler
LFLAGS = -lrt

#Remove command used to remove unwanted object files and project binary
#at `clean' command
RM = rm #-i 

#Make rules for the Project executable
$(PROJNAME): $(LIB_OBJS) $(UTIL_OBJS) $(LOCAL_OBJS) $(PROJ_SRC)
	$(CC) $(INCS) $(PROJ_CFLAGS) -o $(PROJNAME) $(PROJ_SRC) \
	$(LIB_OBJS) $(UTIL_OBJS) $(LOCAL_OBJS) $(LFLAGS) 

#Make rules for the Utility source files
$(UTIL_OBJS):
	$(CC) $(INCS) $(CFLAGS) $(UTIL_DIR)/$*.c $(LFLAGS) 

#Make rules for the Library source files
$(LIB_OBJS): $(UTIL_OBJS) 
	$(CC) $(INCS) $(CFLAGS) $(LIB_DIR)/$*.c $(LFLAGS) 

#Make rules for the local source files
$(LOCAL_OBJS): $(UTIL_OBJS) 
	$(CC) $(INCS) $(CFLAGS) $*.c $(LFLAGS) 

#Remove all object files
clean:
	$(foreach filename, $(wildcard *.o), $(RM) $(filename);)

#Remove all object files and the project executable
clean_all:
	$(foreach filename, $(wildcard *.o $(PROJNAME)), $(RM) $(filename);)

#Object file dependencies below, each represents the location of the source
#code for the specified object file as well as all included headers in that 
#source.  (NOTE: Non-system headers ONLY are included below, host system should
#be configured to include all system headers automatically as the paths to 
#header files may vary by system.)  If the file structure referred to below is
#changed these paths will need to be updated
mip_sdk_user_functions.o: mip_sdk_user_functions.c \
 mip_sdk_user_functions.h ../../../../Library/Include/mip.h \
 ../../../../Library/Include/mip_types.h 

byteswap_utilities.o: ../../../../Utilities/Source/byteswap_utilities.c \
 ../../../../Utilities/Include/byteswap_utilities.h \
 ../../../../Library/Include/mip_types.h

mip_sdk_system.o: ../../../../Library/Source/mip_sdk_system.c \
 ../../../../Library/Include/mip_sdk_system.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/mip_types.h \
 ../../../../Library/Include/mip_sdk_interface.h \
 ../../../../Library/Include/mip_sdk_config.h mip_sdk_user_functions.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/ring_buffer.h \
 ../../../../Utilities/Include/byteswap_utilities.h \
 ../../../../Library/Include/mip_types.h

mip_sdk_base.o: ../../../../Library/Source/mip_sdk_base.c \
 ../../../../Library/Include/mip_sdk_base.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/mip_types.h \
 ../../../../Library/Include/mip_sdk_interface.h \
 ../../../../Library/Include/mip_sdk_config.h mip_sdk_user_functions.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/ring_buffer.h \
 ../../../../Utilities/Include/byteswap_utilities.h \
 ../../../../Library/Include/mip_types.h

mip_sdk_gps.o: ../../../../Library/Source/mip_sdk_gps.c \
 ../../../../Library/Include/mip_sdk_gps.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/mip_types.h \
 ../../../../Library/Include/mip_sdk_interface.h \
 ../../../../Library/Include/mip_sdk_config.h mip_sdk_user_functions.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/ring_buffer.h \
 ../../../../Utilities/Include/byteswap_utilities.h \
 ../../../../Library/Include/mip_types.h

mip.o: ../../../../Library/Source/mip.c ../../../../Library/Include/mip.h \
 ../../../../Library/Include/mip_types.h

mip_sdk_3dm.o: ../../../../Library/Source/mip_sdk_3dm.c \
 ../../../../Library/Include/mip_sdk_3dm.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/mip_types.h \
 ../../../../Library/Include/mip_sdk_interface.h \
 ../../../../Library/Include/mip_sdk_config.h mip_sdk_user_functions.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/ring_buffer.h \
 ../../../../Utilities/Include/byteswap_utilities.h \
 ../../../../Library/Include/mip_types.h \
 ../../../../Library/Include/mip_sdk_gps.h \
 ../../../../Library/Include/mip_sdk_ahrs.h

ring_buffer.o: ../../../../Library/Source/ring_buffer.c \
 ../../../../Library/Include/ring_buffer.h \
 ../../../../Library/Include/mip_types.h 

mip_sdk_inteface.o: ../../../../Library/Source/mip_sdk_inteface.c \
 ../../../../Library/Include/mip_sdk_interface.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/mip_types.h \
 ../../../../Library/Include/mip_sdk_config.h mip_sdk_user_functions.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/ring_buffer.h \
 ../../../../Utilities/Include/byteswap_utilities.h \
 ../../../../Library/Include/mip_types.h

mip_sdk_filter.o: ../../../../Library/Source/mip_sdk_filter.c \
 ../../../../Library/Include/mip_sdk_filter.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/mip_types.h \
 ../../../../Library/Include/mip_sdk_interface.h \
 ../../../../Library/Include/mip_sdk_config.h mip_sdk_user_functions.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/ring_buffer.h \
 ../../../../Utilities/Include/byteswap_utilities.h \
 ../../../../Library/Include/mip_types.h \
 ../../../../Library/Include/mip_sdk_system.h

mip_sdk_ahrs.o: ../../../../Library/Source/mip_sdk_ahrs.c \
 ../../../../Library/Include/mip_sdk_ahrs.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/mip_types.h \
 ../../../../Library/Include/mip_sdk_interface.h \
 ../../../../Library/Include/mip_sdk_config.h mip_sdk_user_functions.h \
 ../../../../Library/Include/mip.h \
 ../../../../Library/Include/ring_buffer.h \
 ../../../../Utilities/Include/byteswap_utilities.h \
 ../../../../Library/Include/mip_types.h
