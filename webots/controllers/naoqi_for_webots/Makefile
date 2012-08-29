###
### Makefile for the "naoqi_for_webots" project
### 
### Author:    Yvan Bourquin - April 18th, 2011.
###
### Important:
###   On Mac OS X and Linux you can compile this project with Webots text editor
###   On Windows you need to use the naoqi_for_webots.sln
###   Please see the instructions in the readme.html file.

ifeq ($(AL_DIR),)
$(error you must define AL_DIR to compile this project)
endif

### -----C++ Sources-----
###
CPP_SOURCES = naoqi_for_webots.cpp naoproxy.cpp

### -----C/C++ Options-----
###
### Additional flags for the gcc compiler:
###
CFLAGS=-I"$(AL_DIR)/include"
USE_C_API=1

### Do not modify: this includes Webots global Makefile.include
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/projects/default/controllers/Makefile.include

### additional libraries:
LIBRARIES=-L"$(AL_DIR)/lib" -lnaoqiclient

### On Mac: do not attemps to compile and link with x86_64 arch
### because libnaoqiclient.dylib comes as i368 only
ifeq ($(OSTYPE),darwin)
EXTRA_CFLAGS=-mmacosx-version-min=10.6 -arch i386
EXTRA_LFLAGS=-mmacosx-version-min=10.6 -arch i386
endif
