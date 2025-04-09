# Author : Byunghun Hwang <bh.hwang@iae.re.kr>


# Build for architecture selection (editable!!)
ARCH := $(shell uname -m)
OS := $(shell uname)

CURRENT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
CURRENT_DIR_NAME := $(notdir $(patsubst %/,%,$(dir $(CURRENT_DIR))))

# path
FLAME_PATH = $(CURRENT_DIR)/flame
INCLUDES = $(FLAME_PATH)/include
SOURCE_FILES = .

#Compilers
ifeq ($(ARCH),arm64)
	CC := g++
	GCC := gcc
	LD_LIBRARY_PATH += -L./lib/arm64
	OUTDIR		= $(CURRENT_DIR)/bin/arm64/
	BUILDDIR	= $(CURRENT_DIR)/bin/arm64/
	INCLUDE_DIR = -I./ -I$(CURRENT_DIR)/ -I$(CURRENT_DIR)/include/ -I$(CURRENT_DIR)/include/dep -I/usr/include
	LD_LIBRARY_PATH += -L/usr/local/lib -L./lib/arm64/
else ifeq ($(ARCH), armhf)
	CC := /usr/bin/arm-linux-gnueabihf-g++-9
	GCC := /usr/bin/arm-linux-gnueabihf-gcc-9
	LD_LIBRARY_PATH += -L./lib/armhf
	OUTDIR		= $(CURRENT_DIR)/bin/armhf/
	BUILDDIR	= $(CURRENT_DIR)/bin/armhf/
	INCLUDE_DIR = -I./ -I$(CURRENT_DIR)/ -I$(CURRENT_DIR)/include/ -I$(CURRENT_DIR)/include/dep -I/usr/include
	LD_LIBRARY_PATH += -L/usr/local/lib -L./lib/armhf/
else ifeq ($(ARCH), aarch64) # for Mac Apple Silicon
	CC := g++
	GCC := gcc
#	LD_LIBRARY_PATH += -L./lib/aarch64-linux-gnu
	OUTDIR		= $(CURRENT_DIR)/bin/aarch64/
	BUILDDIR	= $(CURRENT_DIR)/bin/aarch64/
	INCLUDE_DIR = -I./ -I$(CURRENT_DIR) -I$(FLAME_PATH)/include -I$(FLAME_PATH)/include/dep -I/usr/include -I/usr/local/include -I/usr/include/opencv4 -I/opt/pylon/include
	LIBDIR = -L/usr/local/lib -L$(CURRENT_DIR)/lib/aarch64-linux-gnu/
export LD_LIBRARY_PATH := $(LIBDIR):$(LD_LIBRARY_PATH)
else
	CC := g++
	GCC := gcc
#	LD_LIBRARY_PATH += -L./lib/x86_64
	OUTDIR		= $(CURRENT_DIR)/bin/x86_64/
	BUILDDIR	= $(CURRENT_DIR)/bin/x86_64/
	INCLUDE_DIR = -I./ -I$(CURRENT_DIR) -I$(FLAME_PATH)/include -I$(FLAME_PATH)/include/dep -I/usr/include -I/usr/local/include -I/usr/include/opencv4 -I/opt/pylon/include
	LIBDIR = -L/usr/local/lib -L$(FLAME_PATH)/lib/x86_64/ -L/opt/pylon/lib/
export LD_LIBRARY_PATH := $(LIBDIR):$(LD_LIBRARY_PATH)
endif

# OS
ifeq ($(OS),Linux) #for Linux
	LDFLAGS = -Wl,--export-dynamic -Wl,-rpath=. $(LIBDIR) -L$(LIBDIR)
	LDLIBS = -pthread -lrt -ldl -lm -lzmq -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_videoio
endif



$(shell mkdir -p $(OUTDIR))
$(shell mkdir -p $(BUILDDIR))
REV_COUNT = $(shell git rev-list --all --count)
MIN_COUNT = 0 #$(shell git tag | wc -l)

#if release(-O3), debug(-O0)
# if release mode compile, remove -DNDEBUG
CXXFLAGS = -O3 -fPIC -Wall -std=c++20 -D__cplusplus=202002L

#custom definitions
CXXFLAGS += -D__MAJOR__=0 -D__MINOR__=$(MIN_COUNT) -D__REV__=$(REV_COUNT)
RM	= rm -rf


# flame service engine
flame:	$(BUILDDIR)flame.o \
		$(BUILDDIR)config.o \
		$(BUILDDIR)manager.o \
		$(BUILDDIR)driver.o \
		$(BUILDDIR)instance.o
		$(CC) $(LDFLAGS) $(LD_LIBRARY_PATH) -o $(BUILDDIR)$@ $^ $(LDLIBS)

$(BUILDDIR)flame.o:	$(FLAME_PATH)/tools/flame/flame.cc
					$(CC) $(CXXFLAGS) $(INCLUDE_DIR) -c $^ -o $@
$(BUILDDIR)instance.o: $(FLAME_PATH)/tools/flame/instance.cc
						$(CC) $(CXXFLAGS) $(INCLUDE_DIR) -c $^ -o $@
$(BUILDDIR)manager.o: $(FLAME_PATH)/tools/flame/manager.cc
						$(CC) $(CXXFLAGS) $(INCLUDE_DIR) -c $^ -o $@
$(BUILDDIR)driver.o: $(INCLUDES)/flame/component/driver.cc
						$(CC) $(CXXFLAGS) $(INCLUDE_DIR) -c $^ -o $@
$(BUILDDIR)config.o: $(INCLUDES)/flame/config.cc
						$(CC) $(CXXFLAGS) $(INCLUDE_DIR) -c $^ -o $@


# components

basler_gige_cam_grabber.comp:	$(BUILDDIR)basler.gige.cam.grabber.o
							$(CC) $(LDFLAGS) $(LD_LIBRARY_PATH) -shared -o $(BUILDDIR)/patroller/$@ $^ $(LDFLAGS) $(LDLIBS) -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lpylonbase -lpylonutility 
$(BUILDDIR)basler.gige.cam.grabber.o:	$(CURRENT_DIR)/components/basler.gige.cam.grabber/basler.gige.cam.grabber.cc
									$(CC) $(CXXFLAGS) $(INCLUDE_DIR) -c $^ -o $@


all : flame

patroller : flame basler_gige_cam_grabber.comp

deploy : FORCE
	cp $(BUILDDIR)/*.comp $(BUILDDIR)/flame $(BINDIR)
clean : FORCE 
		$(RM) $(BUILDDIR)/*.o $(BUILDDIR)/*.comp $(BUILDDIR)/patroller/*.comp $(BUILDDIR)/flame
debug:
	@echo "Building for Architecture : $(ARCH)"
	@echo "Building for OS : $(OS)"

FORCE : 