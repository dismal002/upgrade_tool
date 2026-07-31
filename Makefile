CXX ?= g++
CXXFLAGS ?= -O2 -Wall -std=c++14

# Auto-detect libusb flags
LIBUSB_SO := $(shell find /usr/lib /usr/lib64 /usr/lib/*-linux-* -name "libusb-1.0.so*" 2>/dev/null | head -n 1)

ifneq ($(LIBUSB_SO),)
    LIBS ?= $(LIBUSB_SO) -lpthread
else
    LIBS ?= -lusb-1.0 -lpthread
endif

TARGET = upgrade_tool
SRCS = upgrade_tool.cpp

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) $(LIBS) -o $(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all clean
