CXX ?= g++
CXXFLAGS ?= -O2 -Wall -std=c++14

LIBS ?= -lusb-1.0 -lpthread

TARGET = upgrade_tool
SRCS = main.cpp crc.cpp RKLog.cpp RKComm.cpp RKScan.cpp

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) $(LIBS) -o $(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all clean
