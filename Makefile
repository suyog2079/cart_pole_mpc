CXX      = g++
CXXFLAGS = -std=c++17 -O3 -I/usr/local/include -Iinclude
LDFLAGS  = -L/usr/local/lib -lcasadi -lipopt -lblas -llapack -lm -lpthread -latomic
MAKEFLAGS += -j$(nproc)

TARGET  = cartpole_nmpc.out
SRCS    = src/main.cpp
HEADERS = include/nmpc.hpp include/network.hpp

$(TARGET): $(SRCS) $(HEADERS)
	$(CXX) $(SRCS) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET)

clean:
	rm -f $(TARGET)
