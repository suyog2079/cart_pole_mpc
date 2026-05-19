CXX      = g++
CXXFLAGS = -std=c++17 -O3 -I/usr/local/include
LDFLAGS  = -L/usr/local/lib -lcasadi -lipopt -lblas -llapack -lm -lpthread -latomic
MAKEFLAGS += -j$(nproc)

TARGET  = cartpole_nmpc.out
SRCS    = cartpole_nmpc.cpp
HEADERS =

$(TARGET): $(SRCS) $(HEADERS)
	$(CXX) $(SRCS) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET)

clean:
	rm -f $(TARGET)
