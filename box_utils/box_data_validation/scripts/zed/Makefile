# Compiler
CXX = g++

# CUDA Path (adjust this if necessary)
CUDA_PATH = /usr/local/cuda

# ZED SDK Path (adjust if necessary)
ZED_PATH = /usr/local/zed

# Compiler flags
CXXFLAGS = -std=c++11 -I$(ZED_PATH)/include -I$(CUDA_PATH)/include

# Linker flags for ZED SDK and CUDA
LDFLAGS = -L$(ZED_PATH)/lib -L$(CUDA_PATH)/lib64 -lsl_zed -lcudart

# Output executable name
TARGET = zed_recording_app

# Source files
SRC = main.cpp

# Build target
all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)

# Clean the build
clean:
	rm -f $(TARGET)
