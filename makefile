# Identifiera om vi kör Apple Silicon (arm64) eller Intel (x86_64)
UNAME_M := $(shell uname -m)

ifeq ($(UNAME_M),arm64)
    BREW_PATH = /opt/homebrew
else
    BREW_PATH = /usr/local
endif

CXX = clang++
CXXFLAGS = -std=c++17 -Wall -I$(BREW_PATH)/include
LDFLAGS = -L$(BREW_PATH)/lib -lraylib -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo

TARGET = game
SRC = main.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

run: all
	./$(TARGET)

clean:
	rm -f $(TARGET)