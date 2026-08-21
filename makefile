# Cross-platform makefile (Linux + macOS).
#
# Föredrar pkg-config (funkar t.ex. med `brew install raylib` på macOS,
# eller `sudo make install` från raylib-källkod på Linux). Om pkg-config
# inte hittar raylib faller vi tillbaka på vanliga standardsökvägar per OS.

CXX ?= clang++
CXXFLAGS = -std=c++17 -Wall -O2

RAYLIB_CFLAGS := $(shell pkg-config --cflags raylib 2>/dev/null)
RAYLIB_LIBS := $(shell pkg-config --libs raylib 2>/dev/null)

ifeq ($(strip $(RAYLIB_LIBS)),)
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Darwin)
        UNAME_M := $(shell uname -m)
        ifeq ($(UNAME_M),arm64)
            BREW_PATH = /opt/homebrew
        else
            BREW_PATH = /usr/local
        endif
        RAYLIB_CFLAGS = -I$(BREW_PATH)/include
        RAYLIB_LIBS = -L$(BREW_PATH)/lib -lraylib -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo
    else
        RAYLIB_CFLAGS = -I/usr/local/include
        RAYLIB_LIBS = -L/usr/local/lib -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
    endif
endif

TARGET = game
SRC = $(wildcard src/*.cpp)

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(RAYLIB_CFLAGS) $(SRC) -o $(TARGET) $(RAYLIB_LIBS)

run: all
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all run clean
