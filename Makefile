#makefile
all: hsv2amigo

LIBS =`pkg-config --cflags --libs opencv`
CFLAGS=-fPIC -g -Wall
ARIA_INCLUDE=-I/usr/local/Aria/include
ARIA_LINK=-L/usr/local/Aria/lib -lAria -lpthread -lv4l2

% : %.cpp
	$(CXX) $(CFLAGS) $(ARIA_INCLUDE) $^ -o $@ $(LIBS) $(ARIA_LINK)

clean:
	rm hsv2amigo
