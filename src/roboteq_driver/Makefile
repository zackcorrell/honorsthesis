#Subdir makefile for Roboteq AX2850 plugin driver
#2009 IGVC Competition
#Penn State Robotics Club
#rjm5066@psu.edu

include ../../Makefile.inc

SRC = roboteq.cc
OBJLIBS = $(LIBDIR)/libroboteqplugin.so
OBJS = roboteq.o

all: $(OBJLIBS)

$(OBJS): $(SRC)
	$(ECHO) Building Roboteq Driver...
	$(CXX) $(PRJCXXFLAGS) `pkg-config --cflags playercore` -c -Wextra -Wall $(SRC)

$(OBJLIBS): $(OBJS)
	$(LD) $(PRJLDFLAGS)  -o $@ $^

clean:
	$(ECHO) Cleaning up Roboteq Driver...
	$(RM) -f $(OBJS) $(OBJLIBS)

