#Subdir makefile for Roboteq AX2850 plugin driver
#2009 IGVC Competition
#Penn State Robotics Club
#rjm5066@psu.edu

include ../../Makefile.inc

SRC = Timer.cpp nn.cpp mricp_driver.cpp map.cpp lasermodel.cpp icp.cpp geometry2D.cpp
OBJLIBS = $(LIBDIR)/libMRICPplugin.so
OBJS = Timer.o nn.o mricp_driver.o map.o lasermodel.o icp.o geometry2D.o

all: $(OBJLIBS)

$(OBJS): $(SRC)
	$(ECHO) Building MRICP Driver...
	$(CXX) $(PRJCXXFLAGS) `pkg-config --cflags playercore gtk+-2.0 gdk-pixbuf glib-2.0` -c $(SRC)

$(OBJLIBS): $(OBJS)
	$(LD) $(PRJLDFLAGS)  -o $@ $^

clean:
	$(ECHO) Cleaning up MRICP Driver...
	$(RM) -f $(OBJS) $(OBJLIBS)

