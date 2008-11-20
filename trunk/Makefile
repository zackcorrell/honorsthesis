# Makefile for 2009 IGVC Project
# Penn State Robotics Club
# www.psurobotics.org
# rjm5066@psu.edu

include Makefile.inc

LIBDIRS = 
DRIVERDIRS = src/roboteq_driver src/mricp_plugin
EXEDIRS = 

# Build whole project

all: libraries program drivers

# Build algorithm libraries
libraries:
	$(ECHO) Building Libraries...
	for d in $(LIBDIRS); do (cd $$d; $(MAKE) $(MFLAGS) ); done
	
# Build hardware drivers
drivers:
	$(ECHO) Building Drivers...
	for d in $(DRIVERDIRS); do (cd $$d; $(MAKE) $(MFLAGS) ); done
	
# Build exectuable programs
program:
	$(ECHO) Building Executables...
	for d in $(EXEDIRS); do (cd $$d; $(MAKE) $(MFLAGS) ); done
	
# Clean whole project
clean :
	$(ECHO) Cleaning Up Project...
	for d in $(DRIVERDIRS); do (cd $$d; $(MAKE) clean ); done
	for d in $(LIBDIRS); do (cd $$d; $(MAKE) clean ); done
	for d in $(EXEDIRS); do (cd $$d; $(MAKE) clean ); done
	