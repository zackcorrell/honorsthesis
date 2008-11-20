# Makefile Include for 2009 IGVC Project
# Penn State Robotics Club
# www.psurobotics.org
# rjm5066@psu.edu

CC	= gcc
CXX = g++

#PRJCXXFLAGS = -Wall -O3
PRJCXXFLAGS = -Wall -fpic -g3 -O2
PRJCFLAGS = -g

LD	= g++
PRJLDFLAGS	= -Wall -shared -nostartfiles -rdynamic 

AR	= ar
ARFLAGS	=
RANLIB	= ranlib
RM	= rm
ECHO = echo

LIBDIR = ../../lib
EXEDIR = ../../bin

SHELL	= /bin/sh

#.SILENT: