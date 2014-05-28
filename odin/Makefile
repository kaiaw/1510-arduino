# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++
# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
CFLAGS=-c -g -Wall
SOFTLIB=/home/kai/Downloads/arduino-1.0.5/libraries/SoftwareSerial/SoftwareSerial.h
SOFTLIB=/home/kai/Downloads/arduino-1.0.5/libraries/

all: odin

odin: odin.ino GPS.cpp GPS_UTILS.cpp  
	$(CC) $(SOFTLIB) $(ARDULIB) -g -Wall odin.ino GPS.cpp GPS_UTILS.cpp -o odin

odin.ino: odin.ino
	$(CC) $(SOFTLIB) $(ARDULIB) $(CFLAGS) odin.ino

GPS.cpp: GPS.cpp
	$(CC) $(SOFTLIB) $(ARDULIB) $(CFLAGS) GPS.cpp

GPS_UTILS.cpp: GPS_UTILS.cpp
	$(CC) $(SOFTLIB) $(ARDULIB) $(CFLAGS) GPS_UTILS.cpp

clean:
	rm -rf *.o odin
