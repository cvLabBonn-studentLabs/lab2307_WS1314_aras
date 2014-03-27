INCLUDES = -I. -I/usr/local/include/opencv -I/usr/local/include/opencv/opencv2 -I/usr/include/gsl -I/usr/include
LIBS = -lopencv_core -lopencv_highgui -lopencv_imgproc -lpng -lGLEW -lGL -lGLU -lglut -lX11 -lXmu -lXi -lgsl -lgslcblas
LIBDIRS = -L/usr/local/lib/ -Wl,-rpath,/usr/local/lib/ -L/usr/include/gsl

# Standard
OPT = -O3 -Wno-deprecated

CC=g++

.PHONY: all clean

OBJS = Transform.o NShow.o CTMesh.o NMath.o NRBM.o CRender.o fbo.o 

clean:
		rm -f *.o *~ *.gch Segment

all:	Project
		echo all: make complete
%.o:%.cpp
	$(CC) -c $(INCLUDES) $+ $(OPT)

Project: Project.o $(OBJS)
		$(CC) $(LIBDIRS) $(LIBS) -o $@ $+ $(OPT)




