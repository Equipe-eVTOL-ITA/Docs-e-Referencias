CC = g++
PROJECT = rec_cor
SRC = rec_cor_quadrado.cpp

OPENCV_FLAGS = $(shell pkg-config opencv4 --cflags)
OPENCV_LIBS = $(shell pkg-config opencv4 --libs)
$(PROJECT) : $(SRC)
	$(CC) $(OPENCV_FLAGS) $(SRC) $(OPENCV_LIBS) -o $(PROJECT) 