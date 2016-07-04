########################### O Makefile principal ##########################

CC=g++
CFLAGS=-g -ggdb -lrt -lm -Wall -pedantic -ansi -pthread -std=c++11
EXEC=bin/server
VPATH=source


all: $(EXEC)
	rm -f *.o;
$(EXEC): main.o gps.o
	@echo "\nCreating executable: $(EXEC)\n";
	$(CC) *.o -o $(EXEC) $(CFLAGS);
main.o: main.cpp
	@echo "\ncompiling main.o\n";
	$(CC) $^ -c $(CFLAGS)
gps.o: gps.cpp
	@echo "\ncompiling gps.o\n";
	$(CC) $^ -c $(CFLAGS)
clean:
	rm -f bin/*.o *.o $(EXEC);
