########################### O Makefile principal ##########################

CC=gcc
CFLAGS=-g -ggdb -lm -pthread -Wall -pedantic -ansi  -std=c99
EXEC=server
VPATH=src


all: $(EXEC)
	rm -f *.o;
$(EXEC): process_data.o server_daemon_start.o server_sock.o server_daemon.o
	@echo "\nCreating executable: $(EXEC)\n";  
	$(CC) *.o  -o $(EXEC) $(CFLAGS);
server_daemon.o: serv_struct.h server_daemon.c
	@echo "\nCompiling server_daemon.o\n"; 
	$(CC) $^ -c $(CFLAGS);
server_sock.o: serv_struct.h server_sock.c
	@echo "\nCompiling server_sock.o\n"; 
	$(CC) $^ -c $(CFLAGS);
server_daemon_start.o: serv_struct.h server_daemon_start.c
	@echo "\nCompiling server_daemon_start.o\n"; 
	$(CC) $^ -c $(CFLAGS);
process_data.o: process_data.c
	@echo "\nCompiling process_data.o\n";
	$(CC) $^ -c $(CFLAGS);
clean: 
	rm -f bin/*.o *.o $(EXEC);
