all:
	gcc -o main main.c head.c dht.c ultrasonic.c pms.c l298n.c -lwiringPi -lpthread
clean:
	rem -rf main
