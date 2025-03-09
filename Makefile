all:
	#gcc -o main main.c head.c dht.c ultrasonic.c pms.c -lwiringPi -lpthread
	gcc -fPIC -shared -o ultrasonic.so ultrasonic.c -lwiringPi -lpthread
	gcc -fPIC -shared -o dht.so dht.c -lwiringPi -lpthread
clean:
	#rem -rf main
	rem -rf ultrasonic.so
	rem -rf dht.so
