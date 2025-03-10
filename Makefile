all:
	gcc -o main main.c head.c dht.c ultrasonic.c pms.c l298n.c -lwiringPi -lpthread
	gcc -fPIC -shared -o body.so body.c head.c ultrasonic.c dht.c pms.c l298n.c -lwiringPi -lpthread
	gcc -fPIC -shared -o ultrasonic.so ultrasonic.c head.c -lwiringPi -lpthread
	gcc -fPIC -shared -o dht.so dht.c head.c -lwiringPi -lpthread
	gcc -fPIC -shared -o l298n.so l298n.c head.c -lwiringPi -lpthread
clean:
	rem -rf main
	rem -rf ultrasonic.so
	rem -rf dht.so
	rem -rf l298n.so
