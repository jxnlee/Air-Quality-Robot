all:
	gcc -o main main.c head.c dht.c ultrasonic.c pms.c l298n.c fan.c nion_gen.c mpu6050.c -lwiringPi
	gcc -fPIC -shared -o body.so body.c head.c ultrasonic.c dht.c pms.c l298n.c fan.c nion_gen.c -lwiringPi
	gcc -fPIC -shared -o ultrasonic.so ultrasonic.c head.c -lwiringPi
	gcc -fPIC -shared -o dht.so dht.c head.c -lwiringPi
	gcc -fPIC -shared -o l298n.so l298n.c head.c -lwiringPi
	gcc -fPIC -shared -o pms.so pms.c head.c -lwiringPi
	gcc -fPIC -shared -o fan.so fan.c head.c -lwiringPi
	gcc -fPIC -shared -o nion_gen.so nion_gen.c head.c -lwiringPi
clean:
	rem -rf ultrasonic.so
	rem -rf dht.so
	rem -rf l298n.so
	rem -rf body.so
	rem -rf pms.so
	rem -rf fan.so
	rem -rf nion_gen.so
