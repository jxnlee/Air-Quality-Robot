all:
	gcc -o main main.c drivers/head.c drivers/dht.c drivers/ultrasonic.c drivers/pms.c drivers/l298n.c drivers/fan.c drivers/nion_gen.c -lwiringPi
	gcc -fPIC -shared -o drivers/body.so drivers/body.c drivers/head.c drivers/ultrasonic.c drivers/dht.c drivers/pms.c drivers/l298n.c drivers/fan.c drivers/nion_gen.c -lwiringPi
	gcc -fPIC -shared -o drivers/ultrasonic.so drivers/ultrasonic.c drivers/head.c -lwiringPi
	gcc -fPIC -shared -o drivers/dht.so drivers/dht.c drivers/head.c -lwiringPi
	gcc -fPIC -shared -o drivers/l298n.so drivers/l298n.c drivers/head.c -lwiringPi
	gcc -fPIC -shared -o drivers/pms.so drivers/pms.c drivers/head.c -lwiringPi
	gcc -fPIC -shared -o drivers/fan.so drivers/fan.c drivers/head.c -lwiringPi
	gcc -fPIC -shared -o drivers/nion_gen.so drivers/nion_gen.c drivers/head.c -lwiringPi
clean:
	rem -rf drivers/ultrasonic.so
	rem -rf drivers/dht.so
	rem -rf drivers/l298n.so
	rem -rf drivers/body.so
	rem -rf drivers/pms.so
	rem -rf drivers/fan.so
	rem -rf drivers/nion_gen.so
