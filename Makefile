port = /dev/ttyACM0
fqbn = arduino:avr:mega

default: upload

upload: compile
	arduino-cli upload -v -p $(port) --fqbn $(fqbn) firmware

compile: clean
	arduino-cli compile --fqbn $(fqbn) firmware

install_libs:
	arduino-cli lib install \
		SD \
		"Adafruit GPS Library" \
		"Adafruit MPU6050" \
		"Adafruit BusIO" \
		"Adafruit Unified Sensor"

clean:
	rm -f \
		firmware/*.elf \
		firmware/*.hex 
