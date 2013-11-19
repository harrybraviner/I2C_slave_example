I2C_slave_example.hex : I2C_slave_example.c
	sdcc --use-non-free -mpic16 -p18f2420 I2C_slave_example.c
