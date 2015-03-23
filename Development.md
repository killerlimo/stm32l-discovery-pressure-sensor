# Design #

The BMP0805 sensor was purchased on a small PCB that included the I2C pull ups along with a 5V to 3V regulator. This was then soldered directly on the back of the Discovery board picking up the EXT\_5V and GND pins directly.

The I2C lines were connected to PB10 (SCL) & PB11 (SDA) via two thin wires.

By soldering the sensor board on the back of the Discovery it is mechanically supported and to some degree protected.


# I2C #

The STM32L has 2 I2C ports. On the Discovery board these are connected to:

  * I2C1 - The LEDs
  * I2C2 - The LCD display

Initial development was done using I2C2 and sacrificing some of the segments on the LCD. Later this may be changed to I2C1 and the LEDs removed from the circuit.