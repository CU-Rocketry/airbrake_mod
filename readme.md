# airbrake_mod

## Changes

- More BOM attention (remember to order the inductor!)
- Active antenna
- 0603 passives
- maybe a supercap drain resistor and switch
- Select alternative to AP7361C with enable (consider AP7361C-33Y5-13) to allow software disable of power to sensors
- expose testpoints
  - accelerometer SPI (and maybe flash)
  - GPS UART
  - barometer I2C
- flash footprint
- PPTC fuses protecting power tree
- zener protection on applicable signal lines. see [alex's proposed protection](https://discord.com/channels/826903793320591380/1161778268005994497/1353923222676049960)
- rearrange SPI pins to allow choice between hardware and software CS
