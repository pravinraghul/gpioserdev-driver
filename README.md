# GPIO Serial Communication Device

## Data Transfer Design
In this design, the strobe signal controls the timing, and on each rising edge, the value on the data line is captured. The data is transmitted sequentially, with each bit being read on the strobe's rising edge. 

The data is transmitted in the order of the least significant bit (LSB) to the most significant bit (MSB) in the below example, but it can be configured to transmit in the opposite direction.

![Data Transfer Design](https://github.com/pravinraghul/gpiobitserdev-driver/blob/master/gpioserdev-data-transfer-design.png)