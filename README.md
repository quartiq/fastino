# Fastino gateware and software development

* Fastino is a 32 channels, 2 M/s simultaneous, 16 bit DAC
* [AD5542ABCPZ](https://www.analog.com/media/en/technical-documentation/data-sheets/AD5512A_5542A.pdf)

## Hardware

Hardware https://github.com/sinara-hw/Fastino/wiki

## Link

* Interface: single EEM
    * CLK word clock at 250 MHz/7, 1:7 serialization/deserialization, and 3:4 clock duty cycle
    * MOSI0 data 250 Mb/s, 125 MHz DDR bit clock
    * MOSI1
    * MOSI2
    * MOSI3
    * MOSI4
    * MOSI5
    * MISO return data at 125/7 Mb/s (TBD)
* Each word (one word clock cycle with 7 bits per lane) is `7*6=42` bits.
* A frame consists of 14 words.
* A frame contains 14/2+1 marker bits to provide EOF alignment and a 12 bit CRC.
* This leaves 568 bits per frame for `32*16` bits DAC data, a 32 bit channel
  mask, and 24 bits for configuration and return data addressing
* On 6 data lanes, this achieves 1.5 Gb/s raw from the FPGA and 1.45 Gb/s
  net payload (after accounting for framing and checksum).
* The frame period is 392 ns and the DAC data/sample/update rate is
  therefore 2.55 MS/s.
* There is one slower upstream lane at 125/7 Mb/s providing 7 bits per
  frame return data. Details TBD
* Similar to well-known video and highspeed/serdes data links
* Link design benefits Grabber HITL testing
* The Link design can be reused for Humpback and Banker where the same FPGA sits on the EEM.
* Link training (bit slip, delay alignment) is automatic
* Checksum protects the data

### Tools

* migen
* yosys
* nextpnr

### Build

* Path migen to use the `heap` placer in `nextpnr`

```
python fastino_phy.py
```

### FLash

See https://github.com/quartiq/kasli-i2c

Figure out the `aa-bb-cc-dd-ee-ff` mac address of the Kasli connected. Otherwise
if you are using a Kasli that was not provisioned using the tools in `kasli-i2c`,
patch `flash_fastino.py` to make it find your Kasli. Assuming Fastino is
connected to EEM0:

```
python flash_fastino.py aa-bb-cc-dd-ee-ff EEM0 write fastino.bin
```
