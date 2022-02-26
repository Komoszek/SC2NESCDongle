# SC2NESCDongle

SC2NESCDongle is a special firmware for CC2541 MCU that transforms your BLE module into dongle used to wirelessly connect Steam Controller with NES Classic/Mini via front controller port.

## How does it work?

This firmware makes CC2541 MCU translate connected SC output into Wii Classic Controller output which is compatible with NES Classic. Data from SC is transported via BLE and data from CC2541 is send via i2c.

## Requirements for building
- [IAR Embedded Workbench IDE - 8051 10.40](https://www.iar.com/products/architectures/iar-embedded-workbench-for-8051/) (not the code size limited version)
- [TI BLE Stack for CC254x](https://www.ti.com/tool/download/BLE-STACK-1-X)

## Burning firmware
I personally use [CCloader](https://github.com/RedBearLab/CCLoader) to burn firmware. You may also use official CC Debugger but you would have to buy it.

## Before you build
1. Install TI BLE Stack.
2. Clone this repository to `./Projects/ble` directory of installed TI BLE Stack.
3. Edit file `./Projects/ble/common/cc2540/ti_51ew_cc2540b.xcl` by appending following lines:
```
-D?B=F0
-D?IE=A8
```

## Hooking up CC2541 with NESC
First of all, you'll need [Wiimote extension connector](https://www.aliexpress.com/item/1005003281952214.html) (i've got mine from NESC extension cord). Then you will need to connect this plug with CC2541 appropriately. Pinout of Wiimote extension can be found [here](https://www.flickr.com/photos/aalmada/7085452395) and CC2541 datasheet can be found [here](https://www.ti.com/lit/ds/symlink/cc2541.pdf?ts=1645871792478).

## Disclaimer
I've started this project way back in 2018 and even though it is in working state i.e. it works fine with SC and NESC, it will probably
never be fully finished.

## TODO
- Working buttons and status LED
- Board design
- Controller configuration loading
- Custom case
