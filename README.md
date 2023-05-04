# mux_board_v1
Custom PCB with ItsyBitsy M0 Express controlling **FOUR ADG1606BRUZ 16 input muxes!**

**--> UPDATED design available: [mux_board_v5](https://github.com/charkster/mux_board_v5) <--**

![picture](https://github.com/charkster/mux_board_v1/blob/main/mux_board_v1.png)

Check-out my **PDF** [schematic](https://github.com/charkster/mux_board_v1/blob/main/kicad/mux_board_v1_sch_plot.pdf) !!

This board is super-easy to order from Oshpark. Just upload kicad/mux_board_v1.kicad_pcb and pay $31 for 3 boards (including shipping).
The ItsyBitsy M0 Express board was selected as (1) I am very familar with the samd21, (2) it has enought gpios to control all the muxes and (3) it is $12 and almost always in-stock. The ItsyBitsy board can be programmed by just double-clicking the reset button and drag-n-drop the itsybitsy_m0-usbtmc_mux_board.uf2 file.

**Welcome to the fricken future people...** drag-n-drop the kicad_pcb file to order the board, and drag-n-drop the uf2 file to program the MCU! Sorry, nothing I can do about the soldering.

Mux outputs are on the 1757268 connector, which is a Phoenix Contact connector. This connector can be omitted and posts can be put in the holes.

Dual channel opamp and muxes working great, but the level-shifter schematic is incorrect (don't populate the NVT2008 part). The NVT2008 needs to have high value resistors connect to the VREFB and EN inputs (unlike the TI TXS01018E).

What am I planning to do with this board? I need to connect two Keithley 2460 SMUs to my ASIC board to support bench trimming. I thought this would be a cool new lab instrument. The VDD PIN can be driven by the USB's 5V or externally up to 16V (mux inputs should be less than VDD).

Here are the SCPI commands which can be used:

**MUX1:EN 1** # enable MUX1, a zero will disable the mux

**MUX1:SEL 1** # select S1 input, values 1 through 16 are valid

**MUX1:EN?** # this query returns the state of MUX1

**MUX1:SEL?** # this query returns the presently selected input

**SOURC1:VOLT:LEV 2.2** # drive 2.2V from DAC

**SOURC1:VOLT:LEV?** # this query returns the DAC level

***RST** # disables all MUX enables and sets DAC to 0V

***IDN?** # returns valid commands and this URL

The tinyusb_usbtmc files are modifications of https://github.com/hathach/tinyusb source files. 
