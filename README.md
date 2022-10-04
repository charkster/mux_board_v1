# mux_board_v1
Custom PCB with ItsyBitsy M0 Express controlling 4 ADG1606 16 input muxes!

![picture](https://github.com/charkster/mux_board_v1/blob/main/mux_board_v1.png)

This board is super-easy to order from Oshpark. Just upload kicad/mux_board_v1.kicad_pcb and pay $31 for 3 boards (including shipping).
The ADG1606BRUZ muxes are difficult to obtain as Digikey and Mouser are frequently sold-out. I usually back-order the part and wait for it (my last wait was 3 months). The ItsyBitsy M0 Express board was selected as (1) I am very familar with the samd21, (2) it has enought gpios to control all the muxes and (3) it is $12 and almost always in-stock. The board can be programmed by just double-clicking the reset button and drag-n-drop the itsybitsy_m0-usbtmc_mux_board.uf2 file.
