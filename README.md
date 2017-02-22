<!-- vim: fo=a tw=80 colorcolumn=80 syntax=markdown :
-->

STM32F767ZI-Nucleo-144 board example project
============================================

Small example project for STM32F767ZI-Nucleo-144 board using the
hackers choice of tools:

* GCC

* Make

* GDB and one of st-link or openocd for debugging

Originally based on https://github.com/ulfen/stm32f7-discovery-blinky.git
(thanks to http://github.com/ulfen!), the Makefile has been enhanced with
additional debugging options and simple GDB integration.

The sources and Makefile have been stripped to bare minimum by referencing the
original CUBE7 package downloadable from ST at
http://www.st.com/en/embedded-software/stm32cubef7.html



Use
---

* The setup is optimized for debugging and GDB integration. Thus, it uses `-O0
-g -gdwarf2` to allow remote debugging. For production use, you might want to
change `DEBUG_OPTIMIZE_FLAGS` to `-O2`

* HEAP SIZE is currently fixed to 0x400 (in the Makefile). For a larger heap,
fix this. It is used in `sys/_sbrk.c`. A better solution would be to create an
appropriate linker script that defines the value for `HEAP_SIZE`, just like it
currently defines the start of the heap as `_end`.

* You need to download the CUBE F7 zip from ST and set the correspondig
environment variable or fix the Makefile.



Authors
-------

(alphabetically)

* David R. Piegdon <dgit@piegdon.de>

* Liviu Ionescu (`sys/_sbrk.c` via GNU ARM Eclipse Plug-in)

* ST Microelectronic (example `src/stm32f7xx_hal_conf.h`)

* Ulfen ( http://github.com/ulfen/ )



Prior Art
---------

* original version which this is based on, for a slightly different board:
https://github.com/ulfen/stm32f7-discovery-blinky.git

* ChibiOS has an example for many different boards, including this one:
http://chibios.org/

