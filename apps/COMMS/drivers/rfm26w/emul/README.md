# rfm26w/emul: software model of the radio, for host tests

`rfm26w_emul.c` and `rfm26w_emul.h` model the RFM26W / Si446x on Zephyr's SPI and GPIO emulators, for
`native_sim`. They are test support code, not part of the driver or the firmware. Two test apps compile them:
`apps/tests/rfm26wEmulTest` (tests the driver) and `apps/tests/commsTest` (tests the firmware against the
driver). Both use the same devicetree node label, `rfm26w0`.

What the model is and is not: see the comment at the top of `rfm26w_emul.h` and PLANNED.md section 5.2, "Emulator
tests". It checks logic, not the chip's real behavior.
