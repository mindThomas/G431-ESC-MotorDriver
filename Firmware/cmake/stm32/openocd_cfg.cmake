set(CONFIG_TEXT "source [find interface/stlink.cfg]\n\
\n\
set WORKAREASIZE 0x8000\n\
\n\
transport select \"hla_swd\"\n\
\n\
set CHIPNAME STM32H743ZITx\n\
set BOARDNAME AUTOGENERATED\n\
\n\
# CHIPNAMES state\n\
set CHIPNAME_CPU0_ACTIVATED 1\n\
\n\
# Enable debug when in low power modes\n\
set ENABLE_LOW_POWER 1\n\
\n\
# Stop Watchdog counters when halt\n\
set STOP_WATCHDOG 1\n\
\n\
# STlink Debug clock frequency\n\
set CLOCK_FREQ 8000\n\
\n\
# use hardware reset, connect under reset\n\
# connect_assert_srst needed if low power mode application running (WFI...)\n\
reset_config srst_only srst_nogate connect_assert_srst\n\
set CONNECT_UNDER_RESET 1\n\
\n\
source [find target/${OPENOCD_TARGET_CFG}]\n\
")

file(WRITE "${OPENOCD_CFG}" "${CONFIG_TEXT}")