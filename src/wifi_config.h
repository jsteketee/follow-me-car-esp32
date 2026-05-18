#pragma once

#include <stdarg.h>

void wifi_init();
void wifi_update();         // call each loop — accepts new telnet clients
void wifi_printf(const char *fmt, ...);  // print to USB Serial + telnet client
bool wifi_available();      // bytes waiting from telnet client
char wifi_read();           // read one byte from telnet client
