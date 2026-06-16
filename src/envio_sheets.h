

#ifndef ENVIO_SHEETS_H
#define ENVIO_SHEETS_H

#include <Arduino.h>
#include "config.h"

#if MEU_ID == BASE_ID
void conectarWiFi();
void iniciarSheets();
void enviarParaSheets(uint8_t nodeId, float temp, float hum);
#endif

#endif
