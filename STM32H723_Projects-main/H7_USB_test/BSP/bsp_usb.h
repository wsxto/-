#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

extern uint8_t Buf[300];


void SendByte(uint8_t date);
void Send(const uint8_t *data,uint8_t len);
uint16_t CRC16_Check(const uint8_t *data,uint8_t len);
void Send_Cmd_Data(uint8_t cmd,const uint8_t *datas,uint8_t len);

void Data_Analysis(uint8_t cmd, const uint8_t* datas, uint8_t len);
void Receive(uint8_t bytedata);




