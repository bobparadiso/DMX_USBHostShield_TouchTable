//running on Arduino UNO w/ USB Host Shield
//connected over USB to Enttec DMXIS or DMX USB PRO

#define LABEL_DMX_SEND 6

#define CHANNEL_PAN 1
#define CHANNEL_TILT 3
#define CHANNEL_COLORWHEEL 6
#define CHANNEL_DIMMER 11
#define CHANNEL_SHUTTER 12

#define DMX_START_CODE 0x7E 
#define DMX_END_CODE 0xE7 
#define DMX_HEADER_LENGTH 4
#define DMX_PACKET_SIZE 513

uint8_t dmxData[DMX_PACKET_SIZE];

#include <cdcftdi.h>
#include <usbhub.h>

//touch lib from Bare Conductive: https://github.com/BareConductive/mpr121/tree/public/MPR121
#include <MPR121.h>
#include <MPR121_Datastream.h>
#include <Wire.h>

// touch constants
const uint8_t MPR121_ADDR = 0x5A;
const uint8_t MPR121_INT = 4;

// MPR121 datastream behaviour constants
const bool MPR121_SAVED_THRESHOLDS = true;
//const bool MPR121_SAVED_THRESHOLDS = false;

#include <SPI.h>

//
class FTDIAsync : public FTDIAsyncOper
{
public:
    uint8_t OnInit(FTDI *pftdi);
};

//
uint8_t FTDIAsync::OnInit(FTDI *pftdi)
{
    uint8_t rcode = 0;
    rcode = pftdi->SetBaudRate(115200);

    if (rcode)
    {
        ErrorMessage<uint8_t>(PSTR("SetBaudRate"), rcode);
        return rcode;
    }
    rcode = pftdi->SetFlowControl(FTDI_SIO_DISABLE_FLOW_CTRL);

    if (rcode)
        ErrorMessage<uint8_t>(PSTR("SetFlowControl"), rcode);

    return rcode;
}

USB              Usb;
//USBHub         Hub(&Usb);
FTDIAsync        FtdiAsync;
FTDI             Ftdi(&Usb, &FtdiAsync);

//
int SendDMX(uint8_t label, uint8_t* data, uint16_t length)
{
  uint8_t  rcode;
	uint8_t end_code = DMX_END_CODE;
	// Form Packet Header
	uint8_t header[DMX_HEADER_LENGTH];
	header[0] = DMX_START_CODE;
	header[1] = label;
	header[2] = length & 0xFF;
	header[3] = length >> 8;
	// Write The Header
  rcode = Ftdi.SndData(DMX_HEADER_LENGTH, header);
  if (rcode) {
      ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
      return -1;
  }
	// Write The Data
  rcode = Ftdi.SndData(length, data);
  if (rcode) {
      ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
      return -1;
  }
	// Write End Code
  rcode = Ftdi.SndData(1, &end_code);
  if (rcode) {
      ErrorMessage<uint8_t>(PSTR("SndData"), rcode);
      return -1;
  }

  return 0;
}

//configure MPR121
void setupTouch() {
  if (!MPR121.begin(MPR121_ADDR)) {
    Serial.println("error setting up MPR121");
    switch (MPR121.getError()) {
      case NO_ERROR:
        Serial.println("no error");
        break;
      case ADDRESS_UNKNOWN:
        Serial.println("incorrect address");
        break;
      case READBACK_FAIL:
        Serial.println("readback failure");
        break;
      case OVERCURRENT_FLAG:
        Serial.println("overcurrent on REXT pin");
        break;
      case OUT_OF_RANGE:
        Serial.println("electrode out of range");
        break;
      case NOT_INITED:
        Serial.println("not initialised");
        break;
      default:
        Serial.println("unknown error");
        break;
    }
    while (1); //spin
  }

  MPR121.setInterruptPin(MPR121_INT);

  if (MPR121_SAVED_THRESHOLDS) {
    MPR121.restoreSavedThresholds();
  } else {
    MPR121.setTouchThreshold(20);
    MPR121.setReleaseThreshold(10);
  }

  MPR121.setFFI(FFI_10);
  MPR121.setSFI(SFI_10);
  MPR121.setGlobalCDT(CDT_4US);  // reasonable for larger capacitances

  digitalWrite(LED_BUILTIN, HIGH);  // switch on user LED while auto calibrating electrodes
  delay(1000);
  MPR121.autoSetElectrodes();  // autoset all electrode settings
  digitalWrite(LED_BUILTIN, LOW);

  //MPR121_Datastream.begin(&Serial);  // start datastream object using provided Serial reference
}

//
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  setupTouch();

  if (Usb.Init() == -1)
      Serial.println("OSC did not start.");

  memset(dmxData, 0, DMX_PACKET_SIZE);
  dmxData[CHANNEL_DIMMER] = 8;

  int8_t prevTarget = -1;
  int8_t target;
  while (1) {
    Usb.Task();

    MPR121.updateAll();
    //MPR121_Datastream.update();

    if (MPR121.getTouchData(3)) {
      dmxData[CHANNEL_PAN] = 152;
      dmxData[CHANNEL_TILT] = 0;
      dmxData[CHANNEL_COLORWHEEL] = 28;
      dmxData[CHANNEL_SHUTTER] = 255;
      target = 3;
    }
    else if (MPR121.getTouchData(2)) {
      dmxData[CHANNEL_PAN] = 161;
      dmxData[CHANNEL_TILT] = 0;
      dmxData[CHANNEL_COLORWHEEL] = 21;
      dmxData[CHANNEL_SHUTTER] = 255;
      target = 2;
    }
    else if (MPR121.getTouchData(1)) {
      dmxData[CHANNEL_PAN] = 176;
      dmxData[CHANNEL_TILT] = 0;
      dmxData[CHANNEL_COLORWHEEL] = 56;
      dmxData[CHANNEL_SHUTTER] = 255;
      target = 1;
    }
    else if (MPR121.getTouchData(0)) {
      dmxData[CHANNEL_PAN] = 186;
      dmxData[CHANNEL_TILT] = 0;
      dmxData[CHANNEL_COLORWHEEL] = 190;
      dmxData[CHANNEL_SHUTTER] = 255;
      target = 0;
    }
    else {
      dmxData[CHANNEL_PAN] = 169;
      dmxData[CHANNEL_TILT] = 126;
      dmxData[CHANNEL_SHUTTER] = 0;
      target = -1;
    }

    if( Usb.getUsbTaskState() == USB_STATE_RUNNING )
    {
        uint8_t  rcode;

        if (target != prevTarget)
          SendDMX(LABEL_DMX_SEND, dmxData, DMX_PACKET_SIZE);

        #define RECEIVE_SIZE 64
        uint8_t buf[RECEIVE_SIZE];
        memset(buf, 0, RECEIVE_SIZE);

        uint16_t rcvd = RECEIVE_SIZE;
        rcode = Ftdi.RcvData(&rcvd, buf);

        if (rcode && rcode != hrNAK)
            ErrorMessage<uint8_t>(PSTR("Ret"), rcode);

        // The device reserves the first two bytes of data
        //   to contain the current values of the modem and line status registers.
        if (rcvd > 2)
            Serial.print((char*)(buf+2));

        delay(10);
    }

    prevTarget = target;

    delay(10);
  }
}

//
void loop() {}
