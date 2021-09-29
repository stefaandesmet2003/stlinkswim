#include <stdio.h>
#include <stdlib.h>
#include <string.h> // memset needs this

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>

#include "SWIM.h"
#include <timing_stm32.h>

// STLINK usb interface
#define STLINK_GET_VERSION             0xF1
#define STLINK_DEBUG_COMMAND           0xF2
#define STLINK_DFU_COMMAND             0xF3
#define STLINK_SWIM_COMMAND            0xF4
#define STLINK_GET_CURRENT_MODE        0xF5
#define STLINK_GET_TARGET_VOLTAGE      0xF7

#define STLINK_DEV_DFU_MODE            0x00
#define STLINK_DEV_MASS_MODE           0x01
#define STLINK_DEV_DEBUG_MODE          0x02
#define STLINK_DEV_SWIM_MODE           0x03
#define STLINK_DEV_BOOTLOADER_MODE     0x04
#define STLINK_DEV_UNKNOWN_MODE        -1

#define STLINK_DFU_EXIT                0x07

// swim commands
#define STLINK_SWIM_ENTER               0x00
#define STLINK_SWIM_EXIT                0x01
#define STLINK_SWIM_READ_CAP            0x02
#define STLINK_SWIM_SPEED               0x03
#define STLINK_SWIM_ENTER_SEQ           0x04
#define STLINK_SWIM_GEN_RST             0x05
#define STLINK_SWIM_RESET               0x06
#define STLINK_SWIM_ASSERT_RESET        0x07
#define STLINK_SWIM_DEASSERT_RESET      0x08
#define STLINK_SWIM_READSTATUS          0x09
#define STLINK_SWIM_WRITEMEM            0x0a
#define STLINK_SWIM_READMEM             0x0b
#define STLINK_SWIM_READBUF             0x0c
#define STLINK_SWIM_READ_BUFFERSIZE     0x0d

// swim error codes
#define STLINK_SWIM_OK          0x00
#define STLINK_SWIM_BUSY        0x01
#define STLINK_SWIM_NO_RESPONSE 0x04 // Target did not respond. SWIM not active?
#define STLINK_SWIM_BAD_STATE   0x05 // ??

// for handling the stlink protocol
#define STLINK_STATE_CMD    0
#define STLINK_STATE_READ   1
#define STLINK_STATE_WRITE  2

typedef struct {
  uint8_t state;
  uint8_t mode;
  uint16_t totalBytes;
  uint16_t curBytes;
  uint32_t address;
} stlinkStatus_t;
static stlinkStatus_t stlinkStatus;

static uint8_t epBuffer[64];
static uint8_t swimBuffer[SWIM_BUFFERSIZE];

static const struct usb_device_descriptor dev = {
  .bLength = USB_DT_DEVICE_SIZE,
  .bDescriptorType = USB_DT_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = 0,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = 64,
  .idVendor = 0x0483, // STMicroelectronics
  .idProduct = 0x3748, // ST-LINK/V2
  .bcdDevice = 0x0100,
  .iManufacturer = 1,
  .iProduct = 2,
  .iSerialNumber = 3,
  .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor data_endp[] = {{
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x81,
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 0,
}, {
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x02,
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 0,
}, {
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x83,
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 0,
}};

static const struct usb_interface_descriptor data_iface[] = {{
  .bLength = USB_DT_INTERFACE_SIZE,
  .bDescriptorType = USB_DT_INTERFACE,
  .bInterfaceNumber = 0,
  .bAlternateSetting = 0,
  .bNumEndpoints = 3,
  .bInterfaceClass = 255, // vendor specific class
  .bInterfaceSubClass = 255, // vendor specific subclass
  .bInterfaceProtocol = 255, // vendor specific protocol
  .iInterface = 4,

  .endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {
  {
  .num_altsetting = 1,
  .altsetting = data_iface,
  }
};

static const struct usb_config_descriptor config = {
  .bLength = USB_DT_CONFIGURATION_SIZE,
  .bDescriptorType = USB_DT_CONFIGURATION,
  .wTotalLength = 0, // filled in by usb_standard.c
  .bNumInterfaces = 1,
  .bConfigurationValue = 1,
  .iConfiguration = 0,
  .bmAttributes = 0x80,
  .bMaxPower = 0x64,

  .interface = ifaces,
};

static const char *usb_strings[] = {
  "STMicroelectronics",
  "STM32 STLink",
  "PÿnPfVW59",
  "ST Link",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

// like BMP
static void platform_init (void)
{
  // clock setup
  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  /* Enable peripherals */
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_GPIOA); // usb & uart
  rcc_periph_clock_enable(RCC_GPIOB); // swim lines
  rcc_periph_clock_enable(RCC_GPIOC); // led
}

#ifdef SWIM_DEBUG

static void usart_setup(void)
{
  /* Enable GPIOA clock for USART1 */
  rcc_periph_clock_enable(RCC_GPIOA);

  /* Enable clocks for USART1. */
  rcc_periph_clock_enable(RCC_USART1);

  /* Setup GPIO pin GPIO_USART1_TX. */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

  /* Setup GPIO pin GPIO_USART1_RX  */
  // is eigenlijk de GPIO reset waarde, dus overbodig hier
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

  /* Setup UART parameters. */
  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(USART1);
} //usart_setup

int _write(int fd, char *ptr, int len);
int _write(int fd, char *ptr, int len) {
  int i = 0;

  /*
   * Write "len" of char from "ptr" to file id "fd"
   * Return number of char written.
   *
   * Only work for STDOUT, STDIN, and STDERR
   */
  if (fd > 2) {
    return -1;
  }
  while (*ptr && (i < len)) {
    usart_send_blocking(USART1, *ptr);
    if (*ptr == '\n') {
      usart_send_blocking(USART1, '\r');
    }
    i++;
    ptr++;
  }
  return i;
}
#endif // SWIM_DEBUG

static void stlink_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  (void)ep;

  int len = usbd_ep_read_packet(usbd_dev, 0x02, epBuffer, 64);
  #ifdef SWIM_DEBUG
    printf ("cb:%d-%d\n",epBuffer[0],epBuffer[1]);
    fflush(stdout);
  #endif
  switch (stlinkStatus.state) {
    case STLINK_STATE_CMD : // get a new command
      // TODO check if we get exactly 16 bytes
      switch (epBuffer[0]) {
        case STLINK_GET_VERSION :
          // version = [0]<<8 | [1]
          //version[b0..b5] = SWIM version (7)
          //version[b6..b11] = JTAG version (37)
          //version[b12..b15] = STLINK version (2)
          epBuffer[0] = 0x29; 
          epBuffer[1] = 0x47;
          epBuffer[2] = 0x83; // 2 bytes VID
          epBuffer[3] = 0x04;
          epBuffer[4] = 0x48; // 2 bytes PID
          epBuffer[5] = 0x37;
          usbd_ep_write_packet(usbd_dev, 0x81, epBuffer, 6);
          break;
        case STLINK_GET_CURRENT_MODE :
          memset(epBuffer,0,2);
          epBuffer[0] = stlinkStatus.mode;
          epBuffer[1] = 1; // TODO : what's this byte?
          usbd_ep_write_packet(usbd_dev, 0x81, epBuffer, 2);
          break;
        case STLINK_SWIM_COMMAND :
          switch (epBuffer[1]) { // swim subcommand
            case STLINK_SWIM_ENTER :
              stlinkStatus.mode = STLINK_DEV_SWIM_MODE;
              swim_init(true);
              break;
            case STLINK_SWIM_EXIT :
              swim_exit();
              break;
            case STLINK_SWIM_READ_CAP :
              // ignore for now, need to send 8 bytes, that's all i know
              memset(epBuffer,0,8);
              usbd_ep_write_packet(usbd_dev, 0x81, epBuffer, 8);
              break;
            case STLINK_SWIM_SPEED :
              swim_setHighSpeed(epBuffer[2] != 0);
              break;
            case STLINK_SWIM_ENTER_SEQ :
              swim_doEntrySequence();
              break;
            case STLINK_SWIM_GEN_RST : 
              swim_srst();
              break;
            case STLINK_SWIM_RESET : 
              swim_commsReset();
              break;
            case STLINK_SWIM_ASSERT_RESET :
              swim_assertReset();
              break;
            case STLINK_SWIM_DEASSERT_RESET :
              swim_deassertReset();
              break;
            case STLINK_SWIM_READSTATUS : {
              swimStatusAsync_t swimStatus;
              swim_readStatus(&swimStatus);
              switch (swimStatus.state) {
                case STATE_READY :
                  epBuffer[0] = STLINK_SWIM_OK;
                  break;
                case STATE_ERROR :
                  epBuffer[0] = STLINK_SWIM_NO_RESPONSE;
                  break;
                default :
                  epBuffer[0] = STLINK_SWIM_BUSY;
                  break;
              }
              epBuffer[1] = swimStatus.curBytes & 0xFF;
              epBuffer[2] = (swimStatus.curBytes >> 8) & 0xFF;
              epBuffer[3] = 0;
              usbd_ep_write_packet(usbd_dev, 0x81, epBuffer, 4);
              break;
            }
            case STLINK_SWIM_WRITEMEM :
              stlinkStatus.totalBytes = ((uint16_t)epBuffer[2] << 8) + epBuffer[3];
              stlinkStatus.address = (uint32_t)epBuffer[7]+ ((uint32_t)epBuffer[6]<<8);
              if (stlinkStatus.totalBytes > 8) { // more bytes to follow
                stlinkStatus.state = STLINK_STATE_WRITE;
                stlinkStatus.curBytes = 8; // maybe len-8 is safer, but len==16 always here
                memcpy(swimBuffer,epBuffer+8,8);
              }
              else { // we have all bytes, so can initiate the WOTF transaction
                memcpy(swimBuffer,epBuffer+8,stlinkStatus.totalBytes);
                swim_wotf(stlinkStatus.address, stlinkStatus.totalBytes,swimBuffer);
              }
              break;
            case STLINK_SWIM_READMEM : 
              stlinkStatus.totalBytes = ((uint16_t)epBuffer[2] << 8) + epBuffer[3];
              stlinkStatus.address = (uint32_t)epBuffer[7]+ ((uint32_t)epBuffer[6]<<8);
              stlinkStatus.curBytes = 0;
              swim_rotf(stlinkStatus.address,stlinkStatus.totalBytes,swimBuffer);
              break;
            case STLINK_SWIM_READBUF : {
              swimStatusAsync_t swimStatus;
              swim_readStatus(&swimStatus);
              if (swimStatus.state != STATE_READY)
                break; // not ready, so we don't return anything

              if (stlinkStatus.totalBytes <= 64) {
                usbd_ep_write_packet(usbd_dev, 0x81, swimBuffer, stlinkStatus.totalBytes);
              }
              else {
                usbd_ep_write_packet(usbd_dev, 0x81, swimBuffer, 64);
                stlinkStatus.state = STLINK_STATE_READ;
                stlinkStatus.curBytes = 64;
                // we haven't returned all data yet, we send the remainder from the main loop
              }
              break;
            }
            case STLINK_SWIM_READ_BUFFERSIZE : 
              memset(epBuffer,0,2);
              epBuffer[0] = SWIM_BUFFERSIZE & 0xFF;
              epBuffer[1] = (SWIM_BUFFERSIZE >> 8) & 0xFF;
              usbd_ep_write_packet(usbd_dev, 0x81, epBuffer, 2);
              break;
          }
          break;
        case STLINK_DFU_COMMAND : 
        default : 
          break;
      }
      break;
    case STLINK_STATE_READ :
      // for now we don't buffer commands while we are sending out data from a READBUF
      // or we could simply abort the READBUF operation?
      break;
    case STLINK_STATE_WRITE : 
      // we already received STLINK_SWIM_WRITEMEM, but expect more bytes to write
      memcpy(swimBuffer+stlinkStatus.curBytes,epBuffer,len);
      stlinkStatus.curBytes += len;
      if (stlinkStatus.curBytes >= stlinkStatus.totalBytes) { // we have all bytes, so can initiate the WOTF transaction
        stlinkStatus.state = STLINK_STATE_CMD;
        swim_wotf(stlinkStatus.address, stlinkStatus.totalBytes,swimBuffer);
      }
      break;
  }
} // stlink_data_rx_cb

static void stlink_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
  (void)wValue;
  (void)usbd_dev;

  usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
  usbd_ep_setup(usbd_dev, 0x02, USB_ENDPOINT_ATTR_BULK, 64, stlink_data_rx_cb);
  usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_BULK, 64, NULL); // TODO, geen idee hoe dit werkt bij STLINK
}

int main(void)
{
  usbd_device *usbd_dev;
  uint32_t blinkMillis;

  platform_init();
  platform_timing_init();

  // voor de blinkie
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_set(GPIOC, GPIO13);
  gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
  gpio_clear(GPIOC,GPIO13);


  usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
  usbd_register_set_config_callback(usbd_dev, stlink_set_config);
  gpio_clear(GPIOC, GPIO13);  

  #ifdef SWIM_DEBUG
    usart_setup();
    printf("UsbSwimProgrammer - STLINK SWIM\n");
  #endif

  while (1) {
    usbd_poll(usbd_dev);
    swim_update();

    if (stlinkStatus.state == STLINK_STATE_READ) {
      uint16_t numBytes;

      // fake read data
      numBytes = stlinkStatus.totalBytes - stlinkStatus.curBytes; // the number of bytes left to do
      if (numBytes > 64) numBytes = 64; // limit to 64 bytes 
      numBytes = usbd_ep_write_packet(usbd_dev, 0x81, swimBuffer+stlinkStatus.curBytes, numBytes);
      // numBytes = actual number sent to host, will be 0 if ep is busy
      stlinkStatus.curBytes += numBytes;
      if (stlinkStatus.curBytes >= stlinkStatus.totalBytes)
        stlinkStatus.state = STLINK_STATE_CMD; // read is finished
    }

    if ((platform_time_ms() - blinkMillis) > 500) {
      gpio_toggle(GPIOC,GPIO13);
      blinkMillis = platform_time_ms();
    }
  } 

  return 0;
}
