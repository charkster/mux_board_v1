/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Nathan Conrad
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#define DOTSTAR_DATA    PORT_PA01
#define DOTSTAR_CLK     PORT_PA00
#define DAC_PORT        PORT_PA02
#define ADC_PORT        PORT_PB08 // PORT B
#define MUX1_EN_PORT    PORT_PA17
#define MUX1_EN_PIN      PIN_PA17
#define MUX1_S0_PORT    PORT_PA07
#define MUX1_S0_PIN      PIN_PA07
#define MUX1_S1_PORT    PORT_PA21
#define MUX1_S1_PIN      PIN_PA21
#define MUX1_S2_PORT    PORT_PA10
#define MUX1_S2_PIN      PIN_PA10
#define MUX1_S3_PORT    PORT_PA11
#define MUX1_S3_PIN      PIN_PA11
#define MUX2_EN_PORT    PORT_PA19
#define MUX2_EN_PIN      PIN_PA19
#define MUX2_S0_PORT    PORT_PA08
#define MUX2_S0_PIN      PIN_PA08
#define MUX2_S1_PORT    PORT_PA09
#define MUX2_S1_PIN      PIN_PA09
#define MUX2_S2_PORT    PORT_PA30
#define MUX2_S2_PIN      PIN_PA30
#define MUX2_S3_PORT    PORT_PA31
#define MUX2_S3_PIN      PIN_PA31
#define MUX3_EN_PORT    PORT_PA16
#define MUX3_EN_PIN      PIN_PA16
#define MUX3_S0_PORT    PORT_PA14
#define MUX3_S0_PIN      PIN_PA14
#define MUX3_S1_PORT    PORT_PA12
#define MUX3_S1_PIN      PIN_PA12
#define MUX3_S2_PORT    PORT_PB10 // PORT B
#define MUX3_S2_PIN      PIN_PB10
#define MUX3_S3_PORT    PORT_PB11 // PORT B
#define MUX3_S3_PIN      PIN_PB11
#define MUX4_EN_PORT    PORT_PA18
#define MUX4_EN_PIN      PIN_PA18
#define MUX4_S0_PORT    PORT_PB02 // PORT B
#define MUX4_S0_PIN      PIN_PB02
#define MUX4_S1_PORT    PORT_PA05
#define MUX4_S1_PIN      PIN_PA05
#define MUX4_S2_PORT    PORT_PA04
#define MUX4_S2_PIN      PIN_PA04
#define MUX4_S3_PORT    PORT_PB09 // PORT B
#define MUX4_S3_PIN      PIN_PB09
#define LS_EN_PORT      PORT_PA03
#define LS_EN_PIN        PIN_PA03
#define GPIO1_PORT      PORT_PB02 // PORT B, same as MUX4_S0_PORT
#define GPIO1_PIN        PIN_PB02
#define GPIO2_PORT      PORT_PA05 //         same as MUX4_S1_PORT
#define GPIO2_PIN        PIN_PA05
#define GPIO3_PORT      PORT_PA04 //         same as MUX4_S2_PORT
#define GPIO3_PIN        PIN_PA04
#define GPIO4_PORT      PORT_PB09 // PORT B, same as MUX4_S3_PORT
#define GPIO4_PIN        PIN_PB09
#define GPIO5_PORT      PORT_PA14 //         same as MUX3_S0_PORT
#define GPIO5_PIN        PIN_PA14
#define GPIO6_PORT      PORT_PA12 //         same as MUX3_S1_PORT
#define GPIO6_PIN        PIN_PA12
#define SDA_PORT        PORT_PA22
#define SCL_PORT        PORT_PB11  // PORT B
#define DAC_MAX_VALUE   1024 // 10bit value
#define DAC_REF_VOLTAGE 3.3
#define ADC_MAX_VALUE   4096 // 12bit value
#define ADC_REF_VOLTAGE 3.3
#define IDN             "MUX1:SEL 16\nMUX1:EN 1\nhttps://github.com/charkster/mux_board_v1"
#define IDN_QUERY       "*idn?"
#define RST_CMD         "*rst"
#define DAC_CMD         "sourc1:volt:lev " // SOURCe1:VOLTage:LEVel
#define DAC_QUERY       "sourc1:volt:lev?" // SOURCe1:VOLTage:LEVel
#define ADC_QUERY       "sens1:volt?"      // SENSe1:VOLTage
#define MUX1_SEL_CMD    "mux1:sel "        // MUX1:SEL
#define MUX1_SEL_QUERY  "mux1:sel?"        // MUX1:SEL
#define MUX1_EN_CMD     "mux1:en "         // MUX1:ENable
#define MUX1_EN_QUERY   "mux1:en?"         // MUX1:ENable
#define MUX2_SEL_CMD    "mux2:sel "        // MUX2:SEL
#define MUX2_SEL_QUERY  "mux2:sel?"        // MUX2:SEL
#define MUX2_EN_CMD     "mux2:en "         // MUX2:ENable
#define MUX2_EN_QUERY   "mux2:en?"         // MUX2:ENable
#define MUX3_SEL_CMD    "mux3:sel "        // MUX3:SEL
#define MUX3_SEL_QUERY  "mux3:sel?"        // MUX3:SEL
#define MUX3_EN_CMD     "mux3:en "         // MUX3:ENable
#define MUX3_EN_QUERY   "mux3:en?"         // MUX3:ENable
#define MUX4_SEL_CMD    "mux4:sel "        // MUX4:SEL
#define MUX4_SEL_QUERY  "mux4:sel?"        // MUX4:SEL
#define MUX4_EN_CMD     "mux4:en "         // MUX4:ENable
#define MUX4_EN_QUERY   "mux4:en?"         // MUX4:ENable
#define GPIO1_LEV_CMD    "gpio1:lev "      // GPIO1:LEVel
#define GPIO1_LEV_QUERY  "gpio1:lev?"      // GPIO1:LEVel
#define GPIO1_DIR_CMD    "gpio1:dir "      // GPIO1:DIRection
#define GPIO1_DIR_QUERY  "gpio1:dir?"      // GPIO1:DIRection
#define GPIO2_LEV_CMD    "gpio2:lev "      // GPIO2:LEVel
#define GPIO2_LEV_QUERY  "gpio2:lev?"      // GPIO2:LEVel
#define GPIO2_DIR_CMD    "gpio2:dir "      // GPIO2:DIRection
#define GPIO2_DIR_QUERY  "gpio2:dir?"      // GPIO2:DIRection
#define GPIO3_LEV_CMD    "gpio3:lev "      // GPIO3:LEVel
#define GPIO3_LEV_QUERY  "gpio3:lev?"      // GPIO3:LEVel
#define GPIO3_DIR_CMD    "gpio3:dir "      // GPIO3:DIRection
#define GPIO3_DIR_QUERY  "gpio3:dir?"      // GPIO3:DIRection
#define GPIO4_LEV_CMD    "gpio4:lev "      // GPIO4:LEVel
#define GPIO4_LEV_QUERY  "gpio4:lev?"      // GPIO4:LEVel
#define GPIO4_DIR_CMD    "gpio4:dir "      // GPIO4:DIRection
#define GPIO4_DIR_QUERY  "gpio4:dir?"      // GPIO4:DIRection
#define GPIO5_LEV_CMD    "gpio5:lev "      // GPIO5:LEVel
#define GPIO5_LEV_QUERY  "gpio5:lev?"      // GPIO5:LEVel
#define GPIO5_DIR_CMD    "gpio5:dir "      // GPIO5:DIRection
#define GPIO5_DIR_QUERY  "gpio5:dir?"      // GPIO5:DIRection
#define GPIO6_LEV_CMD    "gpio6:lev "      // GPIO6:LEVel
#define GPIO6_LEV_QUERY  "gpio6:lev?"      // GPIO6:LEVel
#define GPIO6_DIR_CMD    "gpio6:dir "      // GPIO6:DIRection
#define GPIO6_DIR_QUERY  "gpio6:dir?"      // GPIO6:DIRection
#define END_RESPONSE    "\n"               // USB488

#include <strings.h>
#include <stdlib.h>     /* atoi */
#include <stdio.h>      /* fprintf */
#include "tusb.h"
#include "bsp/board.h"
#include "main.h"
#include "sam.h" /* ADC, DAC, GPIO */

char * get_value(char *in_string);
uint32_t adc_get_sample(void);
void ftoa(float num, char *str);

#if (CFG_TUD_USBTMC_ENABLE_488)
static usbtmc_response_capabilities_488_t const
#else
static usbtmc_response_capabilities_t const
#endif
tud_usbtmc_app_capabilities  =
{
    .USBTMC_status = USBTMC_STATUS_SUCCESS,
    .bcdUSBTMC = USBTMC_VERSION,
    .bmIntfcCapabilities =
    {
        .listenOnly = 0,
        .talkOnly = 0,
        .supportsIndicatorPulse = 1
    },
    .bmDevCapabilities = {
        .canEndBulkInOnTermChar = 0
    },

#if (CFG_TUD_USBTMC_ENABLE_488)
    .bcdUSB488 = USBTMC_488_VERSION,
    .bmIntfcCapabilities488 =
    {
        .supportsTrigger = 1,
        .supportsREN_GTL_LLO = 0,
        .is488_2 = 1
    },
    .bmDevCapabilities488 =
    {
      .SCPI = 1,
      .SR1 = 0,
      .RL1 = 0,
      .DT1 =0,
    }
#endif
};

#define IEEE4882_STB_QUESTIONABLE (0x08u)
#define IEEE4882_STB_MAV          (0x10u)
#define IEEE4882_STB_SER          (0x20u)
#define IEEE4882_STB_SRQ          (0x40u)

//static const char idn[] = "TinyUSB,Seeeduino Xiao,v1\r\n";
static volatile uint8_t status;

// 0=not query, 1=queried, 2=delay,set(MAV), 3=delay 4=ready?
// (to simulate delay)
static volatile uint16_t queryState = 0;
static volatile uint32_t queryDelayStart;
static volatile uint32_t bulkInStarted;

static volatile bool idnQuery;
static volatile bool rst_cmd;
static volatile bool dac_cmd;
static volatile bool dac_query;
static volatile bool adc_query;
static volatile bool mux_sel_cmd;
static volatile bool mux_sel_query;
static volatile bool mux_en_cmd;
static volatile bool mux_en_query;
static volatile bool gpio_lev_cmd;
static volatile bool gpio_lev_query;
static volatile bool gpio_dir_cmd;
static volatile bool gpio_dir_query;


static uint32_t resp_delay = 125u; // Adjustable delay, to allow for better testing
static size_t   buffer_len;
static size_t   buffer_tx_ix;      // for transmitting using multiple transfers
static uint8_t  buffer[225];       // A few packets long should be enough.

char adc_voltage_str[10];
char dac_voltage_str[10];
char mux_en_str[2];
char mux_sel_str[5];
char gpio_lev_str[2];
char gpio_dir_str[5];

static uint32_t  MUX_S0_PORT;
static uint32_t  MUX_S1_PORT;
static uint32_t  MUX_S2_PORT;
static uint32_t  MUX_S3_PORT;
static uint32_t  MUX_EN_PORT;

static uint32_t  MUX_S0_PIN;
static uint32_t  MUX_S1_PIN;
static uint32_t  MUX_S2_PIN;
static uint32_t  MUX_S3_PIN;
static uint32_t  MUX_EN_PIN;

static uint32_t  GPIO_PORT;
static uint32_t  GPIO_PIN;

static usbtmc_msg_dev_dep_msg_in_header_t rspMsg = {
    .bmTransferAttributes =
    {
      .EOM = 1,
      .UsingTermChar = 0
    }
};

void tud_usbtmc_open_cb(uint8_t interface_id)
{
  (void)interface_id;
  tud_usbtmc_start_bus_read();
}

#if (CFG_TUD_USBTMC_ENABLE_488)
usbtmc_response_capabilities_488_t const *
#else
usbtmc_response_capabilities_t const *
#endif
tud_usbtmc_get_capabilities_cb()
{
  return &tud_usbtmc_app_capabilities;
}


bool tud_usbtmc_msg_trigger_cb(usbtmc_msg_generic_t* msg) {
  (void)msg;
  // Let trigger set the SRQ
  status |= IEEE4882_STB_SRQ;
  return true;
}

bool tud_usbtmc_msgBulkOut_start_cb(usbtmc_msg_request_dev_dep_out const * msgHeader)
{
  (void)msgHeader;
  buffer_len = 0;
  if(msgHeader->TransferSize > sizeof(buffer))
  {

    return false;
  }
  return true;
}

bool tud_usbtmc_msg_data_cb(void *data, size_t len, bool transfer_complete)
{
  // If transfer isn't finished, we just ignore it (for now)

  if(len + buffer_len < sizeof(buffer))
  {
    memcpy(&(buffer[buffer_len]), data, len);
    buffer_len += len;
  }
  else
  {
    return false; // buffer overflow!
  }

  queryState     = transfer_complete;
  idnQuery       = false;
  rst_cmd        = false;
  dac_cmd        = false;
  dac_query      = false;
  adc_query      = false;
  mux_sel_cmd    = false;
  mux_sel_query  = false;
  mux_en_cmd     = false;
  mux_en_query   = false;
  gpio_lev_cmd   = false;
  gpio_lev_query = false;
  gpio_dir_cmd   = false;
  gpio_dir_query = false;

  if(transfer_complete && (len >=4) && !strncasecmp(IDN_QUERY,data,5))
  {
    idnQuery = true;
  }
  else if (transfer_complete && (len >=4) && !strncasecmp(RST_CMD,data,4))
  {
    rst_cmd = true;
    DAC->DATA.reg = 0x0000;                  // clear DAC value
    PORT->Group[MUX1_EN_PIN/32].DIRSET.reg = MUX1_EN_PORT; // mux enable as output
    PORT->Group[MUX1_EN_PIN/32].OUTCLR.reg = MUX1_EN_PORT; // mux_enable drive low value
    PORT->Group[MUX2_EN_PIN/32].DIRSET.reg = MUX2_EN_PORT; // mux enable as output
    PORT->Group[MUX2_EN_PIN/32].OUTCLR.reg = MUX2_EN_PORT; // mux_enable drive low value
    PORT->Group[MUX3_EN_PIN/32].DIRSET.reg = MUX3_EN_PORT; // mux enable as output
    PORT->Group[MUX3_EN_PIN/32].OUTCLR.reg = MUX3_EN_PORT; // mux_enable drive low value
    PORT->Group[MUX4_EN_PIN/32].DIRSET.reg = MUX4_EN_PORT; // mux enable as output
    PORT->Group[MUX4_EN_PIN/32].OUTCLR.reg = MUX4_EN_PORT; // mux_enable drive low value
  }
  else if (transfer_complete && (len >=16) && !strncasecmp(DAC_CMD,data,16))
  {
    dac_cmd            = true;
    char *ptr_value    = get_value(data);
    float dac_voltage  = strtof(ptr_value,NULL);
    uint16_t dac_value = (int)( (dac_voltage / DAC_REF_VOLTAGE) * DAC_MAX_VALUE );
    DAC->DATA.reg = dac_value;
  }
  else if (transfer_complete && (len >= 16) && !strncasecmp(DAC_QUERY,data,16))
  {
    dac_query = true;
    float dac_voltage = (float)(DAC->DATA.reg) * (DAC_REF_VOLTAGE / DAC_MAX_VALUE);
    ftoa(dac_voltage,dac_voltage_str);
//    strcat(dac_voltage_str,"\n");
  }
  else if (transfer_complete && (len >= 11) && !strncasecmp(ADC_QUERY,data,11))
  {
    adc_query         = true;
    float adc_voltage =(float)(adc_get_sample()) / ADC_MAX_VALUE * ADC_REF_VOLTAGE;
    ftoa(adc_voltage,adc_voltage_str);
  }
  else if (transfer_complete && (len >= 8) && !strncasecmp(MUX1_EN_CMD,data,8))
  {
    mux_en_cmd  = true;
    MUX_EN_PORT = MUX1_EN_PORT;
    MUX_EN_PIN  = MUX1_EN_PIN;
    PORT->Group[0].DIRSET.reg = MUX1_EN_PORT | MUX1_S3_PORT | MUX1_S2_PORT | MUX1_S1_PORT | MUX1_S0_PORT; // mux enable and sel as outputs
  }
  else if (transfer_complete && (len >= 8) && !strncasecmp(MUX2_EN_CMD,data,8))
  {
    mux_en_cmd  = true;
    MUX_EN_PORT = MUX2_EN_PORT;
    MUX_EN_PIN  = MUX2_EN_PIN;
    PORT->Group[0].DIRSET.reg = MUX2_EN_PORT | MUX2_S3_PORT | MUX2_S2_PORT | MUX2_S1_PORT | MUX2_S0_PORT; // mux enable and sel as outputs
  }
  else if (transfer_complete && (len >= 8) && !strncasecmp(MUX3_EN_CMD,data,8))
  {
    mux_en_cmd  = true;
    MUX_EN_PORT = MUX3_EN_PORT;
    MUX_EN_PIN  = MUX3_EN_PIN;
    PORT->Group[0].DIRSET.reg = MUX3_EN_PORT | MUX3_S3_PORT | MUX3_S2_PORT | MUX3_S1_PORT | MUX3_S0_PORT; // mux enable and sel as outputs
  }
  else if (transfer_complete && (len >= 8) && !strncasecmp(MUX4_EN_CMD,data,8))
  {
    mux_en_cmd  = true;
    MUX_EN_PORT = MUX4_EN_PORT;
    MUX_EN_PIN  = MUX4_EN_PIN;
    PORT->Group[0].DIRSET.reg = MUX4_EN_PORT | MUX4_S3_PORT | MUX4_S2_PORT | MUX4_S1_PORT | MUX4_S0_PORT; // mux enable and sel as outputs
  }
  else if (transfer_complete && (len >= 8) && !strncasecmp(MUX1_EN_QUERY,data,8))
  {
    mux_en_query = true;
    MUX_EN_PORT  = MUX1_EN_PORT;
    MUX_EN_PIN   = MUX1_EN_PIN;
  }
  else if (transfer_complete && (len >= 8) && !strncasecmp(MUX2_EN_QUERY,data,8))
  {
    mux_en_query = true;
    MUX_EN_PORT  = MUX2_EN_PORT;
    MUX_EN_PIN   = MUX2_EN_PIN;
  }
  else if (transfer_complete && (len >= 8) && !strncasecmp(MUX3_EN_QUERY,data,8))
  {
    mux_en_query = true;
    MUX_EN_PORT  = MUX3_EN_PORT;
    MUX_EN_PIN   = MUX3_EN_PIN;
  }
  else if (transfer_complete && (len >= 8) && !strncasecmp(MUX4_EN_QUERY,data,8))
  {
    mux_en_query = true;
    MUX_EN_PORT  = MUX4_EN_PORT;
    MUX_EN_PIN   = MUX4_EN_PIN;
  }
  else if (transfer_complete && (len >= 9) && !strncasecmp(MUX1_SEL_CMD,data,9))
  {
    mux_sel_cmd = true;
    MUX_S0_PORT  = MUX1_S0_PORT;
    MUX_S1_PORT  = MUX1_S1_PORT;
    MUX_S2_PORT  = MUX1_S2_PORT;
    MUX_S3_PORT  = MUX1_S3_PORT;
    MUX_S0_PIN   = MUX1_S0_PIN;
    MUX_S1_PIN   = MUX1_S1_PIN;
    MUX_S2_PIN   = MUX1_S2_PIN;
    MUX_S3_PIN   = MUX1_S3_PIN;
  }
  else if (transfer_complete && (len >= 9) && !strncasecmp(MUX2_SEL_CMD,data,9))
  {
    mux_sel_cmd = true;
    MUX_S0_PORT  = MUX2_S0_PORT;
    MUX_S1_PORT  = MUX2_S1_PORT;
    MUX_S2_PORT  = MUX2_S2_PORT;
    MUX_S3_PORT  = MUX2_S3_PORT;
    MUX_S0_PIN   = MUX2_S0_PIN;
    MUX_S1_PIN   = MUX2_S1_PIN;
    MUX_S2_PIN   = MUX2_S2_PIN;
    MUX_S3_PIN   = MUX2_S3_PIN;
  }
  else if (transfer_complete && (len >= 9) && !strncasecmp(MUX3_SEL_CMD,data,9))
  {
    mux_sel_cmd = true;
    MUX_S0_PORT  = MUX3_S0_PORT;
    MUX_S1_PORT  = MUX3_S1_PORT;
    MUX_S2_PORT  = MUX3_S2_PORT;
    MUX_S3_PORT  = MUX3_S3_PORT;
    MUX_S0_PIN   = MUX3_S0_PIN;
    MUX_S1_PIN   = MUX3_S1_PIN;
    MUX_S2_PIN   = MUX3_S2_PIN;
    MUX_S3_PIN   = MUX3_S3_PIN;
    PORT->Group[(LS_EN_PIN/32)].OUTCLR.reg = LS_EN_PORT;   // disable the level-shifters
  }
  else if (transfer_complete && (len >= 9) && !strncasecmp(MUX4_SEL_CMD,data,9))
  {
    mux_sel_cmd = true;
    MUX_S0_PORT  = MUX4_S0_PORT;
    MUX_S1_PORT  = MUX4_S1_PORT;
    MUX_S2_PORT  = MUX4_S2_PORT;
    MUX_S3_PORT  = MUX4_S3_PORT;
    MUX_S0_PIN   = MUX4_S0_PIN;
    MUX_S1_PIN   = MUX4_S1_PIN;
    MUX_S2_PIN   = MUX4_S2_PIN;
    MUX_S3_PIN   = MUX4_S3_PIN;
    PORT->Group[(LS_EN_PIN/32)].OUTCLR.reg = LS_EN_PORT;   // disable the level-shifters
  }
  else if (transfer_complete && (len >= 9) && !strncasecmp(MUX1_SEL_QUERY,data,9))
  {
    mux_sel_query = true;
    MUX_S0_PORT  = MUX1_S0_PORT;
    MUX_S1_PORT  = MUX1_S1_PORT;
    MUX_S2_PORT  = MUX1_S2_PORT;
    MUX_S3_PORT  = MUX1_S3_PORT;
    MUX_S0_PIN   = MUX1_S0_PIN;
    MUX_S1_PIN   = MUX1_S1_PIN;
    MUX_S2_PIN   = MUX1_S2_PIN;
    MUX_S3_PIN   = MUX1_S3_PIN;
  }
  else if (transfer_complete && (len >= 9) && !strncasecmp(MUX2_SEL_QUERY,data,9))
  {
    mux_sel_query = true;
    MUX_S0_PORT  = MUX2_S0_PORT;
    MUX_S1_PORT  = MUX2_S1_PORT;
    MUX_S2_PORT  = MUX2_S2_PORT;
    MUX_S3_PORT  = MUX2_S3_PORT;
    MUX_S0_PIN   = MUX2_S0_PIN;
    MUX_S1_PIN   = MUX2_S1_PIN;
    MUX_S2_PIN   = MUX2_S2_PIN;
    MUX_S3_PIN   = MUX2_S3_PIN;
  }
  else if (transfer_complete && (len >= 9) && !strncasecmp(MUX3_SEL_QUERY,data,9))
  {
    mux_sel_query = true;
    MUX_S0_PORT  = MUX3_S0_PORT;
    MUX_S1_PORT  = MUX3_S1_PORT;
    MUX_S2_PORT  = MUX3_S2_PORT;
    MUX_S3_PORT  = MUX3_S3_PORT;
    MUX_S0_PIN   = MUX3_S0_PIN;
    MUX_S1_PIN   = MUX3_S1_PIN;
    MUX_S2_PIN   = MUX3_S2_PIN;
    MUX_S3_PIN   = MUX3_S3_PIN;
  }
  else if (transfer_complete && (len >= 9) && !strncasecmp(MUX4_SEL_QUERY,data,9))
  {
    mux_sel_query = true;
    MUX_S0_PORT  = MUX4_S0_PORT;
    MUX_S1_PORT  = MUX4_S1_PORT;
    MUX_S2_PORT  = MUX4_S2_PORT;
    MUX_S3_PORT  = MUX4_S3_PORT;
    MUX_S0_PIN   = MUX4_S0_PIN;
    MUX_S1_PIN   = MUX4_S1_PIN;
    MUX_S2_PIN   = MUX4_S2_PIN;
    MUX_S3_PIN   = MUX4_S3_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO1_LEV_CMD,data,10))
  {
    gpio_lev_cmd = true;
    GPIO_PORT    = GPIO1_PORT;
    GPIO_PIN     = GPIO1_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO2_LEV_CMD,data,10))
  {
    gpio_lev_cmd = true;
    GPIO_PORT    = GPIO2_PORT;
    GPIO_PIN     = GPIO2_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO3_LEV_CMD,data,10))
  {
    gpio_lev_cmd = true;
    GPIO_PORT    = GPIO3_PORT;
    GPIO_PIN     = GPIO3_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO4_LEV_CMD,data,10))
  {
    gpio_lev_cmd = true;
    GPIO_PORT    = GPIO4_PORT;
    GPIO_PIN     = GPIO4_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO5_LEV_CMD,data,10))
  {
    gpio_lev_cmd = true;
    GPIO_PORT    = GPIO5_PORT;
    GPIO_PIN     = GPIO5_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO6_LEV_CMD,data,10))
  {
    gpio_lev_cmd = true;
    GPIO_PORT    = GPIO6_PORT;
    GPIO_PIN     = GPIO6_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO1_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;
    GPIO_PORT      = GPIO1_PORT;
    GPIO_PIN       = GPIO1_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO2_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;
    GPIO_PORT      = GPIO2_PORT;
    GPIO_PIN       = GPIO2_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO3_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;
    GPIO_PORT      = GPIO3_PORT;
    GPIO_PIN       = GPIO3_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO4_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;
    GPIO_PORT      = GPIO4_PORT;
    GPIO_PIN       = GPIO4_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO5_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;
    GPIO_PORT      = GPIO5_PORT;
    GPIO_PIN       = GPIO5_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO6_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;
    GPIO_PORT     = GPIO6_PORT;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO1_DIR_CMD,data,10))
  {
    gpio_dir_cmd = true;
    GPIO_PORT    = GPIO1_PORT;
    GPIO_PIN     = GPIO1_PIN;
    PORT->Group[(MUX4_EN_PIN/32)].OUTCLR.reg = MUX4_EN_PORT; // disable MUX as select lines shared with gpio
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO2_DIR_CMD,data,10))
  {
    gpio_dir_cmd = true;
    GPIO_PORT    = GPIO2_PORT;
    GPIO_PIN     = GPIO2_PIN;
    PORT->Group[(MUX4_EN_PIN/32)].OUTCLR.reg = MUX4_EN_PORT; // disable MUX as select lines shared with gpio
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO3_DIR_CMD,data,10))
  {
    gpio_dir_cmd = true;
    GPIO_PORT    = GPIO3_PORT;
    GPIO_PIN     = GPIO3_PIN;
    PORT->Group[(MUX4_EN_PIN/32)].OUTCLR.reg = MUX4_EN_PORT; // disable MUX as select lines shared with gpio
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO4_DIR_CMD,data,10))
  {
    gpio_dir_cmd = true;
    GPIO_PORT    = GPIO4_PORT;
    GPIO_PIN     = GPIO4_PIN;
    PORT->Group[(MUX4_EN_PIN/32)].OUTCLR.reg = MUX4_EN_PORT; // disable MUX as select lines shared with gpio
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO5_DIR_CMD,data,10))
  {
    gpio_dir_cmd = true;
    GPIO_PORT    = GPIO5_PORT;
    GPIO_PIN     = GPIO5_PIN;
    PORT->Group[(MUX3_EN_PIN/32)].OUTCLR.reg = MUX3_EN_PORT; // disable MUX as select lines shared with gpio
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO6_DIR_CMD,data,10))
  {
    gpio_dir_cmd = true;
    GPIO_PORT    = GPIO6_PORT;
    GPIO_PIN     = GPIO6_PIN;
    PORT->Group[(MUX3_EN_PIN/32)].OUTCLR.reg = MUX3_EN_PORT; // disable MUX as select lines shared with gpio
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO1_DIR_QUERY,data,10))
  {
    gpio_dir_query = true;
    GPIO_PORT      = GPIO1_PORT;
    GPIO_PIN       = GPIO1_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO2_DIR_QUERY,data,10))
  {
    gpio_dir_query = true;
    GPIO_PORT      = GPIO2_PORT;
    GPIO_PIN       = GPIO2_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO3_DIR_QUERY,data,10))
  {
    gpio_dir_query = true;
    GPIO_PORT      = GPIO3_PORT;
    GPIO_PIN       = GPIO3_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO4_DIR_QUERY,data,10))
  {
    gpio_dir_query = true;
    GPIO_PORT      = GPIO4_PORT;
    GPIO_PIN       = GPIO4_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO5_DIR_QUERY,data,10))
  {
    gpio_dir_query = true;
    GPIO_PORT      = GPIO5_PORT;
    GPIO_PIN       = GPIO5_PIN;
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO6_DIR_QUERY,data,10))
  {
    gpio_dir_query = true;
    GPIO_PORT     = GPIO6_PORT;
    GPIO_PIN      = GPIO6_PIN;
  }


  if (mux_en_cmd)
  {
    char *ptr_value = get_value(data);
    int mux_en = atoi(ptr_value);
    if (mux_en == 1)
    {
      PORT->Group[0].OUTSET.reg = MUX_EN_PORT; // drive high value
    }
    else if (mux_en == 0)
    {
      PORT->Group[0].OUTCLR.reg = MUX_EN_PORT; // drive low value
    }
  }
  else if (mux_en_query)
  {
//    if (PORT->Group[0].OUT.reg & MUX_EN_PORT)
    if (PORT->Group[(MUX_EN_PIN/32)].OUT.reg & MUX_EN_PORT)
    {
      strcpy(mux_en_str,"1");
    }
    else
    {
      strcpy(mux_en_str,"0");
    }
  }
  else if (mux_sel_cmd) 
  {
    char *ptr_value = get_value(data);
    PORT->Group[MUX_S3_PIN/32].DIRSET.reg = MUX_S3_PORT;
    PORT->Group[MUX_S2_PIN/32].DIRSET.reg = MUX_S2_PORT;
    PORT->Group[MUX_S1_PIN/32].DIRSET.reg = MUX_S1_PORT;
    PORT->Group[MUX_S0_PIN/32].DIRSET.reg = MUX_S0_PORT;
    
    if (!strncasecmp("10",ptr_value,2))  // 1001
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTSET.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTCLR.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTCLR.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTSET.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("11",ptr_value,2))  // 1010
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTSET.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTCLR.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTSET.reg =  MUX_S1_PORT; 
      PORT->Group[(MUX_S0_PIN/32)].OUTCLR.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("12",ptr_value,2))  // 1011
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTSET.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTCLR.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTSET.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTSET.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("13",ptr_value,2))  // 1100
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTSET.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTSET.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTCLR.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTCLR.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("14",ptr_value,2))  // 1101
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTSET.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTSET.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTCLR.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTSET.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("15",ptr_value,2)) // 1110
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTSET.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTSET.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTSET.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTCLR.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("16",ptr_value,2))  // 1111
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTSET.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTSET.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTSET.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTSET.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("1",ptr_value,1))  // 0000
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTCLR.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTCLR.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTCLR.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTCLR.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("2",ptr_value,1))  // 0001
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTCLR.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTCLR.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTCLR.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTSET.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("3",ptr_value,1))  // 0010
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTCLR.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTCLR.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTSET.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTCLR.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("4",ptr_value,1)) //0011
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTCLR.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTCLR.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTSET.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTSET.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("5",ptr_value,1)) // 0100
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTCLR.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTSET.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTCLR.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTCLR.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("6",ptr_value,1))  // 0101
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTCLR.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTSET.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTCLR.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTSET.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("7",ptr_value,1))  // 0110
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTCLR.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTSET.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTSET.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTCLR.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("8",ptr_value,1))  // 0111
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTCLR.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTSET.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTSET.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTSET.reg =  MUX_S0_PORT;
    }
    else if (!strncasecmp("9",ptr_value,1))  // 1000
    {
      PORT->Group[(MUX_S3_PIN/32)].OUTSET.reg =  MUX_S3_PORT;
      PORT->Group[(MUX_S2_PIN/32)].OUTCLR.reg =  MUX_S2_PORT;
      PORT->Group[(MUX_S1_PIN/32)].OUTCLR.reg =  MUX_S1_PORT;
      PORT->Group[(MUX_S0_PIN/32)].OUTCLR.reg =  MUX_S0_PORT;
    }
  }
  else if (mux_sel_query) 
  {
    if ((PORT->Group[(MUX_S3_PIN/32)].OUT.reg & MUX_S3_PORT) && (PORT->Group[(MUX_S2_PIN/32)].OUT.reg & MUX_S2_PORT) &&  (PORT->Group[(MUX_S1_PIN/32)].OUT.reg & MUX_S1_PORT) &&  (PORT->Group[(MUX_S0_PIN/32)].OUT.reg & MUX_S0_PORT))
    {
      strcpy(mux_sel_str,"16");
    }
    else if ((PORT->Group[(MUX_S3_PIN/32)].OUT.reg & MUX_S3_PORT) && (PORT->Group[(MUX_S2_PIN/32)].OUT.reg & MUX_S2_PORT) &&  (PORT->Group[(MUX_S1_PIN/32)].OUT.reg & MUX_S1_PORT))
    {
      strcpy(mux_sel_str,"15");
    }
    else if ((PORT->Group[(MUX_S3_PIN/32)].OUT.reg & MUX_S3_PORT) && (PORT->Group[(MUX_S2_PIN/32)].OUT.reg & MUX_S2_PORT) &&  (PORT->Group[(MUX_S0_PIN/32)].OUT.reg & MUX_S0_PORT))
    {
      strcpy(mux_sel_str,"14");
    }
    else if ((PORT->Group[(MUX_S3_PIN/32)].OUT.reg & MUX_S3_PORT) && (PORT->Group[(MUX_S2_PIN/32)].OUT.reg & MUX_S2_PORT))
    {
      strcpy(mux_sel_str,"13");
    }
    else if ((PORT->Group[(MUX_S3_PIN/32)].OUT.reg & MUX_S3_PORT) &&  (PORT->Group[(MUX_S1_PIN/32)].OUT.reg & MUX_S1_PORT) &&  (PORT->Group[(MUX_S0_PIN/32)].OUT.reg & MUX_S0_PORT))
    {
      strcpy(mux_sel_str,"12");
    }
    else if ((PORT->Group[(MUX_S3_PIN/32)].OUT.reg & MUX_S3_PORT) &&  (PORT->Group[(MUX_S1_PIN/32)].OUT.reg & MUX_S1_PORT))
    {
      strcpy(mux_sel_str,"11");
    }
    else if ((PORT->Group[(MUX_S3_PIN/32)].OUT.reg & MUX_S3_PORT) &&  (PORT->Group[(MUX_S0_PIN/32)].OUT.reg & MUX_S0_PORT))
    {
      strcpy(mux_sel_str,"10");
    }
    else if ((PORT->Group[(MUX_S3_PIN/32)].OUT.reg & MUX_S3_PORT))
    {
      strcpy(mux_sel_str,"9");
    }
    else if ((PORT->Group[(MUX_S2_PIN/32)].OUT.reg & MUX_S2_PORT) &&  (PORT->Group[(MUX_S1_PIN/32)].OUT.reg & MUX_S1_PORT) &&  (PORT->Group[(MUX_S0_PIN/32)].OUT.reg & MUX_S0_PORT))
    {
      strcpy(mux_sel_str,"8");
    }
    else if ((PORT->Group[(MUX_S2_PIN/32)].OUT.reg & MUX_S2_PORT) &&  (PORT->Group[(MUX_S1_PIN/32)].OUT.reg & MUX_S1_PORT))
    {
      strcpy(mux_sel_str,"7");
    }
    else if ((PORT->Group[(MUX_S2_PIN/32)].OUT.reg & MUX_S2_PORT) &&  (PORT->Group[(MUX_S0_PIN/32)].OUT.reg & MUX_S0_PORT))
    {
      strcpy(mux_sel_str,"6");
    }
    else if ((PORT->Group[(MUX_S2_PIN/32)].OUT.reg & MUX_S2_PORT))
    {
      strcpy(mux_sel_str,"5");
    }
    else if ((PORT->Group[(MUX_S1_PIN/32)].OUT.reg & MUX_S1_PORT) &&  (PORT->Group[(MUX_S0_PIN/32)].OUT.reg & MUX_S0_PORT))
    {
      strcpy(mux_sel_str,"4");
    }
    else if ((PORT->Group[(MUX_S1_PIN/32)].OUT.reg & MUX_S1_PORT))
    {
      strcpy(mux_sel_str,"3");
    }
    else if ((PORT->Group[(MUX_S0_PIN/32)].OUT.reg & MUX_S0_PORT))
    {
      strcpy(mux_sel_str,"2");
    }
    else
    {
      strcpy(mux_sel_str,"1");
    }
  }
  else if (gpio_lev_cmd)
  {
    char *ptr_value = get_value(data);
    int gpio_level = atoi(ptr_value);
    PORT->Group[(LS_EN_PIN/32)].OUTSET.reg = LS_EN_PORT; // enable the level-shifters
    if (gpio_level == 1)
    {
      PORT->Group[0].OUTSET.reg = GPIO_PORT; // drive high value
    }
    else if (gpio_level == 0)
    {
      PORT->Group[0].OUTCLR.reg = GPIO_PORT; // drive low value
    }
  }
  else if (gpio_lev_query)
  {
    if ((PORT->Group[0].IN.reg & GPIO_PORT) || ((PORT->Group[0].DIR.reg & GPIO_PORT) && (PORT->Group[0].OUT.reg & GPIO_PORT)))
    {
      strcpy(gpio_lev_str,"1");
    }
    else
    {
      strcpy(gpio_lev_str,"0");
    }
  }
  else if (gpio_dir_cmd)
  {
    char *ptr_value = get_value(data);
    PORT->Group[(LS_EN_PIN/32)].OUTSET.reg = LS_EN_PORT; // enable the level-shifters
    if (!strncasecmp("IN",ptr_value,2))
    {
      PORT->Group[0].DIRCLR.reg = GPIO_PORT; // pin as input
    }
    else if (!strncasecmp("OUT",ptr_value,3))
    {
      PORT->Group[0].DIRSET.reg = GPIO_PORT; // pin as output
    }
  }
  else if (gpio_dir_query)
  {
    if (PORT->Group[0].DIR.reg & GPIO_PORT)
    {
      strcpy(gpio_dir_str,"OUT");
    }
    else
    {
      strcpy(gpio_dir_str,"IN");
    }
  }


  if(transfer_complete && !strncasecmp("delay ",data,5))
  {
    queryState = 0;
    int d = atoi((char*)data + 5);
    if(d > 10000)
      d = 10000;
    if(d<0)
      d=0;
    resp_delay = (uint32_t)d;
  }
  tud_usbtmc_start_bus_read();
  return true;
}

bool tud_usbtmc_msgBulkIn_complete_cb()
{
  if((buffer_tx_ix == buffer_len) || idnQuery) // done
  {
    status &= (uint8_t)~(IEEE4882_STB_MAV); // clear MAV
    queryState = 0;
    bulkInStarted = 0;
    buffer_tx_ix = 0;
  }
  tud_usbtmc_start_bus_read();

  return true;
}

static unsigned int msgReqLen;

bool tud_usbtmc_msgBulkIn_request_cb(usbtmc_msg_request_dev_dep_in const * request)
{
  rspMsg.header.MsgID = request->header.MsgID,
  rspMsg.header.bTag = request->header.bTag,
  rspMsg.header.bTagInverse = request->header.bTagInverse;
  msgReqLen = request->TransferSize;

#ifdef xDEBUG
  uart_tx_str_sync("MSG_IN_DATA: Requested!\r\n");
#endif
  if(queryState == 0 || (buffer_tx_ix == 0))
  {
    TU_ASSERT(bulkInStarted == 0);
    bulkInStarted = 1;

    // > If a USBTMC interface receives a Bulk-IN request prior to receiving a USBTMC command message
    //   that expects a response, the device must NAK the request (*not stall*)
  }
  else
  {
    size_t txlen = tu_min32(buffer_len-buffer_tx_ix,msgReqLen);
    tud_usbtmc_transmit_dev_msg_data(&buffer[buffer_tx_ix], txlen,
        (buffer_tx_ix+txlen) == buffer_len, false);
    buffer_tx_ix += txlen;
  }
  // Always return true indicating not to stall the EP.
  return true;
}

void usbtmc_app_task_iter(void) {
  switch(queryState) {
  case 0:
    break;
  case 1:
    queryDelayStart = board_millis();
    queryState = 2;
    break;
  case 2:
    if( (board_millis() - queryDelayStart) > resp_delay) {
      queryDelayStart = board_millis();
      queryState=3;
      status |= 0x10u; // MAV
      status |= 0x40u; // SRQ
    }
    break;
  case 3:
    if( (board_millis() - queryDelayStart) > resp_delay) {
      queryState = 4;
    }
    break;
  case 4: // time to transmit;
    if(bulkInStarted && (buffer_tx_ix == 0)) {
      if(idnQuery)
      {
        tud_usbtmc_transmit_dev_msg_data(IDN, tu_min32(sizeof(IDN)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (adc_query)
      {
        tud_usbtmc_transmit_dev_msg_data(adc_voltage_str, tu_min32(sizeof(adc_voltage_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (dac_query)
      {
        tud_usbtmc_transmit_dev_msg_data(dac_voltage_str, tu_min32(sizeof(dac_voltage_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (mux_en_query)
      {
        tud_usbtmc_transmit_dev_msg_data(mux_en_str, tu_min32(sizeof(mux_en_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (mux_sel_query)
      {
        tud_usbtmc_transmit_dev_msg_data(mux_sel_str, tu_min32(sizeof(mux_sel_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (gpio_lev_query)
      {
        tud_usbtmc_transmit_dev_msg_data(gpio_lev_str, tu_min32(sizeof(gpio_lev_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (gpio_dir_query)
      {
        tud_usbtmc_transmit_dev_msg_data(gpio_dir_str, tu_min32(sizeof(gpio_dir_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (rst_cmd || dac_cmd || mux_en_cmd || mux_sel_cmd || gpio_lev_cmd || gpio_dir_cmd)
      { 
        tud_usbtmc_transmit_dev_msg_data(END_RESPONSE, tu_min32(sizeof(END_RESPONSE)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else
      {
        buffer_tx_ix = tu_min32(buffer_len,msgReqLen);
        tud_usbtmc_transmit_dev_msg_data(buffer, buffer_tx_ix, buffer_tx_ix == buffer_len, false);
      }

      // MAV is cleared in the transfer complete callback.
    }
    break;
  default:
    TU_ASSERT(false,);
    return;
  }
}

bool tud_usbtmc_initiate_clear_cb(uint8_t *tmcResult)
{
  *tmcResult = USBTMC_STATUS_SUCCESS;
  queryState = 0;
  bulkInStarted = false;
  status = 0;
  return true;
}

bool tud_usbtmc_check_clear_cb(usbtmc_get_clear_status_rsp_t *rsp)
{
  queryState = 0;
  bulkInStarted = false;
  status = 0;
  buffer_tx_ix = 0u;
  buffer_len = 0u;
  rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
  rsp->bmClear.BulkInFifoBytes = 0u;
  return true;
}
bool tud_usbtmc_initiate_abort_bulk_in_cb(uint8_t *tmcResult)
{
  bulkInStarted = 0;
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}
bool tud_usbtmc_check_abort_bulk_in_cb(usbtmc_check_abort_bulk_rsp_t *rsp)
{
  (void)rsp;
  tud_usbtmc_start_bus_read();
  return true;
}

bool tud_usbtmc_initiate_abort_bulk_out_cb(uint8_t *tmcResult)
{
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;

}
bool tud_usbtmc_check_abort_bulk_out_cb(usbtmc_check_abort_bulk_rsp_t *rsp)
{
  (void)rsp;
  tud_usbtmc_start_bus_read();
  return true;
}

void tud_usbtmc_bulkIn_clearFeature_cb(void)
{
}
void tud_usbtmc_bulkOut_clearFeature_cb(void)
{
  tud_usbtmc_start_bus_read();
}

// Return status byte, but put the transfer result status code in the rspResult argument.
uint8_t tud_usbtmc_get_stb_cb(uint8_t *tmcResult)
{
  uint8_t old_status = status;
  status = (uint8_t)(status & ~(IEEE4882_STB_SRQ)); // clear SRQ

  *tmcResult = USBTMC_STATUS_SUCCESS;
  // Increment status so that we see different results on each read...

  return old_status;
}

bool tud_usbtmc_indicator_pulse_cb(tusb_control_request_t const * msg, uint8_t *tmcResult)
{
  (void)msg;
  led_indicator_pulse();
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}

//---------------------------- New Code ----------------------------//

void adc_setup(void) {
  PM->APBCMASK.reg  |= PM_APBCMASK_ADC;            // enable ADC
  GCLK->CLKCTRL.reg  = GCLK_CLKCTRL_CLKEN     |    // enable clock
                       GCLK_CLKCTRL_GEN_GCLK0 |    // enable GCLK0
                       GCLK_CLKCTRL_ID_ADC;        // ADC will get GCLK0
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  // get FUSE calibration values
  uint32_t adc_bias       =  (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR    ) & ADC_FUSES_BIASCAL_Msk    ) >> ADC_FUSES_BIASCAL_Pos;
  uint32_t adc_linearity  =  (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
           adc_linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;
  while (ADC->STATUS.bit.SYNCBUSY);                // Wait for synchronization
  // load the calibration values
  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(adc_bias) | ADC_CALIB_LINEARITY_CAL(adc_linearity);
  while (ADC->STATUS.bit.SYNCBUSY);                // Wait for synchronization

  ADC->REFCTRL.reg   = ADC_REFCTRL_REFSEL_INTVCC1; // use internal ref which is 3.3/2 = 1.65V
  ADC->AVGCTRL.reg   = ADC_AVGCTRL_SAMPLENUM_1;    // use single sample
  ADC->CTRLB.reg     = ADC_CTRLB_PRESCALER_DIV4 |  // 8MHz / 512 = 32kHz clock freq
                       ADC_CTRLB_RESSEL_12BIT;     // 12bit results
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2  |  // cut input in half to get back to 0 to 3.3V range
                       ADC_INPUTCTRL_MUXNEG_GND |  // use chip GND
                       ADC_INPUTCTRL_MUXPOS_PIN2;  // AIN2, PB08
  
  PORT->Group[1].DIRCLR.reg     = ADC_PORT;            // PB08 as input
  PORT->Group[1].PINCFG[8].reg |= PORT_PINCFG_PMUXEN; // PB08 as peripheral
  PORT->Group[1].PMUX[4].reg    = PORT_PMUX_PMUXE_B;  // PB08 as function B, analog

  ADC->CTRLA.bit.ENABLE = true;                    // enable ADC
  while (ADC->STATUS.bit.SYNCBUSY);                // Wait for synchronization
  ADC->SWTRIG.bit.START = true;                    // Use software trigger to start conversion
  while (ADC->INTFLAG.bit.RESRDY == 0);            // wait for results
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;           // clear result flag
  // uint32_t adc_result = ADC->RESULT.reg;        // throw away first result
}

uint32_t adc_get_sample(void) {
  ADC->SWTRIG.bit.START = true;                    // Use software trigger to start conversion
  while (ADC->INTFLAG.bit.RESRDY == 0);            // wait for results
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;           // clear result flag
  return ADC->RESULT.reg * 2.0;
}

void dac_setup(void) {
  PM->APBCMASK.reg |= PM_APBCMASK_DAC;    // Enable peripheral clock for DAC
  
  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                      GCLK_GENCTRL_GENEN |        // Enable GCLK
                      GCLK_GENCTRL_SRC_DFLL48M |  // Set the clock source to 48MHz
                      GCLK_GENCTRL_ID(3);         // Set clock source on GCLK 3
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN     |    // enable clock
                      GCLK_CLKCTRL_GEN_GCLK3 |    // enable GCLK3
                      GCLK_CLKCTRL_ID_DAC;        //
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  PORT->Group[0].DIRSET.reg     = DAC_PORT;            // output
  PORT->Group[0].PINCFG[2].reg |= PORT_PINCFG_PMUXEN; // peripheral PA02
  PORT->Group[0].PMUX[1].reg    = PORT_PMUX_PMUXE_B;  // as function B, analog PA02

  DAC->CTRLB.reg |= DAC_CTRLB_EOEN |               // external pin enable
                    DAC_CTRLB_REFSEL_AVCC;         // use 3.3V
  uint16_t dac_value = (int)( (1.01 / DAC_REF_VOLTAGE) * DAC_MAX_VALUE );
  DAC->DATA.reg = dac_value;
  DAC->CTRLA.reg = DAC_CTRLA_ENABLE;               // Enable DAC
  while (DAC->STATUS.bit.SYNCBUSY);                // Wait for synchronization
}

void gpio_setup(void) {
  PORT->Group[0].DIRSET.reg = DOTSTAR_DATA; // clear the DOTSTAR with manual toggles
  PORT->Group[0].OUTCLR.reg = DOTSTAR_DATA;
  PORT->Group[0].DIRSET.reg = DOTSTAR_CLK;
  PORT->Group[0].OUTCLR.reg = DOTSTAR_CLK;

  for (int i = 0; i < 32; i++) {
    PORT->Group[0].OUTSET.reg = DOTSTAR_CLK;
    PORT->Group[0].OUTCLR.reg = DOTSTAR_CLK;
  }
  PORT->Group[0].OUTSET.reg = DOTSTAR_DATA;
  for (int i = 0; i < 3; i++) {
    PORT->Group[0].OUTSET.reg = DOTSTAR_CLK;
    PORT->Group[0].OUTCLR.reg = DOTSTAR_CLK;
  }
  PORT->Group[0].OUTCLR.reg = DOTSTAR_DATA; // intensity
  for (int i = 0; i < 5; i++) {
    if (i == 3) {
       PORT->Group[0].OUTSET.reg = DOTSTAR_DATA;
    }
    PORT->Group[0].OUTSET.reg = DOTSTAR_CLK;
    PORT->Group[0].OUTCLR.reg = DOTSTAR_CLK;
  }
  PORT->Group[0].OUTSET.reg = DOTSTAR_DATA; // blue
  for (int i = 0; i < 8; i++) {
    PORT->Group[0].OUTSET.reg = DOTSTAR_CLK;
    PORT->Group[0].OUTCLR.reg = DOTSTAR_CLK;
  }
  PORT->Group[0].OUTCLR.reg = DOTSTAR_DATA; // green
  for (int i = 0; i < 8; i++) {
    PORT->Group[0].OUTSET.reg = DOTSTAR_CLK;
    PORT->Group[0].OUTCLR.reg = DOTSTAR_CLK;
  }
  PORT->Group[0].OUTCLR.reg = DOTSTAR_DATA; // red
  for (int i = 0; i < 8; i++) {
    PORT->Group[0].OUTSET.reg = DOTSTAR_CLK;
    PORT->Group[0].OUTCLR.reg = DOTSTAR_CLK;
  }
  PORT->Group[0].OUTSET.reg = DOTSTAR_DATA;
  for (int i = 0; i < 32; i++) {
    PORT->Group[0].OUTSET.reg = DOTSTAR_CLK;
    PORT->Group[0].OUTCLR.reg = DOTSTAR_CLK;
  }
  PORT->Group[0].OUTCLR.reg = DOTSTAR_DATA;

  PORT->Group[0].PINCFG[30].reg = 0x00; // allow sw_clk to be gpio
  PORT->Group[0].PINCFG[31].reg = 0x00; // allow swdio  to be gpio

  PORT->Group[MUX1_S3_PIN/32].DIRSET.reg = MUX1_S3_PORT;
  PORT->Group[MUX1_S3_PIN/32].OUTCLR.reg = MUX1_S3_PORT;
  PORT->Group[MUX1_S2_PIN/32].DIRSET.reg = MUX1_S2_PORT;
  PORT->Group[MUX1_S2_PIN/32].OUTCLR.reg = MUX1_S2_PORT;
  PORT->Group[MUX1_S1_PIN/32].DIRSET.reg = MUX1_S1_PORT;
  PORT->Group[MUX1_S1_PIN/32].OUTCLR.reg = MUX1_S1_PORT;
  PORT->Group[MUX1_S0_PIN/32].DIRSET.reg = MUX1_S0_PORT;  
  PORT->Group[MUX1_S0_PIN/32].OUTCLR.reg = MUX1_S0_PORT;
  PORT->Group[MUX2_S3_PIN/32].DIRSET.reg = MUX2_S3_PORT;
  PORT->Group[MUX2_S3_PIN/32].OUTCLR.reg = MUX2_S3_PORT;
  PORT->Group[MUX2_S2_PIN/32].DIRSET.reg = MUX2_S2_PORT;
  PORT->Group[MUX2_S2_PIN/32].OUTCLR.reg = MUX2_S2_PORT;
  PORT->Group[MUX2_S1_PIN/32].DIRSET.reg = MUX2_S1_PORT;
  PORT->Group[MUX2_S1_PIN/32].OUTCLR.reg = MUX2_S1_PORT;
  PORT->Group[MUX2_S0_PIN/32].DIRSET.reg = MUX2_S0_PORT; 
  PORT->Group[MUX2_S0_PIN/32].OUTCLR.reg = MUX2_S0_PORT; 
  PORT->Group[MUX3_S3_PIN/32].DIRSET.reg = MUX3_S3_PORT;
  PORT->Group[MUX3_S3_PIN/32].OUTCLR.reg = MUX3_S3_PORT;
  PORT->Group[MUX3_S2_PIN/32].DIRSET.reg = MUX3_S2_PORT;
  PORT->Group[MUX3_S2_PIN/32].OUTCLR.reg = MUX3_S2_PORT;
  PORT->Group[MUX3_S1_PIN/32].DIRSET.reg = MUX3_S1_PORT;
  PORT->Group[MUX3_S1_PIN/32].OUTCLR.reg = MUX3_S1_PORT;
  PORT->Group[MUX3_S0_PIN/32].DIRSET.reg = MUX3_S0_PORT;
  PORT->Group[MUX3_S0_PIN/32].OUTCLR.reg = MUX3_S0_PORT;
  PORT->Group[MUX4_S3_PIN/32].DIRSET.reg = MUX4_S3_PORT;
  PORT->Group[MUX4_S3_PIN/32].OUTCLR.reg = MUX4_S3_PORT;
  PORT->Group[MUX4_S2_PIN/32].DIRSET.reg = MUX4_S2_PORT;
  PORT->Group[MUX4_S2_PIN/32].OUTCLR.reg = MUX4_S2_PORT;
  PORT->Group[MUX4_S1_PIN/32].DIRSET.reg = MUX4_S1_PORT;
  PORT->Group[MUX4_S1_PIN/32].OUTCLR.reg = MUX4_S1_PORT;
  PORT->Group[MUX4_S0_PIN/32].DIRSET.reg = MUX4_S0_PORT;
  PORT->Group[MUX4_S0_PIN/32].OUTCLR.reg = MUX4_S0_PORT;

  PORT->Group[MUX1_EN_PIN/32].DIRSET.reg = MUX1_EN_PORT;
  PORT->Group[MUX1_EN_PIN/32].OUTCLR.reg = MUX1_EN_PORT; // enable is active high 
  PORT->Group[MUX2_EN_PIN/32].DIRSET.reg = MUX2_EN_PORT;
  PORT->Group[MUX2_EN_PIN/32].OUTCLR.reg = MUX2_EN_PORT; // enable is active high 
  PORT->Group[MUX3_EN_PIN/32].DIRSET.reg = MUX3_EN_PORT;
  PORT->Group[MUX3_EN_PIN/32].OUTCLR.reg = MUX3_EN_PORT; // enable is active high 
  PORT->Group[MUX4_EN_PIN/32].DIRSET.reg = MUX4_EN_PORT;
  PORT->Group[MUX4_EN_PIN/32].OUTCLR.reg = MUX4_EN_PORT; // enable is active high 

  PORT->Group[(LS_EN_PIN/32)].DIRSET.reg = LS_EN_PORT;
  PORT->Group[(LS_EN_PIN/32)].OUTCLR.reg = LS_EN_PORT; // disable until DAC can drive level
}

char * get_value(char *in_string) {
  char *ptr = strrchr(in_string,' ') + 1;
  return ptr;
}

// char *ptr_value = get_value(scpi_string);
// float value     = strtof(ptr_value,NULL);
// char *command   = get_command(scpi_string,ptr_value);

char * get_command(char *in_string, char *ptr_value) {
  uint32_t command_len = ptr_value - in_string - 1;
  char *command = (char *) malloc(command_len +1);
  memcpy(command, in_string, command_len);
  command[command_len] = '\0';
  return command;
}

void ftoa(float num, char *str)
{
  int intpart = num;
  int intdecimal;
  uint32_t i;
  float decimal_part;
  char decimal[20];

  memset(str, 0x0, 20);
  itoa(num, str, 10);

  strcat(str, ".");

  decimal_part = num - intpart;
  intdecimal = decimal_part * 1000;

  if(intdecimal < 0)
  {
    intdecimal = -intdecimal;
  }
  itoa(intdecimal, decimal, 10);
  for(i =0;i < (3 - strlen(decimal));i++)
  {
    strcat(str, "0");
  }
  strcat(str, decimal);
}
