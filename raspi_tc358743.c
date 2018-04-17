/*
Copyright (c) 2015, Raspberry Pi Foundation
Copyright (c) 2015, Dave Stevenson
Copyright (c) 2017, Ben Kazemi
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <getopt.h>
#include <string.h>

#include <signal.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//#include <wiringPi.h>
//#include <wiringPiI2C.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
//#include "i2c-dev.h"

#include "interface/vcos/vcos.h"
#include "bcm_host.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_connection.h"

#include <sys/ioctl.h>
#include "tc358743_regs.h"

// Do the GPIO waggling from here, except that needs root access, plus
// there is variation on pin allocation between the various Pi platforms.
// Provided for reference, but needs tweaking to be useful.
//#define DO_PIN_CONFIG

#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t
   
u8 optional_file = 1; 
u8 called_quit = 0;

struct sensor_regs {
   uint16_t reg;
   uint8_t  data;
};

#define ENCODING MMAL_ENCODING_BGR24 //MMAL_ENCODING_I420// //MMAL_ENCODING_YUYV

#define UNPACK MMAL_CAMERA_RX_CONFIG_UNPACK_NONE
#define PACK MMAL_CAMERA_RX_CONFIG_PACK_NONE

#define I2C_ADDR 0x0F

#define CSI_IMAGE_ID 0x24

void signal_callback_handler(int);

static void i2c_rd(int fd, uint16_t reg, uint8_t *values, uint32_t n)
{
   int err;
   uint8_t buf[2] = { reg >> 8, reg & 0xff };
   struct i2c_rdwr_ioctl_data msgset;
   struct i2c_msg msgs[2] = {
      {
         .addr = I2C_ADDR,
         .flags = 0,
         .len = 2,
         .buf = buf,
      },
      {
         .addr = I2C_ADDR,
         .flags = I2C_M_RD,
         .len = n,
         .buf = values,
      },
   };

   msgset.msgs = msgs;
   msgset.nmsgs = 2;

   err = ioctl(fd, I2C_RDWR, &msgset);
   if(err != msgset.nmsgs)
      vcos_log_error("%s: reading register 0x%x from 0x%x failed, err %d\n",
            __func__, reg, I2C_ADDR, err);
}

static void i2c_wr(int fd, uint16_t reg, uint8_t *values, uint32_t n)
{
   uint8_t data[1024];
   int err, i;
   struct i2c_msg msg;
   struct i2c_rdwr_ioctl_data msgset;

   if ((2 + n) > sizeof(data))
      vcos_log_error("i2c wr reg=%04x: len=%d is too big!\n",
           reg, 2 + n);

   msg.addr = I2C_ADDR;
   msg.buf = data;
   msg.len = 2 + n;
   msg.flags = 0;

   data[0] = reg >> 8;
   data[1] = reg & 0xff;

   for (i = 0; i < n; i++)
      data[2 + i] = values[i];

   msgset.msgs = &msg;
   msgset.nmsgs = 1;

   err = ioctl(fd, I2C_RDWR, &msgset);
   if (err != 1) {
      vcos_log_error("%s: writing register 0x%x from 0x%x failed\n",
            __func__, reg, I2C_ADDR);
      return;
   }
}

static inline u8 i2c_rd8(int fd, u16 reg)
{
   u8 val;

   i2c_rd(fd, reg, &val, 1);

   return val;
}

static inline void i2c_wr8(int fd, u16 reg, u8 val)
{
   i2c_wr(fd, reg, &val, 1);
}

static inline void i2c_wr8_and_or(int fd, u16 reg,
      u8 mask, u8 val)
{
   i2c_wr8(fd, reg, (i2c_rd8(fd, reg) & mask) | val);
}

static inline u16 i2c_rd16(int fd, u16 reg)
{
   u16 val;

   i2c_rd(fd, reg, (u8 *)&val, 2);

   return val;
}

static inline void i2c_wr16(int fd, u16 reg, u16 val)
{
   i2c_wr(fd, reg, (u8 *)&val, 2);
}

static void i2c_wr16_and_or(int fd, u16 reg, u16 mask, u16 val)
{
   i2c_wr16(fd, reg, (i2c_rd16(fd, reg) & mask) | val);
}

static inline u32 i2c_rd32(int fd, u16 reg)
{
   u32 val;

   i2c_rd(fd, reg, (u8 *)&val, 4);

   return val;
}

static inline void i2c_wr32(int fd, u16 reg, u32 val)
{
   i2c_wr(fd, reg, (u8 *)&val, 4);
}

static inline int no_signal(int fd)
{
   int ret;
   ret = i2c_rd8(fd, SYS_STATUS);
   vcos_log_error("no_signal read %02X", ret);
   return !(ret & MASK_S_TMDS);
}

static inline int no_sync(int fd)
{
   int ret;
   ret = i2c_rd8(fd, SYS_STATUS);
   vcos_log_error("no_sync read %02X", ret);
   return !(ret & MASK_S_SYNC);
}

#define CC_RGB_PASSTHROUGH      1
#define CC_RGB_YUV422           2
#define CC_RGB_YUV444           3
#define CC_YUV444_YUV422        4
#define CC_YUV422_YUV444        5
#define COLOR_CONVERSION CC_RGB_PASSTHROUGH

#if COLOR_CONVERSION == CC_RGB_PASSTHROUGH
   // RGB through mode
   #define r8576  0x00 // 0000 0000 -- RGB full
   #define r8573  0x00 // 00000000 -- RGB through
   #define r8574  0x00
   #define r0004  0x0e24 // 0000 1110 0010 0111
#elif COLOR_CONVERSION == CC_RGB_YUV422
   #define r8574  0x08
   #define r8573  /* 11000001 */ 0xC1
   #define r8576  0x60
   #define r0004  0x0ee4
#elif COLOR_CONVERSION == CC_RGB_YUV444
   #define r8574  0x08
   #define r8573  /* 00000001 */ 0x01
   #define r8576  0x60
   #define r0004  0x0e24
#elif COLOR_CONVERSION == CC_YUV444_YUV422
   #define r8574  0x08
   #define r8573  /* 00000001 */ 0x80
   #define r8576  0x00
   #define r0004  0x0ee4
#elif COLOR_CONVERSION == CC_YUV422_YUV444
   #define r8574  0x08
   #define r8573  /* 00000001 */ 0x00
   #define r8576  0x00
   #define r0004  0x0e24
#endif

struct cmds_t {
   uint16_t addr;
   uint32_t value;
   int num_bytes;
};



#define TWOLANES

/* Old EDID for reference
   "\x00\xff\xff\xff\xff\xff\xff\x00\x52\x62\x88\x88\x00\x88\x88\x88"
   "\x1c\x15\x01\x03\x80\x00\x00\x78\x0a\xEE\x91\xA3\x54\x4C\x99\x26"
   "\x0F\x50\x54\x00\x00\x00\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01"
   "\x01\x01\x01\x01\x01\x01\x01\x1d\x00\x72\x51\xd0\x1e\x20\x6e\x28"
   "\x55\x00\xc4\x8e\x21\x00\x00\x1e\x8c\x0a\xd0\x8a\x20\xe0\x2d\x10"
   "\x10\x3e\x96\x00\x13\x8e\x21\x00\x00\x1e\x00\x00\x00\xfc\x00\x54"
   "\x6f\x73\x68\x69\x62\x61\x2d\x48\x32\x43\x0a\x20\x00\x00\x00\xFD"
   "\x00\x3b\x3d\x0f\x2e\x0f\x1e\x0a\x20\x20\x20\x20\x20\x20\x01\x67"

   "\x02\x03\x1A\x42"                                                   //CEA, V3, offset 1a to DTDs,  num of native DTDs | 0x40 (basic audio)
                   "\x47\x84\x13\x03\x02\x07\x06\x01"                   //Video
                                                   "\x23\x09\x07\x07"   //Audio. 2-chan LPCM, 32/44/48kHz, 16/20/24 bit.
   "\x66\x03\x0c\x00\x30\x00\x80"                                       //Vendor specific. IEEE reg 00-0C-03. Source physical address 00-30. Supports AI.
                               "\xE3\x00\x7F"                           //?? Reserved DCB type 7, length 3
                                          "\x8c\x0a\xd0\x8a\x20\xe0"    //DTD #1
   "\x2d\x10\x10\x3e\x96\x00\xc4\x8e\x21\x00\x00\x18"
                                                   "\x8c\x0a\xd0\x8a"   //DTD #2
   "\x20\xe0\x2d\x10\x10\x3e\x96\x00\x13\x8e\x21\x00\x00\x18"
                                                           "\x8c\x0a"   //DTD #3
   "\xa0\x14\x51\xf0\x16\x00\x26\x7c\x43\x00\x13\x8e\x21\x00\x00\x98"
   "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"   //End. 0 padded
   "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
   "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";
*/

#ifdef TWOLANES
static unsigned char TOSHH2C_DEFAULT_EDID[] =
    "00ffffffffffff005262888800888888"
    "1c150103800000780aEE91A3544C9926"
    "0F505400000001010101010101010101"
    "010101010101011d007251d01e206e28"
    "5500c48e2100001e8c0ad08a20e02d10"
    "103e9600138e2100001e000000"
                              "fc0054"   //FC, 00, Device name ("Toshiba-H2C")
    "6f73686962612d4832430a20000000"
                                  "FD"
    "003b3d0f2e0f1e0a2020202020200100"

    "0203"                                //CEA EDID, V3
        "20"                              //offset to DTDs,
          "42"                            //num of native DTDs | 0x40 (basic audio support)
            "4d841303021211012021223c"    //(Length|0x40), then list of VICs.
    "3d3e"
        "23090707"                        //(Length|0x20), then audio data block
                "66030c00300080"          //(Length|0x60), then vendor specific block
                              "E3007F"    //?? Reserved DCB type 7, length 3
    "8c0ad08a20e02d10103e9600c48e2100"    //DTD #1
    "0018"
        "8c0ad08a20e02d10103e9600138e"    //DTD #2
    "21000018"
            "8c0aa01451f01600267c4300"    //DTD #3
    "138e21000098"
                "00000000000000000000"    //End. 0 padded.
    "00000000000000000000000000000000"
    "00000000000000000000000000000000";

#else
//4 LANES
static unsigned char TOSHH2C_DEFAULT_EDID[] =
    "00ffffffffffff005262888800888888"
    "1c150103800000780aEE91A3544C9926"
    "0F505400000001010101010101010101"
    "010101010101011d007251d01e206e28"
    "5500c48e2100001e8c0ad08a20e02d10"
    "103e9600138e2100001e000000"
                              "fc0054"   //FC, 00, Device name ("Toshiba-H2C")
    "6f73686962612d4832430a20000000"
                                  "FD"
    "003b3d0f2e0f1e0a2020202020200100"

    "0203"                                //CEA EDID, V3
        "22"                              //offset to DTDs,
          "42"                            //num of native DTDs | 0x40 (basic audio support)
            "4f841303021211012021223c"    //(Length|0x40), then list of VICs.
    "3d3e101f"
            "2309070766030c00300080"      //Audio. 2-chan LPCM, 32/44/48kHz, 16/20/24 bit.
                                  "E3"    //?? Reserved DCB type 7, length 3
    "007F"
        "8c0ad08a20e02d10103e9600c48e"    //DTD #1
    "21000018"
            "8c0ad08a20e02d10103e9600"    //DTD #2
    "138e21000018"
                "8c0aa01451f01600267c"    //DTD #3
    "4300138e21000098"
                    "0000000000000000"    //End. 0 padded.
    "00000000000000000000000000000000"
    "00000000000000000000000000000000";

#endif



struct cmds_t cmds2[] = 
{
   /* HDMI specification requires HPD to be pulsed low for 100ms when EDID changed */
   {0x8544, 0x01, 1},      // DDC5V detection interlock -- disable
   {0x8544, 0x00, 1},      // DDC5V detection interlock -- pulse low

   {0x0000, 100, 0xFFFF},  // sleep
   {0x8544, 0x10, 1},      // DDC5V detection interlock -- enable

   {0x85D1, 0x01, 1},         // Key loading command
   {0x8560, 0x24, 1},         // KSV Auto Clear Mode
   {0x8563, 0x11, 1},         // EESS_Err auto-unAuth
   {0x8564, 0x0F, 1},      // DI_Err (Data Island Error) auto-unAuth

      // RGB888 to YUV422 conversion (must)
   {0x8574, r8574, 1},
   {0x8573, r8573, 1},        // OUT YUV444[7]
      // 1010 0001
   {0x8576, r8576, 1},        // [7:5] = YCbCr601 Limited ? 3'b011 : 3'b101 (YCbCr 709 Limited)

   {0x8600, 0x00, 1},      // Forced Mute Off, Set Auto Mute On
   {0x8602, 0xF3, 1},      // AUTO Mute (AB_sel, PCM/NLPCM_chg, FS_chg, PX_chg, PX_off, DVI)
   {0x8603, 0x02, 1},         // AUTO Mute (AVMUTE)
   {0x8604, 0x0C, 1},      // AUTO Play (mute-->BufInit-->play)
   {0x8606, 0x05, 1},         // BufInit start time = 0.5sec
   {0x8607, 0x00, 1},         // Disable mute
   {0x8620, 0x00, 1},      // [5] = 0: LPCM/NLPCMinformation extraction from Cbit
   {0x8640, 0x01, 1},         // CTS adjustment=ON
   {0x8641, 0x65, 1},      // Adjustment level 1=1000ppm, Adjustment level 2=2000ppm
   {0x8642, 0x07, 1},         // Adjustment level 3=4000ppm
   {0x8652, 0x02, 1},      // Data Output Format: [6:4] = 0, 16-bit, [1:0] = 2, I2S Format
   {0x8665, 0x10, 1},      // [7:4] 128 Fs Clock divider  Delay 1 * 0.1 s, [0] = 0: 44.1/48 KHz Auto switch setting

   {0x8709, 0xFF, 1},      // ""FF"": Updated secondary Pkts even if there are errors received
   {0x870B, 0x2C, 1},      // [7:4]: ACP packet Intervals before interrupt, [3:0] AVI packet Intervals, [] * 80 ms
   {0x870C, 0x53, 1},      // [6:0]: PKT receive interrupt is detected,storage register automatic clear, video signal with RGB and no repeat are set
   {0x870D, 0x01, 1},      // InFo Pkt: [7]: Correctable Error is included, [6:0] # of Errors before assert interrupt
   {0x870E, 0x30, 1},      // [7:4]: VS packet Intervals before interrupt, [3:0] SPD packet Intervals, [] * 80 ms
   {0x9007, 0x10, 1},      // [5:0]  Auto clear by not receiving 16V GBD
   {0x854A, 0x01, 1},      // HDMIRx Initialization Completed, THIS MUST BE SET AT THE LAST!

};
#define NUM_REGS_CMD2 (sizeof(cmds2)/sizeof(cmds2[0]))

struct cmds_t cmds3[] =
{
   {0x0004, r0004 | 0x03, 2},        // Enable tx buffer_size
};
#define NUM_REGS_CMD3 (sizeof(cmds3)/sizeof(cmds3[0]))

struct cmds_t stop_cmds[] =
{
{0x8544, 0x01, 1},   // regain manual control
{0x8544, 0x00, 1},   // disable HPD
//{0x0014, 0x8000, 0x12}, //power island enable (or'ed with register)
//{0x0000, 10, 0xFFFF}, //Sleep 10ms
//{0x0002, 0x0001, 2}, // put to sleep
};
#define NUM_REGS_STOP (sizeof(stop_cmds)/sizeof(stop_cmds[0]))

void write_regs(int fd, struct cmds_t *regs, int count)
{
   int i;
   for (i=0; i<count; i++)
   {
      switch(regs[i].num_bytes)
      {
         case 1:
            i2c_wr8(fd, regs[i].addr, (uint8_t)regs[i].value);
            break;
         case 2:
            i2c_wr16(fd, regs[i].addr, (uint16_t)regs[i].value);
            break;
         case 4:
            i2c_wr32(fd, regs[i].addr, regs[i].value);
            break;
         case 0x11:
            i2c_wr8_and_or(fd, regs[i].addr, 0xFF, (uint8_t)regs[i].value);
            break;
         case 0x12:
            i2c_wr16_and_or(fd, regs[i].addr, 0xFFFF, (uint16_t)regs[i].value);
            break;
         case 0xFFFF:
            vcos_sleep(regs[i].value);
            break;
         default:
            vcos_log_error("%u bytes specified in entry %d - not supported", regs[i].num_bytes, i);
            break;
      }
   }
}

unsigned char ascii_to_hex(unsigned char c)
{
   if(c>='0' && c<='9')
      return (c-'0');
   else if (c>='A' && c<='F')
      return ((c-'A') + 10);
   else if (c>='a' && c<='f')
      return ((c-'a') + 10);
   return 0;
}

void start_camera_streaming(int fd)
{
   #ifdef DO_PIN_CONFIG
      wiringPiSetupGpio();
      pinModeAlt(0, INPUT);
      pinModeAlt(1, INPUT);
      //Toggle these pin modes to ensure they get changed.
      pinModeAlt(28, INPUT);
      pinModeAlt(28, 4);   //Alt0
      pinModeAlt(29, INPUT);
      pinModeAlt(29, 4);   //Alt0
      digitalWrite(41, 1); //Shutdown pin on B+ and Pi2
      digitalWrite(32, 1); //LED pin on B+ and Pi2
   #endif

   #define ENABLE_DATALANE_1 0x0
   #define DISABLE_DATALANE_1 0x1

   u8 _s_v_format = i2c_rd8(fd, VI_STATUS) & 0x0F;
   
   vcos_log_error("VI_STATUS to select cfg.data_lanes: %u", _s_v_format);

   u16 r0006;
   u8 r0148;
   u32 r0500;

   if (_s_v_format < 12)
   {
      r0006 = 0x0080;
      r0148 = DISABLE_DATALANE_1;
      r0500 = 0xA3008080;
      vcos_log_error("Selected Sub 720p registers");
   }
   else
   {
      r0006 = 0x0008;
      r0148 = ENABLE_DATALANE_1;
      r0500 = 0xA3008082;
      vcos_log_error("Selected 720p+ registers");
   }

   struct cmds_t cmds[] = 
   {
      {0x0004, 0x0000, 2}, // Disable video TX Buffer 
      // Turn off power and put in reset
      // handle.first_boot = VC_FALSE;
      {0x0002, 0x0F00, 2},    // Assert Reset, [0] = 0: Exit Sleep, wait

      {0x0000, 1, 0xFFFF},      // V054 requires us to wait 1ms for PLL to lock
      {0x0002, 0x0000, 2},    // Release Reset, Exit Sleep
      {0x0006, r0006, 2},    // FIFO level
      {0x0008, 0x005f, 2},    // Audio buffer level -- 96 bytes = 0x5F + 1
      {0x0014, 0xFFFF, 2},     // Clear HDMI Rx, CSI Tx and System Interrupt Status
      {0x0016, 0x051f, 2},    // Enable HDMI-Rx Interrupt (bit 9), Sys interrupt (bit 5). Disable others. 11-15, 6-7 reserved
      {0x0020, 0x8111, 2},        // PRD[15:12], FBD[8:0]
      {0x0022, 0x0213, 2},    // FRS[11:10], LBWS[9:8]= 2, Clock Enable[4] = 1,  ResetB[1] = 1,  PLL En[0]
      {0x0004, r0004,  2},    // PwrIso[15], 422 output, send infoframe
      {0x0140, 0x0,    4},    //Enable CSI-2 Clock lane
      {0x0144, 0x0,    4},    //Enable CSI-2 Data lane 0
      {0x0148, r0148,    4},    //Enable CSI-2 Data lane 1
      {0x014C, 0x1,    4},    //Disable CSI-2 Data lane 2
      {0x0150, 0x1,    4},    //Disable CSI-2 Data lane 3

      {0x0210, 0x00002988, 4},   // LP11 = 100 us for D-PHY Rx Init
      {0x0214, 0x00000005, 4},   // LP Tx Count[10:0]
      {0x0218, 0x00001d04, 4},   // TxClk_Zero[15:8]
      {0x021C, 0x00000002, 4},   // TClk_Trail =
      {0x0220, 0x00000504, 4},   // HS_Zero[14:8] =
      {0x0224, 0x00004600, 4},   // TWAKEUP Counter[15:0]
      {0x0228, 0x0000000A, 4},   // TxCLk_PostCnt[10:0]
      {0x022C, 0x00000004, 4},   // THS_Trail =
      {0x0234, 0x0000001F, 4},   // Enable Voltage Regulator for CSI (4 Data + Clk) Lanes
      {0x0204, 0x00000001, 4},   // Start PPI

      {0x0518, 0x00000001, 4},   // Start CSI-2 Tx
      {0x0500, r0500, 4},   // SetBit[31:29]
         //
         //    1010 0011 0000 0000    1000 0000 1010 0010

      {0x8502, 0x01, 1},      // Enable HPD DDC Power Interrupt
      {0x8512, 0xFE, 1},      // Disable HPD DDC Power Interrupt Mask
      {0x8513, (uint8_t) ~0x20, 1},    // Receive interrupts for video format change (bit 5)
      {0x8515, (uint8_t) ~0x02, 1},    // Receive interrupts for format change (bit 1)

      {0x8531, 0x01, 1},      // [1] = 1: RefClk 42 MHz, [0] = 1, DDC5V Auto detection
      {0x8540, 0x0A8C, 2}, // SysClk Freq count with RefClk = 27 MHz (0x1068 for 42 MHz, default)
      {0x8630, 0x00041eb0, 4},   // Audio FS Lock Detect Control [19:0]: 041EB0 for 27 MHz, 0668A0 for 42 MHz (default)
      {0x8670, 0x01, 1},         // SysClk 27/42 MHz: 00:= 42 MHz

      {0x8532, 0x80, 1},      // PHY_AUTO_RST[7:4] = 1600 us, PHY_Range_Mode = 12.5 us
      {0x8536, 0x40, 1},      // [7:4] Ibias: TBD, [3:0] BGR_CNT: Default
      {0x853F, 0x0A, 1},      // [3:0] = 0x0a: PHY TMDS CLK line squelch level: 50 uA

      {0x8543, 0x32, 1},      // [5:4] = 2'b11: 5V Comp, [1:0] = 10, DDC 5V active detect delay setting: 100 ms
      {0x8544, 0x10, 1},      // DDC5V detection interlock -- enable
      {0x8545, 0x31, 1},      //  [5:4] = 2'b11: Audio PLL charge pump setting to Normal, [0] = 1: DAC/PLL Power On
      {0x8546, 0x2D, 1},      // [7:0] = 0x2D: AVMUTE automatic clear setting (when in MUTE and no AVMUTE CMD received) 45 * 100 ms

      {0x85C7, 0x01, 1},      // [6:4] EDID_SPEED: 100 KHz, [1:0] EDID_MODE: Internal EDID-RAM & DDC2B mode
      {0x85CB, 0x01, 1},      // EDID Data size read from EEPROM EDID_LEN[10:8] = 0x01, 256-Byte
   };
   #define NUM_REGS_CMD (sizeof(cmds)/sizeof(cmds[0]))


   write_regs(fd, cmds, NUM_REGS_CMD);
      // default is 256 bytes, 256 / 16
   // write in groups of 16
   {
      u8 edid[256];
      int i, j;
      unsigned char checksum = 0;
      for (i = 0; i < sizeof(TOSHH2C_DEFAULT_EDID)/2; i += 16)
      {
         for (j = 0; j < 16; j++)
         {
            edid[i + j] = (ascii_to_hex(TOSHH2C_DEFAULT_EDID[(i+j)*2])<<4) +
                           ascii_to_hex(TOSHH2C_DEFAULT_EDID[(i+j)*2 + 1]);
            checksum -= edid[i + j];
         }
         // if checksum byte
         if (i == (7 * 16) || i == (15 * 16))
         {
            edid[i + 15] = checksum;
            checksum = 0;
         }
         i2c_wr(fd, 0x8C00 + i, &edid[i], 16);
      }
   }
   write_regs(fd, cmds2, NUM_REGS_CMD2);
}

void stop_camera_streaming(int fd)
{
   write_regs(fd, stop_cmds, NUM_REGS_STOP);
#ifdef DO_PIN_CONFIG
   digitalWrite(41, 0); //Shutdown pin on B+ and Pi2
   digitalWrite(32, 0); //LED pin on B+ and Pi2
#endif
}

int running = 0;

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   MMAL_STATUS_T status;
   vcos_log_error("Buffer %p returned, filled %d, timestamp %llu, flags %04X", buffer, buffer->length, buffer->pts, buffer->flags);
   //vcos_log_error("File handle: %p", port->userdata);
   int bytes_written = buffer->length;

   if (running)
   {
      if (optional_file == 1)
      {
         FILE *file = (FILE*)port->userdata;
         bytes_written = fwrite(buffer->data, 1, buffer->length, file);
         fflush(file);
      }
      mmal_buffer_header_mem_unlock(buffer);

      if (bytes_written != buffer->length)
      {
         vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
      }

      mmal_buffer_header_release(buffer);
      status = mmal_port_send_buffer(port, buffer);
      if(status != MMAL_SUCCESS)
      {
         vcos_log_error("mmal_port_send_buffer failed on buffer %p, status %d", buffer, status);
      }
   }
}


/**
 * Open a file based on the settings in state
 *
 * @param state Pointer to state
 */
static FILE *open_filename(const char *filename)
{
   FILE *new_handle = NULL;

   if (filename && optional_file == 1)
   {
      int network = 0, socktype;
      if(!strncmp("tcp://", filename, 6))
      {
         network = 1;
         socktype = SOCK_STREAM;
      }
      if(!strncmp("udp://", filename, 6))
      {
         network = 1;
         socktype = SOCK_DGRAM;
      }
      if(network)
      {
         filename += 6;
         char *colon;
         colon = strchr(filename, ':');
         int sfd = socket(AF_INET, socktype, 0);
         if(sfd < 0)
         {
              perror("socket");
         }

         unsigned short port;
         sscanf(colon + 1, "%hu", &port);
         *colon = 0;

         fprintf(stderr, "Connecting to %s:%hu\n", filename, port);
         
         struct sockaddr_in saddr;
         saddr.sin_family = AF_INET;
         saddr.sin_port = htons(port);
         inet_aton(filename, &saddr.sin_addr);

         if(connect(sfd, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in)) < 0)
         {
            perror("connect");
         }

         new_handle = fdopen(sfd, "w");
      }
      else
      {
         new_handle = fopen(filename, "wb");
      }
   }

   return new_handle;
}

/* Clean up after Ctrl+c */
void signal_callback_handler(int signum)
{
   printf("Caught signal %d\nCleanly exiting\n", signum);
   called_quit = 1; 
   // goto exit;
   // exit(signum);
}

int main ( int argc, char *argv[] )
{
   int8_t option_index = 0; 
   u32 sleep_duration = 0; //strtol(argv[1], &ptr, 10);
   char *_filename = "test_encode.h264"; 
   while ((option_index = getopt(argc, argv, "nht:o:")) != -1)
   {
      switch (option_index)
      {
         case 'n':
            optional_file = 0; 
            break;
         case 't':
            sleep_duration = atoi(optarg); // not type safe 
            break; 
         case 'o':
            if (strcmp(optarg, "") != 0)
            {
               _filename = (char*) malloc(1 + strlen(optarg) + strlen(".h264"));
               strcpy(_filename, optarg);
               strcat(_filename, ".h264");   
            }  
            break; 
         case 'h':
            printf("Available options:\n\n");
            printf("\t-t\tDefaults to 0 ms which playbacks until ctrl+c.\n");
            printf("\t-o\tOutput filename, default is 'test_encode.h264'.\n");
            printf("\t-n\tNo output file saved.\n");
            printf("\t-h\tHelp\n");
            return 1; 
         default: 
            printf("Incorrect option!\n");
            return 1;
      }
   }
   printf("Filename is %s\n", _filename);
   if (sleep_duration != 0)
      printf("Playing back for %u ms.\n", sleep_duration);
   else
      printf("Playing back indefinitely\n");
   u8 playthroughs = 0;
   MMAL_COMPONENT_T *rawcam, *render, *isp, *splitter, *encoder;
   MMAL_STATUS_T status;
   MMAL_PORT_T *output, *input, *isp_input, *isp_output, *encoder_input, *encoder_output;
   MMAL_POOL_T *pool;
   MMAL_CONNECTION_T *connection[4] = {0};
   MMAL_PARAMETER_CAMERA_RX_CONFIG_T rx_cfg = {{MMAL_PARAMETER_CAMERA_RX_CONFIG, sizeof(rx_cfg)}};
   MMAL_PARAMETER_CAMERA_RX_TIMING_T rx_timing = {{MMAL_PARAMETER_CAMERA_RX_TIMING, sizeof(rx_timing)}};
   int i2c_fd;
   unsigned int width, height, fps, frame_interval;
   unsigned int frame_width, frame_height;

   /* CALL THE SIGNAL HANDLER FUNCTION ON A Ctrl-C EXIT */
   signal(SIGINT, signal_callback_handler);

   i2c_fd = open("/dev/i2c-0", O_RDWR);
   if (!i2c_fd)
   {
      vcos_log_error("Couldn't open I2C device");
      return -1;
   }
   if(ioctl(i2c_fd, I2C_SLAVE, 0x0F) < 0)
   {
      vcos_log_error("Failed to set I2C address");
      return -1;
   }

   bcm_host_init();
   vcos_log_register("RaspiRaw", VCOS_LOG_CATEGORY);

   status = mmal_component_create("vc.ril.rawcam", &rawcam);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create rawcam");
      return -1;
   }
   status = mmal_component_create("vc.ril.video_render", &render);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create render");
      return -1;
   }

   status = mmal_component_create("vc.ril.isp", &isp);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create isp");
      return -1;
   }

   status = mmal_component_create("vc.ril.video_splitter", &splitter);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create isp");
      return -1;
   }

   status = mmal_component_create("vc.ril.video_encode", &encoder);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create encoder");
      return -1;
   }

   loop:

   output = rawcam->output[0];
   isp_input = isp->input[0];
   isp_output = isp->output[0];
   encoder_input = encoder->input[0];
   encoder_output = encoder->output[0];
   input = render->input[0];

   // setup CSI configs on rawcam
   status = mmal_port_parameter_get(output, &rx_cfg.hdr);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to get cfg");
      goto component_destroy;
   }
   rx_cfg.image_id = CSI_IMAGE_ID;


   u8 _s_v_format = i2c_rd8(i2c_fd, VI_STATUS) & 0x0F;
   vcos_log_error("VI_STATUS to select cfg.data_lanes: %u", _s_v_format);

   if (_s_v_format < 12)
   {
      rx_cfg.data_lanes = 1; 
      vcos_log_error("rx_cfg.data_lanes = 1");
   }
   else
   {
      rx_cfg.data_lanes = 2; 
      vcos_log_error("rx_cfg.data_lanes = 2");
   }



   rx_cfg.unpack = UNPACK;
   rx_cfg.pack = PACK;
   rx_cfg.embedded_data_lines = 128;
   vcos_log_error("Set pack to %d, unpack to %d", rx_cfg.unpack, rx_cfg.pack);
   status = mmal_port_parameter_set(output, &rx_cfg.hdr);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to set cfg");
      goto component_destroy;
   }

   vcos_log_error("Enable rawcam....");

   status = mmal_component_enable(rawcam);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to enable");
      goto component_destroy;
   }
   status = mmal_port_parameter_set_boolean(output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to set zero copy");
      goto component_disable;
   }

   // end setup rawcam

   vcos_log_error("Enable isp....");

   status = mmal_component_enable(isp);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to enable");
      goto component_destroy;
   }
   status = mmal_port_parameter_set_boolean(isp_output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to set zero copy");
      goto component_disable;
   }

   vcos_log_error("Enable splitter....");

   status = mmal_component_enable(splitter);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to enable");
      goto component_destroy;
   }
   status = mmal_port_parameter_set_boolean(splitter->output[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to set zero copy");
      goto component_disable;
   }

   start_camera_streaming(i2c_fd);
   vcos_sleep(500);  //Give a chance to detect signal
   vcos_log_error("Waiting to detect signal...");

   int count=0;
   while((count<20) && (no_sync(i2c_fd) || no_signal(i2c_fd)))
   {
      vcos_sleep(200);
      count++;
   }
   vcos_log_error("Signal reported");
   width = ((i2c_rd8(i2c_fd, DE_WIDTH_H_HI) & 0x1f) << 8) +
      i2c_rd8(i2c_fd, DE_WIDTH_H_LO);
   height = ((i2c_rd8(i2c_fd, DE_WIDTH_V_HI) & 0x1f) << 8) +
      i2c_rd8(i2c_fd, DE_WIDTH_V_LO);
   frame_width = ((i2c_rd8(i2c_fd, H_SIZE_HI) & 0x1f) << 8) +
      i2c_rd8(i2c_fd, H_SIZE_LO);
   frame_height = (((i2c_rd8(i2c_fd, V_SIZE_HI) & 0x3f) << 8) +
      i2c_rd8(i2c_fd, V_SIZE_LO)) / 2;
   





   if (playthroughs == 0)
   {
      playthroughs++;
      vcos_log_error("First playthrough, Goto Loop");
      goto loop;
   }


   /* frame interval in milliseconds * 10
    * Require SYS_FREQ0 and SYS_FREQ1 are precisely set */
   frame_interval = ((i2c_rd8(i2c_fd, FV_CNT_HI) & 0x3) << 8) +
      i2c_rd8(i2c_fd, FV_CNT_LO);
   fps =  (frame_interval > 0) ?
             (10000/frame_interval) : 0;
   vcos_log_error("Signal is %u x %u, frm_interval %u, so %u fps", width, height, frame_interval, fps);
   vcos_log_error("Frame w x h is %u x %u", frame_width, frame_height);


   output->format->es->video.crop.width = width;
   output->format->es->video.crop.height = height;
   output->format->es->video.width = VCOS_ALIGN_UP(width, 32); //VCOS_ALIGN_UP(WIDTH, 32);
   output->format->es->video.height = VCOS_ALIGN_UP(height, 16); //VCOS_ALIGN_UP(HEIGHT, 16);
   output->format->es->video.frame_rate.num = 10000;
   output->format->es->video.frame_rate.den = frame_interval ? frame_interval : 10000;
   output->format->encoding = ENCODING;
   status = mmal_port_format_commit(output);

   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("output: Failed port_format_commit");
      mmal_log_dump_port(output);
      goto component_disable;
   }

   output->buffer_size = output->buffer_size_recommended;
   output->buffer_num = 3;
   vcos_log_error("output: buffer size is %d bytes, num %d", output->buffer_size, output->buffer_num);

   vcos_log_error("Create connection rawcam output to isp input....");
   status =  mmal_connection_create(&connection[0], output, isp_input, MMAL_CONNECTION_FLAG_TUNNELLING);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create connection status %d: rawcam->isp", status);
      goto component_disable;
   }

   // ISP output port does not follow input, so do have to set that one up.
   mmal_format_copy(isp_output->format, isp_input->format);
   isp_output->format->encoding = MMAL_ENCODING_I420;
   vcos_log_error("Setting isp output port format");
   status = mmal_port_format_commit(isp_output);
   isp_output->buffer_size = isp_output->buffer_size_recommended;
   isp_output->buffer_num = isp_output->buffer_num_recommended;
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create connection status %d: rawcam->isp", status);
      goto component_disable;
   }

   //  Encoder setup
   vcos_log_error("Create connection isp output to splitter input....");
   status =  mmal_connection_create(&connection[1], isp_output, splitter->input[0], MMAL_CONNECTION_FLAG_TUNNELLING);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create connection status %d: isp->splitter", status);
      goto component_disable;
   }

   vcos_log_error("Create connection splitter output to render input....");
   status =  mmal_connection_create(&connection[2], splitter->output[0], input, MMAL_CONNECTION_FLAG_TUNNELLING);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create connection status %d: splitter->render", status);
      goto component_disable;
   }

   vcos_log_error("Create connection splitter output2 to encoder input....");
   status =  mmal_connection_create(&connection[3], splitter->output[1], encoder_input, MMAL_CONNECTION_FLAG_TUNNELLING);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create connection status %d: splitter->encoder", status);
      goto component_disable;
   }

   // Only supporting H264 at the moment
   encoder_output->format->encoding = MMAL_ENCODING_H264;

   encoder_output->format->bitrate = 17000000;
   encoder_output->buffer_size = encoder_output->buffer_size_recommended;

   if (encoder_output->buffer_size < encoder_output->buffer_size_min)
      encoder_output->buffer_size = encoder_output->buffer_size_min;

   encoder_output->buffer_num = 8; //encoder_output->buffer_num_recommended;

   if (encoder_output->buffer_num < encoder_output->buffer_num_min)
      encoder_output->buffer_num = encoder_output->buffer_num_min;

   // We need to set the frame rate on output to 0, to ensure it gets
   // updated correctly from the input framerate when port connected
   encoder_output->format->es->video.frame_rate.num = fps;//0;
   encoder_output->format->es->video.frame_rate.den = 1;

   // Commit the port changes to the output port
   status = mmal_port_format_commit(encoder_output);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on encoder output port");
   }

   {
      MMAL_PARAMETER_VIDEO_PROFILE_T  param;
      param.hdr.id = MMAL_PARAMETER_PROFILE;
      param.hdr.size = sizeof(param);

      param.profile[0].profile = MMAL_VIDEO_PROFILE_H264_HIGH;//state->profile;
      param.profile[0].level = MMAL_VIDEO_LEVEL_H264_4; // This is the only value supported

      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("Unable to set H264 profile");
      }
   }

   if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, 1) != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set immutable input flag");
      // Continue rather than abort..
   }

   //set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
   if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, 0) != MMAL_SUCCESS)
   {
      vcos_log_error("failed to set INLINE HEADER FLAG parameters");
      // Continue rather than abort..
   }

   //set INLINE VECTORS flag to request motion vector estimates
   if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, 0) != MMAL_SUCCESS)
   {
      vcos_log_error("failed to set INLINE VECTORS parameters");
      // Continue rather than abort..
   }

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on video encoder input port");
   }

   vcos_log_error("Enable encoder....");

   status = mmal_component_enable(encoder);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to enable");
      goto component_destroy;
   }
   status = mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to set zero copy");
      goto component_disable;
   }

/*
   vcos_log_error("rawcam supported encodings:");
   display_supported_encodings(output);
   vcos_log_error("isp input supported encodings:");
   display_supported_encodings(isp_input);
   vcos_log_error("isp output supported encodings:");
   display_supported_encodings(isp_output);
   vcos_log_error("encoder input supported encodings:");
   display_supported_encodings(encoder_input);
   vcos_log_error("encoder output supported encodings:");
   display_supported_encodings(encoder_output);
   vcos_log_error("render supported encodings:");
   display_supported_encodings(input);
*/


   status = mmal_port_parameter_set_boolean(input, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to set zero copy on video_render");
      goto component_disable;
   }

   if (status == MMAL_SUCCESS)
   {
      vcos_log_error("Enable connection[0]...");
      vcos_log_error("buffer size is %d bytes, num %d", output->buffer_size, output->buffer_num);
      status =  mmal_connection_enable(connection[0]);
      if (status != MMAL_SUCCESS)
      {
         mmal_connection_destroy(connection[0]);
      }
      vcos_log_error("Enable connection[1]...");
      vcos_log_error("buffer size is %d bytes, num %d", isp_output->buffer_size, isp_output->buffer_num);
      status =  mmal_connection_enable(connection[1]);
      if (status != MMAL_SUCCESS)
      {
         mmal_connection_destroy(connection[1]);
      }

      vcos_log_error("Enable connection[2]...");
      vcos_log_error("buffer size is %d bytes, num %d", splitter->output[0]->buffer_size, splitter->output[0]->buffer_num);
      status =  mmal_connection_enable(connection[2]);
      if (status != MMAL_SUCCESS)
      {
         mmal_connection_destroy(connection[2]);
      }

      vcos_log_error("Enable connection[3]...");
      vcos_log_error("buffer size is %d bytes, num %d", splitter->output[1]->buffer_size, splitter->output[1]->buffer_num);
      status =  mmal_connection_enable(connection[3]);
      if (status != MMAL_SUCCESS)
      {
         mmal_connection_destroy(connection[3]);
      }
   }

   // open h264 file and put the file handle in userdata for the encoder output port
   encoder_output->userdata = (void*)open_filename(_filename);
   if (strcmp(_filename, "test_encode.h264") != 0)
   {
      free(_filename);
      _filename = NULL;     
   }
   //Create encoder output buffers

   vcos_log_error("Create pool of %d buffers of size %d", encoder_output->buffer_num, encoder_output->buffer_size);
   pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);
   if(!pool)
   {
      vcos_log_error("Failed to create pool");
      goto component_disable;
   }

   status = mmal_port_enable(encoder_output, encoder_buffer_callback);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to enable port");
   }

   running = 1;
   int i;
   for(i=0; i<encoder_output->buffer_num; i++)
   {
      MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);

      if (!buffer)
      {
         vcos_log_error("Where'd my buffer go?!");
         goto port_disable;
      }
      status = mmal_port_send_buffer(encoder_output, buffer);
      if(status != MMAL_SUCCESS)
      {
         vcos_log_error("mmal_port_send_buffer failed on buffer %p, status %d", buffer, status);
         goto port_disable;
      }
      vcos_log_error("Sent buffer %p", buffer);
   }


   // Setup complete
   vcos_log_error("All done. Start streaming...");
   write_regs(i2c_fd, cmds3, NUM_REGS_CMD3);
   vcos_log_error("View!");

   if (sleep_duration > 0)
   {
      vcos_log_error("Sleeping for %u ms", sleep_duration);
      vcos_sleep(sleep_duration);
   }
   else
   {
      vcos_log_error("Sleeping until you ctrl+c me!");
      while (called_quit != 1)
      {
         vcos_sleep(100);
      }
   }

// exit:
   running = 0;

   vcos_log_error("Stopping streaming...");
   stop_camera_streaming(i2c_fd);

   playthroughs = 0;

port_disable:

   mmal_connection_disable(connection[0]);
   mmal_connection_destroy(connection[0]);

   mmal_connection_disable(connection[1]);
   mmal_connection_destroy(connection[1]);

   mmal_connection_disable(connection[2]);
   mmal_connection_destroy(connection[2]);

   mmal_connection_disable(connection[3]);
   mmal_connection_destroy(connection[3]);

component_disable:
   status = mmal_component_disable(rawcam);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to disable rawcam");
   }

   status = mmal_component_disable(isp);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to disable isp");
   }

   status = mmal_component_disable(render);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to disable render");
   }

   status = mmal_component_disable(splitter);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to disable splitter");
   }

   status = mmal_component_disable(encoder);
   if(status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to disable encoder");
   }

component_destroy:
   mmal_component_destroy(rawcam);
   mmal_component_destroy(isp);
   mmal_component_destroy(render);
   mmal_component_destroy(splitter);
   mmal_component_destroy(encoder);
   
   close(i2c_fd);
   return 0;
}



#define MAX_ENCODINGS_NUM 20
typedef struct {
   MMAL_PARAMETER_HEADER_T header;
   MMAL_FOURCC_T encodings[MAX_ENCODINGS_NUM];
} MMAL_SUPPORTED_ENCODINGS_T;

void display_supported_encodings(MMAL_PORT_T *port)
{
   MMAL_SUPPORTED_ENCODINGS_T sup_encodings = {{MMAL_PARAMETER_SUPPORTED_ENCODINGS, sizeof(sup_encodings)}, {0}};
   if (mmal_port_parameter_get(port, &sup_encodings.header) == MMAL_SUCCESS)
   {
      int i;
      int num_encodings = (sup_encodings.header.size - sizeof(sup_encodings.header)) /
          sizeof(sup_encodings.encodings[0]);
      for (i=0; i<num_encodings; i++)
      {
         switch (sup_encodings.encodings[i])
         {
            case MMAL_ENCODING_I420:
               vcos_log_error("MMAL_ENCODING_I420");
            break;
            case MMAL_ENCODING_I420_SLICE:
               vcos_log_error("MMAL_ENCODING_I420_SLICE");
            break;
            case MMAL_ENCODING_YV12:
               vcos_log_error("MMAL_ENCODING_YV12");
            break;
            case MMAL_ENCODING_I422:
               vcos_log_error("MMAL_ENCODING_I422");
            break;
            case MMAL_ENCODING_I422_SLICE:
               vcos_log_error("MMAL_ENCODING_I422_SLICE");
            break;
            case MMAL_ENCODING_YUYV:
               vcos_log_error("MMAL_ENCODING_YUYV");
            break;
            case MMAL_ENCODING_YVYU:
               vcos_log_error("MMAL_ENCODING_YVYU");
            break;
            case MMAL_ENCODING_UYVY:
               vcos_log_error("MMAL_ENCODING_UYVY");
            break;
            case MMAL_ENCODING_VYUY:
               vcos_log_error("MMAL_ENCODING_VYUY");
            break;
            case MMAL_ENCODING_NV12:
               vcos_log_error("MMAL_ENCODING_NV12");
            break;
            case MMAL_ENCODING_NV21:
               vcos_log_error("MMAL_ENCODING_NV21");
            break;
            case MMAL_ENCODING_ARGB:
               vcos_log_error("MMAL_ENCODING_ARGB");
            break;
            case MMAL_ENCODING_RGBA:
               vcos_log_error("MMAL_ENCODING_RGBA");
            break;
            case MMAL_ENCODING_ABGR:
               vcos_log_error("MMAL_ENCODING_ABGR");
            break;
            case MMAL_ENCODING_BGRA:
               vcos_log_error("MMAL_ENCODING_BGRA");
            break;
            case MMAL_ENCODING_RGB16:
               vcos_log_error("MMAL_ENCODING_RGB16");
            break;
            case MMAL_ENCODING_RGB24:
               vcos_log_error("MMAL_ENCODING_RGB24");
            break;
            case MMAL_ENCODING_RGB32:
               vcos_log_error("MMAL_ENCODING_RGB32");
            break;
            case MMAL_ENCODING_BGR16:
               vcos_log_error("MMAL_ENCODING_BGR16");
            break;
            case MMAL_ENCODING_BGR24:
               vcos_log_error("MMAL_ENCODING_BGR24");
            break;
            case MMAL_ENCODING_BGR32:
               vcos_log_error("MMAL_ENCODING_BGR32");
            break;
         }
      }
   }
   else
   {
      vcos_log_error("Failed to get supported encodings");
   }
}
