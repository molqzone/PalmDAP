#pragma once
#include <cstdint>

namespace DAP
{

// DAP Command IDs
enum class CommandId : uint8_t
{
  // Core Commands (0x00-0x0F)
  Info = 0x00,
  HostStatus = 0x01,
  Connect = 0x02,
  Disconnect = 0x03,
  TransferConfigure = 0x04,
  Transfer = 0x05,
  TransferBlock = 0x06,
  TransferAbort = 0x07,
  WriteABORT = 0x08,
  Delay = 0x09,
  ResetTarget = 0x0A,

  // SWJ (Serial Wire JTAG) Commands (0x10-0x1F)
  SWJ_Pins = 0x10,
  SWJ_Clock = 0x11,
  SWJ_Sequence = 0x12,
  SWD_Configure = 0x13,
  JTAG_Sequence = 0x14,
  JTAG_Configure = 0x15,
  JTAG_IDCODE = 0x16,

  // SWO (Serial Wire Output) Commands (0x17-0x1D)
  SWO_Transport = 0x17,
  SWO_Mode = 0x18,
  SWO_Baudrate = 0x19,
  SWO_Control = 0x1A,
  SWO_Status = 0x1B,
  SWO_Data = 0x1D,  // New in v2

  // UART Commands (0x1E-0x22) - New in v2
  UART_Transport = 0x1E,
  UART_Configure = 0x1F,
  UART_Status = 0x20,
  UART_Control = 0x21,
  UART_Transfer = 0x22,

  // Additional SWD Command (0x1C) - New in v2
  SWD_Sequence = 0x1C,

  // Command Queue Commands (0x7E-0x7F)
  QueueCommands = 0x7E,
  ExecuteCommands = 0x7F,

  // Vendor Commands (0x80-0x9F) - Reserved for custom functionality
  VendorStart = 0x80,
  VendorEnd = 0x9F,

  // Invalid Command Response
  Invalid = 0xFF
};

// Information IDs
enum class InfoId : uint8_t
{
  Vendor = 0x01,
  Product = 0x02,
  SerialNumber = 0x03,
  FirmwareVersion = 0x04,
  DeviceVendor = 0x05,
  DeviceName = 0x06,
  BoardVendor = 0x07,
  BoardName = 0x08,
  ProductFirmwareVersion = 0x09,
  Capabilities = 0xF0,
  TimestampClock = 0xF1,
  SWO_BufferSize = 0xFD,
  PacketCount = 0xFE,
  PacketSize = 0xFF
};

// Helper functions for CommandId enum
inline constexpr bool IsVendorCommand(CommandId cmd)
{
  uint8_t cmd_val = static_cast<uint8_t>(cmd);
  return cmd_val >= 0x80 && cmd_val <= 0x9F;  // Vendor command range
}

inline constexpr bool IsValidCommand(CommandId cmd)
{
  uint8_t cmd_val = static_cast<uint8_t>(cmd);
  return cmd_val != static_cast<uint8_t>(CommandId::Invalid) && cmd_val <= 0x9F;
}

// Vendor Commands (0x80-0x9F)
// Reserved for vendor-specific functionality. Total of 32 commands available.

enum class VendorCommandId : uint8_t
{
  Vendor0 = 0x80,
  Vendor1 = 0x81,
  Vendor2 = 0x82,
  Vendor3 = 0x83,
  Vendor4 = 0x84,
  Vendor5 = 0x85,
  Vendor6 = 0x86,
  Vendor7 = 0x87,
  Vendor8 = 0x88,
  Vendor9 = 0x89,
  Vendor10 = 0x8A,
  Vendor11 = 0x8B,
  Vendor12 = 0x8C,
  Vendor13 = 0x8D,
  Vendor14 = 0x8E,
  Vendor15 = 0x8F,
  Vendor16 = 0x90,
  Vendor17 = 0x91,
  Vendor18 = 0x92,
  Vendor19 = 0x93,
  Vendor20 = 0x94,
  Vendor21 = 0x95,
  Vendor22 = 0x96,
  Vendor23 = 0x97,
  Vendor24 = 0x98,
  Vendor25 = 0x99,
  Vendor26 = 0x9A,
  Vendor27 = 0x9B,
  Vendor28 = 0x9C,
  Vendor29 = 0x9D,
  Vendor30 = 0x9E,
  Vendor31 = 0x9F
};

// DAP Status and Port Enums

enum class Status : uint8_t
{
  OK = 0x00,
  Error = 0xFF
};

enum class Port : uint8_t
{
  AutoDetect = 0x00,  // Deprecated
  Disabled = 0x00,
  SWD = 0x01,
  JTAG = 0x02
};

// DAP Transfer Request bits (in 'request' byte)

constexpr uint8_t DAP_TRANSFER_APnDP = (1U << 0);
constexpr uint8_t DAP_TRANSFER_RnW = (1U << 1);
constexpr uint8_t DAP_TRANSFER_A2 = (1U << 2);
constexpr uint8_t DAP_TRANSFER_A3 = (1U << 3);
constexpr uint8_t DAP_TRANSFER_MATCH_VALUE = (1U << 4);
constexpr uint8_t DAP_TRANSFER_MATCH_MASK = (1U << 5);
constexpr uint8_t DAP_TRANSFER_TIMESTAMP = (1U << 7);  // New in v2

// DAP Transfer Response bits (in 'response' byte)

constexpr uint8_t DAP_TRANSFER_OK = (1U << 0);
constexpr uint8_t DAP_TRANSFER_WAIT = (1U << 1);
constexpr uint8_t DAP_TRANSFER_FAULT = (1U << 2);
constexpr uint8_t DAP_TRANSFER_ERROR = (1U << 3);
constexpr uint8_t DAP_TRANSFER_MISMATCH = (1U << 4);
constexpr uint8_t DAP_TRANSFER_NO_TARGET = (1U << 7);  // New in v2

// SWJ (Serial Wire JTAG) Constants

// SWJ_Pins bits
constexpr uint8_t DAP_SWJ_SWCLK_TCK = (1U << 0);
constexpr uint8_t DAP_SWJ_SWDIO_TMS = (1U << 1);
constexpr uint8_t DAP_SWJ_TDI = (1U << 2);
constexpr uint8_t DAP_SWJ_TDO = (1U << 3);
constexpr uint8_t DAP_SWJ_nTRST = (1U << 5);
constexpr uint8_t DAP_SWJ_nRESET = (1U << 7);

// SWD (Serial Wire Debug) Constants

// SWD_Sequence bits
constexpr uint8_t SWD_SEQUENCE_CLK = 0x3F;       // Number of TCK cycles
constexpr uint8_t SWD_SEQUENCE_DIN = (1U << 7);  // SWDIO sampled

// JTAG Constants

// JTAG_Sequence bits
constexpr uint8_t JTAG_SEQUENCE_TCK = 0x3F;       // Number of TCK cycles
constexpr uint8_t JTAG_SEQUENCE_TMS = (1U << 6);  // TMS value
constexpr uint8_t JTAG_SEQUENCE_TDO = (1U << 7);  // TDO captured

// Debug Port Register Addresses (for SWD/JTAG Transfer commands)

// DP Registers (APnDP=0)
constexpr uint8_t DP_IDCODE = 0x00;     // Read only
constexpr uint8_t DP_ABORT = 0x00;      // Write only
constexpr uint8_t DP_CTRL_STAT = 0x04;  // R/W
constexpr uint8_t DP_WCR = 0x04;        // Write only (JTAG specific)
constexpr uint8_t DP_SELECT = 0x08;     // Write only
constexpr uint8_t DP_RDBUFF = 0x0C;     // Read only

// AP Registers (APnDP=1, address bits A3:A2 define the register)
// APBANKSEL in DP_SELECT selects the bank of 4 AP registers
constexpr uint8_t AP_CSW = 0x00;  // Control/Status Word
constexpr uint8_t AP_TAR = 0x04;  // Transfer Address Register
constexpr uint8_t AP_DRW = 0x0C;  // Data Read/Write Register
constexpr uint8_t AP_IDR = 0xFC;  // Identification Register (Bank 0xF)

}  // namespace DAP