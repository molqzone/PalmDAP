#pragma once

#include <cstdint>

namespace DAP
{

// XRDAP-SWD-Probe protocol constants.
constexpr int kSpiFrameBits = 48;
constexpr int kSpiFrameBytes = 6;
constexpr int kAckBits = 3;
constexpr int kPaddingBits = 3;
constexpr int kRequestBits = 8;

// XRDAP-SWD-Probe 48-bit request frame structure (LSB first):
struct SpiFrame
{
  uint64_t raw;

  // Bit field definitions:
  // bits 0-2:   PADDING (3 zeros for alignment)
  // bits 3-10:  REQ_WINDOW (8-bit SWD request)
  // bit 11:     TURN1 (REQ�ACK turnaround)
  // bits 12-14: ACK_WINDOW (3-bit ACK from target - capture only)
  // bits 15-47: AFTER_ACK_STREAM (write data or padding)

  enum BitPositions
  {
    kPaddingStart = 0,
    kPaddingEnd = 2,
    kReqStart = 3,
    kReqEnd = 10,
    kTurn1Pos = 11,
    kAckStart = 12,
    kAckEnd = 14,
    kAfterAckStart = 15,
    kAfterAckEnd = 47
  };

  enum BitMasks
  {
    kPaddingMask = 0x07ULL,           // bits 0-2
    kReqMask = 0xFFULL << 3,          // bits 3-10
    kTurn1Mask = 0x1ULL << 11,        // bit 11
    kAckMask = 0x7ULL << 12,          // bits 12-14
    kAfterAckMask = 0x1FFFFULL << 15  // bits 15-47
  };

  inline uint8_t GetPadding() const { return (raw & kPaddingMask); }
  inline uint8_t GetRequest() const { return (raw & kReqMask) >> kReqStart; }
  inline bool GetTurn1() const { return (raw & kTurn1Mask) != 0; }
  inline uint8_t GetAck() const { return (raw & kAckMask) >> kAckStart; }
  inline uint32_t GetAfterAck() const { return (raw & kAfterAckMask) >> kAfterAckStart; }

  inline void SetPadding(uint8_t padding)
  {
    raw = (raw & ~kPaddingMask) | (padding & kPaddingMask);
  }
  inline void SetRequest(uint8_t request)
  {
    raw = (raw & ~kReqMask) | ((uint64_t)request << kReqStart);
  }
  inline void SetTurn1(bool turn1)
  {
    raw = (raw & ~kTurn1Mask) | (turn1 ? kTurn1Mask : 0);
  }
  inline void SetAck(uint8_t ack)
  {
    raw = (raw & ~kAckMask) | ((uint64_t)ack << kAckStart);
  }
  inline void SetAfterAck(uint32_t data)
  {
    raw = (raw & ~kAfterAckMask) | ((uint64_t)data << kAfterAckStart);
  }

  inline void BuildFrame(uint8_t request, uint32_t after_ack_data)
  {
    raw = 0;
    SetPadding(0);                // 3 zeros
    SetRequest(request);          // 8-bit SWD request
    SetTurn1(false);              // Clear for transmission
    SetAck(0);                    // Clear for transmission
    SetAfterAck(after_ack_data);  // Write data or padding
  }
};

// ACK values.
constexpr uint8_t kAckOk = 0x01;     // 001
constexpr uint8_t kAckWait = 0x02;   // 010
constexpr uint8_t kAckFault = 0x04;  // 100

// DAP Transfer request constants for XRDAP translation.
constexpr uint8_t DAP_TRANSFER_APnDP = (1U << 0);
constexpr uint8_t DAP_TRANSFER_RnW = (1U << 1);
constexpr uint8_t DAP_TRANSFER_A2 = (1U << 2);
constexpr uint8_t DAP_TRANSFER_A3 = (1U << 3);

// XRDAP-SWD-Probe protocol translation functions.

// Convert DAP transfer request to XRDAP frame request byte.
// DAP format: [APnDP][RnW][A2][A3][parity] + start/stop bits
// XRDAP format: 8-bit request positioned at bits 3-10 of 48-bit frame
inline uint8_t TranslateDapToXrdapRequest(uint8_t dap_request)
{
  // Extract DAP request bits
  bool apndp = (dap_request & DAP_TRANSFER_APnDP) != 0;
  bool rnw = (dap_request & DAP_TRANSFER_RnW) != 0;
  bool a2 = (dap_request & DAP_TRANSFER_A2) != 0;
  bool a3 = (dap_request & DAP_TRANSFER_A3) != 0;

  // Calculate parity for APnDP + RnW + A2 + A3
  bool parity = apndp ^ rnw ^ a2 ^ a3;

  // Build XRDAP request byte: [APnDP][RnW][A2][A3][parity][0][0][0]
  uint8_t xrdap_request = 0;
  if (apndp) xrdap_request |= (1U << 4);
  if (rnw) xrdap_request |= (1U << 5);
  if (a2) xrdap_request |= (1U << 6);
  if (a3) xrdap_request |= (1U << 7);
  if (parity) xrdap_request |= (1U << 3);

  return xrdap_request;
}

// Translate DAP transfer to XRDAP 48-bit frame.
inline SpiFrame TranslateDapToXrdapFrame(uint8_t dap_request, uint32_t write_data = 0)
{
  uint8_t xrdap_req = TranslateDapToXrdapRequest(dap_request);
  SpiFrame frame;
  frame.BuildFrame(xrdap_req, write_data);
  return frame;
}

// Extract SWD protocol fields from XRDAP frame response.
struct SwdResponse
{
  uint8_t ack;        // 3-bit ACK value
  uint32_t data;      // 32-bit data (for reads)
  bool parity_valid;  // Data parity check result
};

// Parse XRDAP response frame to extract SWD response.
inline SwdResponse ParseXrdapResponse(uint64_t response_frame)
{
  SpiFrame frame;
  frame.raw = response_frame;

  SwdResponse response;
  response.ack = frame.GetAck() & 0x07;  // Extract 3-bit ACK
  response.data = frame.GetAfterAck();   // Extract 32-bit data

  // Calculate parity for data (optional implementation)
  response.parity_valid = true;  // TODO: Implement actual parity check

  return response;
}

// DAP Transfer status constants.
constexpr uint8_t DAP_TRANSFER_OK = (1U << 0);
constexpr uint8_t DAP_TRANSFER_WAIT = (1U << 1);
constexpr uint8_t DAP_TRANSFER_FAULT = (1U << 2);
constexpr uint8_t DAP_TRANSFER_ERROR = (1U << 3);
constexpr uint8_t DAP_TRANSFER_MISMATCH = (1U << 4);

// Convert XRDAP ACK to DAP Transfer status.
inline uint8_t ConvertXrdapAckToDapStatus(uint8_t xrdap_ack)
{
  switch (xrdap_ack)
  {
    case kAckOk:
      return DAP_TRANSFER_OK;
    case kAckWait:
      return DAP_TRANSFER_WAIT;
    case kAckFault:
      return DAP_TRANSFER_FAULT;
    default:
      return DAP_TRANSFER_ERROR;
  }
}

}  // namespace DAP