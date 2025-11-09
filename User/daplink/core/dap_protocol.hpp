#pragma once

#include <cstdint>

#include "dap_constants.hpp"
#include "dap_io.hpp"
#include "libxr.hpp"

namespace DAP
{

// -- Constants --
constexpr uint16_t kMaxRequestSize = 512;
constexpr uint16_t kMaxResponseSize = 512;
constexpr uint16_t kDefaultRetryCount = 100;
constexpr uint8_t kDefaultIdleCycles = 0;
constexpr uint8_t kMaxJtagDevices = 8;

// Internal state machine
enum class DapPort : uint8_t
{
  DISABLED = 0,
  SWD = 1,
  JTAG = 2
};

class DapProtocol
{
 public:
  explicit DapProtocol(DapIo& io);

  /**
   * @brief Execute DAP command
   * Main entry point for DAP protocol processing.
   * @param request Pointer to request buffer (contains command ID and parameters)
   * @param response Pointer to response buffer (will be filled with response data)
   * @param in_isr Whether this command is being executed in an interrupt context
   * @return Total response length in bytes
   */
  uint32_t ExecuteCommand(const uint8_t* request, uint8_t* response, bool in_isr = false);

  /**
   * @brief Reset protocol state
   */
  void Reset();

  /**
   * @brief Get current debug port status
   */
  DapPort GetDebugPort() const { return state_.debug_port; }

  // SPI operation synchronization objects - public for external access
  LibXR::Semaphore spi_sem_;
  LibXR::WriteOperation spi_write_op_;
  LibXR::WriteOperation::OperationPollingStatus spi_isr_polling_status_;
  LibXR::WriteOperation spi_isr_write_op_;

 private:
  struct JtagDevice
  {
    uint8_t ir_length = 0;
    uint16_t ir_before = 0;
    uint16_t ir_after = 0;
  };

  struct TransferConfig
  {
    uint8_t idle_cycles = kDefaultIdleCycles;
    uint16_t retry_count = kDefaultRetryCount;
    uint16_t match_retry = 0;
    uint32_t match_mask = 0;
  };

  struct SwdConfig
  {
    uint8_t turnaround = 1;
    bool data_phase = false;
  };

  struct JtagContext
  {
    uint8_t index = 0;
    JtagDevice devices[kMaxJtagDevices];
    uint8_t device_count = 0;
  };

  struct State
  {
    DapPort debug_port = DapPort::DISABLED;
    volatile bool transfer_abort = false;
    TransferConfig transfer_config;
    SwdConfig swd_config;
    JtagContext jtag_context;
  };

  // Command processing result
  struct CommandResult
  {
    uint16_t request_consumed =
        0;  // Bytes consumed from request (including command byte)
    uint16_t response_generated = 0;  // Bytes generated for response
  };

  // Core Processing
  void Setup();
  CommandResult ProcessCommand(const uint8_t* request, uint8_t* response, bool in_isr = false);

  // Command Handlers
  CommandResult HandleInfo(const uint8_t* req, uint8_t* res);
  CommandResult HandleHostStatus(const uint8_t* req, uint8_t* res);
  CommandResult HandleConnect(const uint8_t* req, uint8_t* res, bool in_isr = false);
  CommandResult HandleDisconnect(uint8_t* res, bool in_isr = false);
  CommandResult HandleTransferConfigure(const uint8_t* req, uint8_t* res);
  CommandResult HandleTransfer(const uint8_t* req, uint8_t* res);
  CommandResult HandleTransferBlock(const uint8_t* req, uint8_t* res);
  CommandResult HandleWriteAbort(const uint8_t* req, uint8_t* res);
  CommandResult HandleDelay(const uint8_t* req, uint8_t* res);
  CommandResult HandleResetTarget(uint8_t* res);
  CommandResult HandleSwjPins(const uint8_t* req, uint8_t* res);
  CommandResult HandleSwjClock(const uint8_t* req, uint8_t* res);
  CommandResult HandleSwjSequence(const uint8_t* req, uint8_t* res);
  CommandResult HandleSwdConfigure(const uint8_t* req, uint8_t* res);
  CommandResult HandleSwdSequence(const uint8_t* req, uint8_t* res);
  CommandResult HandleJtagSequence(const uint8_t* req, uint8_t* res);
  CommandResult HandleJtagConfigure(const uint8_t* req, uint8_t* res);
  CommandResult HandleJtagIdcode(const uint8_t* req, uint8_t* res);

  // Transfer Operations
  CommandResult SwdTransfer(const uint8_t* req, uint8_t* res);
  CommandResult JtagTransfer(const uint8_t* req, uint8_t* res);
  CommandResult DummyTransfer(const uint8_t* req, uint8_t* res);
  uint8_t SwdTransferSingle(uint8_t request, uint32_t* data);

  // Prevent copying
  DapProtocol(const DapProtocol&) = delete;
  DapProtocol& operator=(const DapProtocol&) = delete;

 private:
  DapIo& io_;

  State state_;
  uint8_t response_buffer_[kMaxResponseSize];

  using InfoHandler = std::function<uint8_t(uint8_t* response_data_buffer)>;
  struct InfoEntry
  {
    InfoId id;
    InfoHandler handler;
  };

  LibXR::ErrorCode SetupSwd(bool in_isr = false);
  LibXR::ErrorCode SetupJtag(bool in_isr = false);
  void PortOff(bool in_isr = false);
};

}  // namespace DAP
