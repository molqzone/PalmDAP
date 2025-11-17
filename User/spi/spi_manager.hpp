#pragma once

#include <array>
#include <cstdint>

#include "../xrdap/xrdap.hpp"
#include "dap_io.hpp"
#include "libxr.hpp"

namespace DAP
{

class SpiManager
{
 public:
  SpiManager(DapIo& io, LibXR::LockFreeQueue<uint64_t>& request_queue);
  ~SpiManager() = default;

  SpiManager(const SpiManager&) = delete;
  SpiManager& operator=(const SpiManager&) = delete;

  // Lifecycle management.
  void Initialize();
  void ProcessRequests();
  void Stop();

  // Raw mode operations (rst_n = 0): MOSI → SWDIO direct pass-through.
  LibXR::ErrorCode SendRawSequence(const uint8_t* bits, size_t bit_count);
  LibXR::ErrorCode ExecuteLineReset();

  // Transaction mode operations (rst_n = 1): First 15 bits have semantic meaning.
  LibXR::ErrorCode SendSwdFrame(uint64_t frame_data, uint32_t& response_data);
  LibXR::ErrorCode ExecuteSwdRead(uint8_t request, uint32_t* data, uint8_t* ack);
  LibXR::ErrorCode ExecuteSwdWrite(uint8_t request, uint32_t data, uint8_t* ack);

  // Queue management.
  void EnqueueFrame(uint64_t frame);
  bool HasPendingRequests() const;
  size_t GetQueueSize() const;

  // Status getters.
  bool initialized() const { return initialized_; }
  uint32_t transaction_count() const { return transaction_count_; }

 private:
  // Frame construction using XRDAP protocol.
  uint64_t BuildSwdReadFrame(uint8_t request);
  uint64_t BuildSwdWriteFrame(uint8_t request, uint32_t data);

  // Response processing using XRDAP protocol.
  SwdResponse ParseSwdResponse(uint64_t response_frame);

  // Low-level SPI communication.
  LibXR::ErrorCode SendSpi48(uint64_t tx_data, uint32_t* rx_data);

  DapIo& io_;
  LibXR::LockFreeQueue<uint64_t>& request_queue_;

  bool initialized_;
  uint32_t transaction_count_;
};

// Factory function.
SpiManager* CreateSpiManager(DapIo& io, LibXR::LockFreeQueue<uint64_t>& queue);

}  // namespace DAP