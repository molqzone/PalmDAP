#include "spi_manager.hpp"

#include "dap_constants.hpp"
#include "xrdap.hpp"

namespace DAP
{

SpiManager::SpiManager(DapIo& io, LibXR::LockFreeQueue<SpiTransferRequest>& request_queue)
    : io_(io),
      request_queue_(request_queue),
      initialized_(false),
      spi_callback_(LibXR::Callback<LibXR::ErrorCode>::Create(
          [](bool in_isr, SpiManager* manager, LibXR::ErrorCode ec)
          {
            UNUSED(in_isr);
            manager->HandleSpiCompletion(manager->current_request_, ec);
          },
          this))
{
}

void SpiManager::Initialize()
{
  // Initialize SPI for XRDAP communication
  io_.InitializeXrdapSpi();

  initialized_ = true;

  // Create SpiManager task
  spi_thread_.Create(static_cast<void*>(this), SpiManagerTask, "SpiManager", 1024,
                     LibXR::Thread::Priority::MEDIUM);
}

void SpiManager::SpiManagerTask(void* arg)
{
  SpiManager* manager = static_cast<SpiManager*>(arg);

  while (true)
  {
    if (!manager->initialized_)
    {
      LibXR::Thread::Sleep(10);
      continue;
    }

    // Process one pending SPI transfer request at a time for async operation
    if (manager->request_queue_.Pop(manager->current_request_) == LibXR::ErrorCode::OK)
    {
      // Extract original DAP request to determine read/write direction
      uint8_t original_request =
          static_cast<uint8_t>((manager->current_request_.spi_frame >> 2) & 0xFF);
      bool is_read = (original_request & DAP_TRANSFER_RnW);

      // XRDAP Protocol: Set rnw direction before frame starts
      // Hardware will automatically handle the 4 transaction types based on rnw + ack
      // detection
      manager->io_.SetReadWriteDirection(is_read);

      // XRDAP Protocol: Frame reset before each SPI transaction
      // "每帧开始前短暂拉低一次 rst_n，作为该帧的逻辑复位"
      manager->io_.ResetFrame();

      // Send 48-bit SPI frame using LibXR::SPI with callback
      uint8_t tx_data[6];

      // Convert 64-bit frame to 6-byte array (little-endian)
      for (int i = 0; i < 6; i++)
      {
        tx_data[i] = static_cast<uint8_t>(manager->current_request_.spi_frame >> (i * 8));
      }

      // Use pre-initialized callback for SPI operation completion
      LibXR::Callback<LibXR::ErrorCode> spi_callback = manager->spi_callback_;

      // Start asynchronous SPI transaction
      LibXR::SPI::OperationRW spi_op(spi_callback);

      LibXR::ErrorCode err =
          manager->io_.spi.ReadAndWrite({manager->rx_data_, 6}, {tx_data, 6}, spi_op);
      if (err != LibXR::ErrorCode::OK)
      {
        // Handle immediate error
        manager->HandleSpiCompletion(manager->current_request_, err);
      }
    }
    else
    {
      // No requests in queue, wait a bit
      LibXR::Thread::Sleep(10);
    }
  }
}

void SpiManager::Stop()
{
  // Stop SPI hardware
  initialized_ = false;
}

bool SpiManager::HasPendingRequests() const { return request_queue_.Size() > 0; }

size_t SpiManager::GetQueueSize() const { return request_queue_.Size(); }

void SpiManager::SpiCallbackWrapper(bool in_isr, int context, LibXR::ErrorCode ec)
{
  (void)in_isr;  // Suppress unused parameter warning

  // Get the SpiManager instance from context
  SpiManager* manager = reinterpret_cast<SpiManager*>(context);
  if (manager)
  {
    manager->HandleSpiCompletion(manager->current_request_, ec);
  }
}

void SpiManager::HandleSpiCompletion(const SpiTransferRequest& request,
                                     LibXR::ErrorCode ec)
{
  // If no callback is provided, USB will timeout waiting for response
  if (request.response_callback.Empty())
  {
    return;  // Cannot send response without callback
  }

  if (ec != LibXR::ErrorCode::OK)
  {
    // SPI operation failed
    static uint8_t error_response[3] = {static_cast<uint8_t>(DAP::CommandId::Transfer), 1,
                                        DAP_TRANSFER_ERROR};
    request.response_callback.Run(true, error_response, 3);
    return;
  }

  // Process actual SPI response using XRDAP protocol
  static uint8_t response[7];  // Max DAP response size
  size_t response_size = 0;

  // Extract original DAP request from SPI frame (bits 2-9)
  uint8_t original_request = static_cast<uint8_t>((request.spi_frame >> 2) & 0xFF);

  // Convert SPI response to DAP Transfer response
  bool success = DAP::GenerateDapTransferResponse(original_request, rx_data_, response,
                                                  response_size);

  if (success && response_size > 0)
  {
    // Successfully generated DAP response
    request.response_callback.Run(true, response, response_size);
  }
  else
  {
    // Response generation failed
    static uint8_t error_response[3] = {static_cast<uint8_t>(DAP::CommandId::Transfer), 1,
                                        DAP_TRANSFER_ERROR};
    request.response_callback.Run(true, error_response, 3);
  }
}

SpiManager& CreateSpiManager(DapIo& io, LibXR::LockFreeQueue<SpiTransferRequest>& queue)
{
  static SpiManager instance(io, queue);
  return instance;
}

}  // namespace DAP
