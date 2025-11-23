#pragma once

#include <cstdint>

#include "dap_io.hpp"
#include "libxr.hpp"
#include "spi_types.hpp"

namespace DAP
{

/**
 * @brief Manages SPI operations for XRDAP-SWD-Probe communication.
 *
 * This class handles asynchronous SPI transactions using a dedicated FreeRTOS task.
 * It processes SPI transfer requests from a lock-free queue and manages callbacks
 * for operation completion.
 */
class SpiManager
{
 public:
  /**
   * @brief Constructs a SpiManager instance.
   *
   * @param io DAP I/O interface reference for SPI access.
   * @param request_queue Lock-free queue for SPI transfer requests.
   */
  SpiManager(DapIo& io, LibXR::LockFreeQueue<SpiTransferRequest>& request_queue);

  ~SpiManager() = default;

  SpiManager(const SpiManager&) = delete;
  SpiManager& operator=(const SpiManager&) = delete;

  /**
   * @brief Initializes the SPI hardware and starts the SpiManager task.
   *
   * Configures SPI for XRDAP communication and creates a dedicated FreeRTOS task
   * for processing SPI transfer requests.
   */
  void Initialize();

  /**
   * @brief Stops SPI operations and terminates the SpiManager task.
   */
  void Stop();

  /**
   * @brief Checks if there are pending SPI transfer requests.
   *
   * @return True if the request queue has pending items, false otherwise.
   */
  bool HasPendingRequests() const;

  /**
   * @brief Gets the current size of the request queue.
   *
   * @return Number of pending SPI transfer requests.
   */
  size_t GetQueueSize() const;

  /**
   * @brief Gets the initialization status of the SpiManager.
   *
   * @return True if the SpiManager is initialized and ready, false otherwise.
   */
  bool initialized() const { return initialized_; }

 private:
  /**
   * @brief Handles SPI operation completion and triggers response callback.
   *
   * @param request The SPI transfer request that was completed.
   * @param ec Error code from the SPI operation.
   */
  void HandleSpiCompletion(const SpiTransferRequest& request, LibXR::ErrorCode ec);

  /**
   * @brief Static callback wrapper for SPI operations.
   *
   * @param in_isr True if called from interrupt context, false otherwise.
   * @param context Context pointer to the SpiManager instance.
   * @param ec Error code from the SPI operation.
   */
  static void SpiCallbackWrapper(bool in_isr, int context, LibXR::ErrorCode ec);

  /**
   * @brief SpiManager task function that processes SPI transfer requests.
   *
   * This function runs in a dedicated FreeRTOS task and continuously processes
   * SPI transfer requests from the queue.
   *
   * @param arg Pointer to the SpiManager instance.
   */
  static void SpiManagerTask(void* arg);

  DapIo& io_;                             ///< DAP I/O interface reference
  LibXR::LockFreeQueue<SpiTransferRequest>&
      request_queue_;                   ///< SPI transfer request queue
  SpiTransferRequest current_request_;  ///< Current request being processed
  bool initialized_;                    ///< Initialization status flag
  LibXR::Thread spi_thread_;            ///< Dedicated FreeRTOS task
  uint8_t rx_data_[6];                  ///< Static RX buffer for SPI operations
};

/**
 * @brief Factory function to create a SpiManager instance.
 *
 * Uses static allocation to avoid dynamic memory allocation in embedded systems.
 *
 * @param io DAP I/O interface reference for SPI access.
 * @param queue Lock-free queue for SPI transfer requests.
 * @return Reference to the created SpiManager instance.
 */
SpiManager& CreateSpiManager(DapIo& io, LibXR::LockFreeQueue<SpiTransferRequest>& queue);

}  // namespace DAP