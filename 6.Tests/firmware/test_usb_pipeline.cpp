// Real USB worker -> real ASCII parser/commands/formatter -> real USB sender.
// Only the RTOS and hardware CDC transfer are doubles; no physical USB claim.
#include "common_inc.h"
#include "interface_usb.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include "FreeRTOS.h"

struct EndOfInput {};
static void (*worker)(void*);
static std::deque<std::string> requests;
static std::string rxBuffer, reply;
static std::vector<size_t> txSizes;
static bool armed = true, token = true;
static unsigned rearms = 0;
static bool injectReply = false, parseQueuedReply = false;
static std::atomic<bool> contenderWaiting{false};
static std::thread contender;
static const auto mainThread = std::this_thread::get_id();
osMutexId_t osMutexNew(const osMutexAttr_t* attr) {
    assert(attr->cb_mem && attr->cb_size == sizeof(StaticSemaphore_t));
    assert(attr->attr_bits == osMutexPrioInherit);
    return &static_cast<StaticSemaphore_t*>(attr->cb_mem)->mutex;
}
int osMutexAcquire(osMutexId_t id, uint32_t timeout) {
    if (std::this_thread::get_id() != mainThread) contenderWaiting = true;
    return static_cast<std::timed_mutex*>(id)->try_lock_for(std::chrono::milliseconds(timeout)) ? osOK : -1;
}
int osMutexRelease(osMutexId_t id) {
    static_cast<std::timed_mutex*>(id)->unlock(); return osOK;
}
osSemaphoreId sem_usb_rx = reinterpret_cast<void*>(1);
osSemaphoreId sem_usb_tx = reinterpret_cast<void*>(2);
USBD_HandleTypeDef hUsbDeviceFS;
CAN_HandleTypeDef can, hcan1;
CAN_context ctx{&can};
DummyRobot dummy(&can);
CAN_context* get_can_ctx(CAN_HandleTypeDef*) { return &ctx; }
bool CanSendMessage(CAN_context*, uint8_t*, CAN_TxHeaderTypeDef*) { return true; }
uint32_t HAL_GetTick() { return 0; }
void HAL_NVIC_SystemReset() {}
void osDelay(unsigned) {}
osThreadId_t osThreadNew(void (*entry)(void*), void*, const osThreadAttr_t* attr) {
    assert(attr->cb_mem == nullptr && attr->stack_mem == nullptr);
    assert(attr->stack_size == 4096);
    worker = entry;
    return reinterpret_cast<void*>(3);
}
int osSemaphoreAcquire(osSemaphoreId id, uint32_t timeout) {
    if (id == sem_usb_tx) {
        assert(timeout == PROTOCOL_SERVER_TIMEOUT_MS);
        if (!token) return -1;
        token = false;
        return osOK;
    }
    assert(id == sem_usb_rx && timeout == osWaitForever);
    if (requests.empty()) throw EndOfInput{};
    assert(armed); // The preceding packet must have been rearmed by the worker.
    armed = false;
    rxBuffer = requests.front(); requests.pop_front();
    usb_rx_process_packet(reinterpret_cast<uint8_t*>(&rxBuffer[0]), rxBuffer.size(), CDC_OUT_EP);
    return osOK;
}
int osSemaphoreRelease(osSemaphoreId id) {
    if (id == sem_usb_tx) token = true;
    else assert(id == sem_usb_rx);
    return osOK;
}
uint8_t CDC_Transmit_FS(uint8_t* data, uint16_t len, uint8_t endpoint) {
    assert(endpoint == CDC_OUT_EP && len <= USB_TX_DATA_SIZE);
    reply.append(reinterpret_cast<char*>(data), len);
    txSizes.push_back(len);
    if (injectReply) {
        injectReply = false;
        contenderWaiting = false;
        contender = std::thread([] {
            if (parseQueuedReply) dummy.commandHandler.ParseCommand(dummy.commandHandler.Pop(0));
            else Respond(*usbStreamOutputPtr, "ok");
        });
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (!contenderWaiting && std::chrono::steady_clock::now() < deadline) std::this_thread::yield();
        assert(contenderWaiting); // Competing writer reaches the lock between USB packets.
    }
    // Model transfer completion; host tests cannot prove that the real IRQ fires.
    osSemaphoreRelease(sem_usb_tx);
    return USBD_OK;
}
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef* handle, uint8_t endpoint) {
    assert(handle == &hUsbDeviceFS && endpoint == CDC_OUT_EP);
    armed = true; ++rearms;
    return USBD_OK;
}
void pump() {
    try { worker(nullptr); } catch (const EndOfInput&) {}
    assert(armed);
}
int main() {
    dummy.Init();
    StartUsbServer();
    assert(worker && UsbDeviceState() == 3);
    requests = {"#GET", "STATUS\r\n"};
    pump();
    assert(reply == "oks 0 2 0 0 0 0 0 0 0\r\n");
    assert(txSizes.size() == 1 && txSizes[0] == reply.size());
    assert(usb_stats_.cdc_rx_packets == 2 && usb_stats_.cdc_rx_processed == 2);
    assert(usb_stats_.task_running && rearms == 2);
    reply.clear(); txSizes.clear();
    requests = {"#GETSTATUS\n#GETSTATUS\n"};
    pump();
    assert(reply == "oks 0 2 0 0 0 0 0 0 0\r\noks 0 2 0 0 0 0 0 0 0\r\n");
    assert(rearms == 3);
    reply.clear(); txSizes.clear();
    injectReply = true;
    Respond(*usbStreamOutputPtr, "%s", std::string(500, 'x').c_str());
    contender.join();
    assert(reply == std::string(157, 'x') + "\r\nok\r\n");
    assert((txSizes == std::vector<size_t>{64, 64, 31, 4}));
    reply.clear(); txSizes.clear();
    injectReply = true;
    // Run the actual > queue path. The command consumer replies while the
    // USB task is still submitting the numeric acknowledgement.
    for (unsigned j = 1; j <= 6; ++j) dummy.motorJ[j]->UpdateStatus(0, 0, true);
    dummy.SetEnable(true);
    assert(dummy.IsEnabled());
    parseQueuedReply = true;
    requests = {">0,-75,180,0,0,0,30\r\n"};
    pump();
    contender.join();
    parseQueuedReply = false;
    dummy.SetEnable(false);
    assert(reply == "15\r\nok\r\n");
    assert((txSizes == std::vector<size_t>{4, 4}));
    reply.clear(); txSizes.clear();
    token = false; // A pending transfer never completed. Do not steal its buffer/token.
    const auto failures = usb_stats_.tx_failures;
    Respond(*usbStreamOutputPtr, "busy");
    assert(reply.empty() && !token && usb_stats_.tx_failures == failures + 1);
    osSemaphoreRelease(sem_usb_tx); // Completion or USB reinitialization restores availability.
    Respond(*usbStreamOutputPtr, "ready");
    assert(reply == "ready\r\n");
    std::cout << "USB worker, status replies, split/coalesced RX and TX framing tests passed\n";
}
