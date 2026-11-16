#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include <deque>
#include <cstring>
#include <cstdio>
#include <utility>
struct StreamSink {
    enum Channel { CHANNEL_TYPE_USB, CHANNEL_TYPE_UART4, CHANNEL_TYPE_UART5 };
    Channel channelType = CHANNEL_TYPE_USB;
    std::string output;
    virtual int process_bytes(const uint8_t* data, size_t size, size_t*) {
        output.append(reinterpret_cast<const char*>(data), size); return 0;
    }
    virtual size_t get_free_space() { return SIZE_MAX; }
};
struct PacketSink {
    virtual int process_packet(const uint8_t*, size_t) = 0;
};
struct BidirectionalPacketBasedChannel {
    explicit BidirectionalPacketBasedChannel(PacketSink&) {}
    void process_packet(const uint8_t*, size_t) {}
};
template<class... T> int make_protocol_member_list(T&&...) { return 0; }
template<class... T> int make_protocol_function(T&&...) { return 0; }
template<class... T> int make_protocol_object(T&&...) { return 0; }
template<class... T> int make_protocol_ro_property(T&&...) { return 0; }
struct HostQueue { unsigned capacity, size; std::deque<std::vector<char>> items; };
using osMessageQueueId_t = HostQueue*;
using osStatus_t = int;
constexpr int osOK = 0;
inline HostQueue* osMessageQueueNew(unsigned count, unsigned size, void*) { return new HostQueue{count, size, {}}; }
inline int osMessageQueuePut(HostQueue* q, const void* data, unsigned, unsigned) {
    if (q->items.size() == q->capacity) return -1;
    q->items.emplace_back(static_cast<const char*>(data), static_cast<const char*>(data) + q->size); return 0;
}
inline int osMessageQueueGet(HostQueue* q, void* data, void*, unsigned) {
    if (q->items.empty()) return -1;
    memcpy(data, q->items.front().data(), q->size); q->items.pop_front(); return 0;
}
inline unsigned osMessageQueueGetSpace(HostQueue* q) { return q->capacity - q->items.size(); }
inline void osMessageQueueReset(HostQueue* q) { q->items.clear(); }
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
void osDelay(unsigned ticks);
