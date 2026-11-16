#ifndef __ASCII_PROTOCOL_H
#define __ASCII_PROTOCOL_H


/* Includes ------------------------------------------------------------------*/
#include <fibre/protocol.hpp>

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void ASCII_protocol_parse_stream(const uint8_t* buffer, size_t len, StreamSink& response_channel);
void OnUsbAsciiCmd(const char* _cmd, size_t _len, StreamSink& _responseChannel);
void OnUart4AsciiCmd(const char* _cmd, size_t _len, StreamSink& _responseChannel);
void OnUart5AsciiCmd(const char* _cmd, size_t _len, StreamSink& _responseChannel);

// Function to send messages back through specific channel (UART or USB-VCP).
// Use this function instead of printf because printf will send messages over ALL CHANNEL.
// Keep the formatting buffer in one out-of-line function. Inlining a template
// at every branch can reserve all reply buffers in the caller's stack frame.
void Respond(StreamSink &output, const char* fmt, ...)
    __attribute__((format(printf, 2, 3), noinline));


#endif /* __ASCII_PROTOCOL_H */
