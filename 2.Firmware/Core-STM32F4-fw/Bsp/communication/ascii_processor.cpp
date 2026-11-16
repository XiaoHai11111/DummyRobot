/*
* The ASCII protocol is a simpler, human readable alternative to the main native
* protocol.
* In the future this protocol might be extended to support selected GCode commands.
* For a list of supported commands see doc/ascii-protocol.md
*/

/* Includes ------------------------------------------------------------------*/

#include "common_inc.h"
#include "ascii_processor.hpp"
#include <cstdarg>
#include <cstdio>

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/
#define MAX_LINE_LENGTH 256
#define TO_STR_INNER(s) #s
#define TO_STR(s) TO_STR_INNER(s)
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/


void Respond(StreamSink &output, const char* fmt, ...)
{
    char response[160];
    va_list args;
    va_start(args, fmt);
    const int length = vsnprintf(response, sizeof(response) - 2, fmt, args);
    va_end(args);
    if (length < 0) return;
    size_t len = static_cast<size_t>(length);
    if (len >= sizeof(response) - 2) len = sizeof(response) - 3;
    // One transport transaction per line, including its terminator. The USB
    // and UART sinks serialize the whole transaction across packet boundaries.
    response[len++] = '\r';
    response[len++] = '\n';
    output.process_bytes(reinterpret_cast<const uint8_t*>(response), len, nullptr);
}

// @brief Executes an ASCII protocol command
// @param buffer buffer of ASCII encoded characters
// @param len size of the buffer
void ASCII_protocol_process_line(const uint8_t* buffer, size_t len, StreamSink &response_channel)
{
    static_assert(sizeof(char) == sizeof(uint8_t));

    // copy everything into a local buffer so we can insert null-termination
    char cmd[MAX_LINE_LENGTH + 1];
    if (len > MAX_LINE_LENGTH) len = MAX_LINE_LENGTH;
    memcpy(cmd, buffer, len);

    cmd[len] = 0; // null-terminate

    if (response_channel.channelType == StreamSink::CHANNEL_TYPE_USB)
        OnUsbAsciiCmd(cmd, len, response_channel);
    else if (response_channel.channelType == StreamSink::CHANNEL_TYPE_UART4)
        OnUart4AsciiCmd(cmd, len, response_channel);
    else if (response_channel.channelType == StreamSink::CHANNEL_TYPE_UART5)
        OnUart5AsciiCmd(cmd, len, response_channel);
}

void ASCII_protocol_parse_stream(const uint8_t* buffer, size_t len, StreamSink &response_channel)
{
    struct ParserState
    {
        uint8_t buffer[MAX_LINE_LENGTH]{};
        bool active = true;
        uint32_t index = 0;
    };
    static ParserState channels[3];
    const unsigned channel = response_channel.channelType == StreamSink::CHANNEL_TYPE_USB ? 0 :
                             response_channel.channelType == StreamSink::CHANNEL_TYPE_UART4 ? 1 : 2;
    auto &parse_buffer = channels[channel].buffer;
    auto &read_active = channels[channel].active;
    auto &parse_buffer_idx = channels[channel].index;

    while (len--)
    {
        // if the line becomes too long, reset buffer and wait for the next line
        if (parse_buffer_idx >= MAX_LINE_LENGTH)
        {
            read_active = false;
            parse_buffer_idx = 0;
        }

        // Fetch the next char
        uint8_t c = *(buffer++);
        bool is_end_of_line = (c == '\r'|| c == '\n');//was \r \n
        if (is_end_of_line)
        {
            if (read_active && parse_buffer_idx)
                ASCII_protocol_process_line(parse_buffer, parse_buffer_idx, response_channel);
            parse_buffer_idx = 0;
            read_active = true;
        } else
        {
            if (read_active)
            {
                parse_buffer[parse_buffer_idx++] = c;
            }
        }
    }
}
