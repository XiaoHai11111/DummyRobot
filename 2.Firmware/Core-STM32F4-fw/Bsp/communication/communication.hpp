#ifndef COMMANDS_H
#define COMMANDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <cmsis_os.h>

void InitCommunication(void);
void CommitProtocol();
void CommunicationTask(void *ctx);
void UsbDeferredInterruptTask(void *ctx);

#ifdef __cplusplus
}

#include <functional>
#include <limits>
#include "ascii_processor.hpp"
#include "interface_usb.hpp"
#include "interface_uart.hpp"
#include "interface_can.hpp"

#define COMMIT_PROTOCOL \
using treeType = decltype(MakeObjTree());\
extern treeType protocolTree;\
void CommitProtocol()\
{\
    fibre_publish(protocolTree);\
}\


#endif
#endif /* COMMANDS_H */
