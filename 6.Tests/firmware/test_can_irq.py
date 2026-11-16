"""Compile the real CAN1 RX entry with a HAL double modelling sticky FIFO flags.

This tests IRQ routing, not bxCAN electrical behaviour or the vendor HAL itself.
"""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / "2.Firmware/Core-STM32F4-fw/Core/Src/stm32f4xx_it.c").read_text()
start = source.index("void CAN1_RX0_IRQHandler(void)")
opening = source.index("{", start)
depth = 1
end = opening + 1
while depth:
    depth += (source[end] == "{") - (source[end] == "}")
    end += 1
entry = source[start:end]

harness = r"""
#include <stdint.h>
#include <assert.h>
#define READ_REG(reg) (reg)
#define CAN_IT_RX_FIFO0_MSG_PENDING 2U
#define CAN_RF0R_FMP0 3U
#define FULL 8U
#define OVERRUN 16U
typedef struct { uint32_t IER, RF0R; } Registers;
typedef struct { Registers* Instance; } CAN_HandleTypeDef;
Registers registers;
CAN_HandleTypeDef hcan1 = {&registers};
unsigned hal_calls, received;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* handle) {
    assert(handle == &hcan1);
    ++received;
    // Reading a message alone does not acknowledge a latched overrun.
    handle->Instance->RF0R &= ~CAN_RF0R_FMP0;
}
void HAL_CAN_IRQHandler(CAN_HandleTypeDef* handle) {
    assert(handle == &hcan1);
    ++hal_calls;
    handle->Instance->RF0R &= ~(FULL | OVERRUN);
    if (handle->Instance->RF0R & CAN_RF0R_FMP0)
        HAL_CAN_RxFifo0MsgPendingCallback(handle);
}
"""
checks = r"""
int main(void) {
    // Include sticky flags with no pending frame: the old fast path returned
    // without acknowledgement and would immediately enter this IRQ again.
    const uint32_t cases[] = {2, FULL, OVERRUN, FULL | OVERRUN, 3 | FULL | OVERRUN};
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        registers.IER = CAN_IT_RX_FIFO0_MSG_PENDING | FULL | OVERRUN;
        registers.RF0R = cases[i];
        hal_calls = received = 0;
        CAN1_RX0_IRQHandler();
        assert(hal_calls == 1);
        assert(registers.RF0R == 0);
        assert(received == ((cases[i] & CAN_RF0R_FMP0) != 0));
    }
}
"""
with tempfile.TemporaryDirectory(prefix="dummy-can-irq-") as directory:
    code = Path(directory) / "irq.c"
    executable = Path(directory) / "irq"
    code.write_text(harness + entry + checks)
    subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Werror",
                    str(code), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
print("CAN1 IRQ pending/full/overrun routing tests passed")
