#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
build_dir=$(mktemp -d /tmp/dummy-firmware-tests.XXXXXX)
trap 'rm -rf "$build_dir"' EXIT
f1=2.Firmware/Ctrl-Step-Driver-STM32F1-fw
f4=2.Firmware/Core-STM32F4-fw
cxx=${CXX:-g++}
"$cxx" -std=c++17 -O2 -I6.Tests/firmware/stubs -I"$f1/Ctrl" -I"$f1/UserApp" \
  6.Tests/firmware/test_f1.cpp "$f1/Ctrl/Motor/motor.cpp" "$f1/Ctrl/Motor/motion_planner.cpp" -o "$build_dir/f1"
"$build_dir/f1"
"$cxx" -std=c++17 -O2 -I6.Tests/firmware/stubs/f1 -I6.Tests/firmware/stubs -I"$f1/Ctrl" -I"$f1/UserApp" \
  6.Tests/firmware/test_f1_can.cpp "$f1/Ctrl/Motor/motor.cpp" "$f1/Ctrl/Motor/motion_planner.cpp" \
  "$f1/UserApp/protocols/interface_can.cpp" -o "$build_dir/f1-can"
"$build_dir/f1-can"
"$cxx" -std=c++17 -Ofast -I"$f4/Robot" 6.Tests/firmware/test_motion_command.cpp -o "$build_dir/parser"
"$build_dir/parser"
"$cxx" -std=c++17 -O2 -I6.Tests/firmware/stubs -I"$f4/Robot" -I"$f4/Bsp/communication" \
  6.Tests/firmware/test_f4.cpp "$f4/Robot/instances/dummy_robot.cpp" \
  "$f4/Robot/actuators/ctrl_step/ctrl_step.cpp" "$f4/Robot/algorithms/kinematic/6dof_kinematic.cpp" \
  "$f4/Robot/algorithms/gravity_compensation/gravity_compensation.cpp" \
  "$f4/UserApp/protocols/can_protocol.cpp" "$f4/UserApp/protocols/ascii_protocol.cpp" \
  "$f4/Bsp/communication/ascii_processor.cpp" -o "$build_dir/f4"
"$build_dir/f4"
python3 6.Tests/firmware/test_can_irq.py
python3 6.Tests/firmware/test_f1_codegen.py
"$cxx" -std=c++17 -O2 -pthread -I6.Tests/firmware/stubs/usb -I6.Tests/firmware/stubs \
  -I"$f4/Robot" -I"$f4/Bsp/communication" \
  6.Tests/firmware/test_usb_pipeline.cpp "$f4/Bsp/communication/interface_usb.cpp" \
  "$f4/Robot/instances/dummy_robot.cpp" "$f4/Robot/actuators/ctrl_step/ctrl_step.cpp" \
  "$f4/Robot/algorithms/kinematic/6dof_kinematic.cpp" \
  "$f4/Robot/algorithms/gravity_compensation/gravity_compensation.cpp" \
  "$f4/UserApp/protocols/ascii_protocol.cpp" "$f4/Bsp/communication/ascii_processor.cpp" -o "$build_dir/usb"
"$build_dir/usb"
"${CC:-cc}" -std=gnu11 -O2 -Wall -Wextra -Werror -fno-pie -no-pie \
  -I6.Tests/firmware/stubs/heap 6.Tests/firmware/test_newlib_heap.c \
  "$f4/Bsp/memory/newlib_heap.c" \
  -Wl,--defsym=__HeapBase=0x20008000,--defsym=__HeapLimit=0x2001F800 -o "$build_dir/heap"
"$build_dir/heap"
