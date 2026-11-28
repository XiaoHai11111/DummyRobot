"""Cortex-M3 regression: constant-divisor integrators must not call software division.

Checks both the former problem configuration (-Og) and the control build (-O2).
This verifies instruction selection, not a board's cycle count.
"""
from pathlib import Path
import os,re,shutil,subprocess,tempfile
root=Path(__file__).resolve().parents[2]
for relative in ('Ctrl/Motor/motor.cpp','Ctrl/Motor/motion_planner.cpp'):
    hot_source=(root/'2.Firmware/Ctrl-Step-Driver-STM32F1-fw'/relative).read_text()
    assert 'estVelocity <<' not in hot_source, (relative, 'negative left shift in velocity path')
compiler=os.environ.get('ARM_CXX',shutil.which('arm-none-eabi-g++') or '')
nm=os.environ.get('ARM_NM',shutil.which('arm-none-eabi-nm') or '')
if not compiler or not nm:
    raise SystemExit('ARM compiler/nm required for the real-time code-generation check')
probe=r'''
#include "Motor/control_math.h"
extern "C" void pid(int32_t& i, int32_t& r, int64_t inc, int64_t pd, int32_t limit) {
    IntegrateLimited<1024>(i,r,inc,pd,limit);
}
extern "C" void dce(int32_t& i, int32_t& r, int64_t inc, int64_t pd, int32_t limit) {
    IntegrateLimited<128>(i,r,inc,pd,limit);
}
extern "C" int32_t localComp(int32_t current, int32_t velocity, int32_t assistQ16,
                             int32_t coulomb, int32_t deadzone, int32_t rampInvQ16,
                             int32_t limit, int32_t currentLimit) {
    return ApplyLocalCompensationQ16(current, velocity, assistQ16, coulomb,
                                     deadzone, rampInvQ16, limit, currentLimit);
}
extern "C" int32_t harmonicLocalComp(int32_t velocity, int32_t assistQ16,
                                     int32_t coulomb, int32_t deadzone,
                                     int32_t rampInvQ16, int32_t harmonic,
                                     int32_t blendQ16, int32_t limit) {
    return CalculateLocalCompensationQ16(velocity, assistQ16, coulomb, deadzone,
                                         rampInvQ16, harmonic, blendQ16, limit);
}
extern "C" uint32_t harmonicPhase(uint32_t relativePosition) {
    return (relativePosition * 256U) / 51200U;
}
extern "C" uint32_t harmonicPhase400(int32_t relativePosition) {
    relativePosition %= 51200;
    if (relativePosition < 0) relativePosition += 51200;
    return static_cast<uint32_t>(relativePosition) / 400U;
}
'''
with tempfile.TemporaryDirectory(prefix='dummy-f1-codegen-') as directory:
    source=Path(directory)/'probe.cpp'
    source.write_text(probe)
    for optimization in ('-Og','-O2'):
        obj=Path(directory)/(optimization[1:]+'.o')
        subprocess.run([compiler,'-std=c++17','-mcpu=cortex-m3','-mthumb',optimization,
                        '-I'+str(root/'2.Firmware/Ctrl-Step-Driver-STM32F1-fw/Ctrl'),
                        '-c',str(source),'-o',str(obj)],check=True)
        undefined=subprocess.check_output([nm,'-u',str(obj)],text=True)
        assert not re.search(r'__aeabi_.*div|__(?:u?div|u?mod)di3',undefined), undefined
print('F1 Cortex-M3 -Og/-O2 integral, harmonic phase and local compensation: no software division calls')
