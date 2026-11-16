#include "instances/motion_command.h"
#include <cassert>
#include <iostream>
int main() {
    MotionCommand cmd;
    for (const char* text : {">0,-75,180,0,0,0", ">0,-75,180,0,0,0,30",
                            ">0,-75,180,0,0,0,30,45", ">0,-75,180,0,0,0,30,45,0.7",
                            "@220,0,300,0,90,0,30", "> 0, -75, 180, 0, 0, 0, 30 "})
        assert(cmd.Parse(text));
    for (const char* text : {"&0,-75,180,0,0,0,30", ">0,0,0", ">nan,0,0,0,0,0",
                            ">inf,0,0,0,0,0", ">1e999,0,0,0,0,0", ">0,0,0,0,0,0,30junk",
                            ">0,0,0,0,0,0,0", ">0,0,0,0,0,0,-1", ">0,0,0,0,0,0,101",
                            ">0,0,0,0,0,0,30,46", ">0,0,0,0,0,0,30,45,1.1",
                            ">0,0,0,0,0,0,30,45,1,2", "@0,0,0,0,0,0,30,45",
                            ">0,0,0,0,0,0,", ">0,0,0,0,0,0 trailing"})
        assert(!cmd.Parse(text));
    std::cout << "Motion parser validation tests passed\n";
}
