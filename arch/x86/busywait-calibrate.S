.global busywait_doing_calibration
.global busywait_calibrate
.type busywait_calibrate,%function
busywait_calibrate:
    xor %rax,%rax
    movq $0x1,%rdi
1:
    inc %rax                              // increment
    cmpl busywait_doing_calibration,%edi  // L1 cache access
    jz 1b                                 // if we're still doing it, keep incrementing
    ret
.global busywait_1us
.type busywait_1us,%function
busywait_1us:
    xor %rax,%rax
1:
    inc %rax                              // increment
    cmpq busywait_doing_calibration,%rax  // L1 cache access
    jl 1b                                 // if we're still doing it, keep incrementing
    ret
