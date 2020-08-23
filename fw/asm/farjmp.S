.syntax unified

.text
.global farjmp
.thumb_func
farjmp:
    msr MSP, r0
    bx r1
