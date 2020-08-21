MEMORY {
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K   
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 40K
  USBRAM (rw) : ORIGIN = 0x40006c00, LENGTH = 1K
}

SECTIONS {
  .usbram (NOLOAD) : {
    *(.usbram .usbram.*)
  } >USBRAM
} INSERT AFTER .bss;
