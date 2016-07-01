/**
 * FSAE Library NVM
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2015-2016
 */
#include "FSAE_nvm.h"

/**
 * void nvm_write_word(void* addr, uint32_t data)
 *
 * Writes one word of data to the NVM memory located at addr
 *
 * @param addr Address to write the word to
 * @param data The word of data to write
 */
void nvm_write_word(void* addr, uint32_t data) {
  // Set up Address and Data Registers
  NVMADDR = KVA_TO_PA(addr);
  NVMDATA0 = data;

  // Enable flash for write operation and set the NVMOP
  NVMCONCLR = _NVMCON_WREN_MASK;
  NVMCONbits.NVMOP = 0x1; // NVMOP for word programming
  NVMCONSET = _NVMCON_WREN_MASK;

  nvm_init_operation();
}

/**
 * void read_nvm_data(void* buffer, uint32_t count)
 *
 * Copies count bytes from the user data area in non-volatile memory to the
 * given buffer address.
 *
 * @param buffer Address of the buffer to copy data to
 * @param count Number of bytes to copy
 */
void read_nvm_data(void* buffer, uint32_t count) {
  memcpy(buffer, PA_TO_KVA1(USER_NVM_PHYS_ADDR), count);
}

/**
 * void write_nvm_data(void* data, uint32_t count)
 *
 * Writes count bytes from the given buffer address to the user data area in
 * non-volatile memory. Starts at the beginning of the user NVM area.
 *
 * @param buffer Address of the data buffer to copy from
 * @param count Number of bytes to copy
 */
void write_nvm_data(void* buffer, uint32_t count) {
  nvm_erase_page(); // Page must be erased before a write can take place

  uint32_t iter;
  for(iter = 0; iter < count; iter += 4) {
    void* addr = PA_TO_KVA1(USER_NVM_PHYS_ADDR + iter);
    uint32_t word = ((uint32_t*) buffer)[iter / 4];
    nvm_write_word(addr, word);
  }
}

/**
 * void nvm_erase_page(void)
 *
 * Erases the page used for NVM so that the cells can be written to again.
 */
void nvm_erase_page(void) {
  NVMADDR = USER_NVM_PHYS_ADDR; // Set base page address
  NVMCONbits.NVMOP = 0x4; // Set to page erase
  NVMCONSET = _NVMCON_WREN_MASK;
  nvm_init_operation();
}

/**
 * void nvm_init_operation(void);
 *
 * Start the flash memory programming/erase operation.
 */
void nvm_init_operation(void) {
  CLI(); // Disable Interrupts

  // Unlock sequence
  NVMKEY = 0x0;
  NVMKEY = 0xAA996655;
  NVMKEY = 0x556699AA;
  NVMCONSET = _NVMCON_WR_MASK;

  STI(); // Enable interrupts

  // Wait for WR bit to clear
  while(NVMCON & _NVMCON_WR_MASK);

  // Disable future flash write/erase operations
  NVMCONCLR = _NVMCON_WREN_MASK;
}
