/**
 * FSAE Library NVM
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2017-2018
 */
#include "FSAE_nvm.h"

#define _NVM_NUM_CONN 10

SPIConn nvmConnections[_NVM_NUM_CONN];
uint8_t nvmConnIdx = 0;

_NvmSuperblock superblock;

//=============================== INTERFACE ====================================

/**
 * Initializes the 25LC1024 NVM module for the standard FSAE template.
 */
SPIConn* init_nvm_std() {
  return init_nvm(_NVM_STD_BUS, _NVM_STD_CS_LATBITS, _NVM_STD_CS_LATNUM);
}

/**
 * Initializes the 25LC1024 NVM module with user-defined settings.
 */
SPIConn* init_nvm(uint8_t bus, uint32_t* cs_lat, uint8_t cs_num) {
  if (nvmConnIdx == _NVM_NUM_CONN) { return NULL; }

  // Initialize SPI communciations to the NVM chip
  if(!nvmConnIdx) {
    init_spi(bus, 10.0, 8, SPI_MODE0);
  }

  SPIConn* currConn = &nvmConnections[nvmConnIdx];
  currConn->send_fp = get_send_spi(bus);
  currConn->cs_lat = cs_lat;
  currConn->cs_num = cs_num;

  nvmConnIdx++;

  // Write protection info to status register
  _NvmStatusReg status = { .reg = 0x0 };
  status.WPEN = 0;
  status.BP1 = 0;
  status.BP0 = 0;
  _nvm_write_status_reg(currConn, status);

  // Check if superblock exists and is the correct version
  uint32_t superblock_vcode;
  nvm_read_data(currConn, 0x0, 4, &superblock_vcode);

  if (superblock_vcode == _NVM_SB_VER) {
    //TODO: _nvm_fsck()
    // Superblock exists and is correct version, cache in memory
    nvm_read_data(currConn, 0x0, sizeof(_NvmSuperblock), &superblock);
  } else {
    // Superblock either doesn't exist, or is an old version. Create a new
    // empty superblock with the correct version
    memset(&superblock, 0x0, sizeof(_NvmSuperblock));
    superblock.vcode = _NVM_SB_VER;
    nvm_write_data(currConn, 0x0, sizeof(_NvmSuperblock), &superblock);
  }

  return currConn;
}

/**
 * Checks the file system for an existing allocated block, or attempts to
 * create a new one. This is intended to be used to store structs of fixed
 * size in the NVM.
 *
 * First, this function checks to see if a block matching <block_id>/<vcode>
 * is present in the file system. If so, <fd> is dereferenced and set to the
 * index of the matching block.
 *
 * If no matching blocks were found, it will attempt to find an unused node.
 * If one is found, it will attempt to find enough contiguous pages of memory
 * to satisfy <size>.
 *
 * If everything was successful, the node at <fd> will contain metadata for the
 * newly allocated block and NVM_SUCCESS will be returned. Otherwise, the
 * appropriate error will be returned.
 */
uint8_t nvm_alloc(SPIConn* conn, uint8_t* block_id, uint32_t vcode,
                  uint32_t size, uint8_t* fd) {
  uint16_t i;

  // Search for matching node
  for (i = 0; i < 256; i++) {
    _NvmNode node = superblock.nodes[i];
    if (strncmp(block_id, node.block_id, 32) == 0 &&
        vcode == node.vcode && size == node.size) {
      *fd = i;
      return NVM_SUCCESS;
    }
  }

  // Search for available node
  for (i = 0; i < 256; i++) {
    //TODO: Verify this is ref and not copy (I've been doing too much c++)
    _NvmNode node = superblock.nodes[i];
    if (node.addr == 0x0) {
      strncpy(((uint8_t*) &(node.block_id)), block_id, 32);
      node.vcode = vcode;
      node.size = size;

      node.addr = 1; //TODO: Figure out address and mark pages as used

      // Write new superblock (TODO: Only write changed memory)
      nvm_write_data(conn, 0x0, sizeof(_NvmSuperblock), &superblock);

      *fd = i;
      return NVM_SUCCESS;
    }
  }

  return NVM_ERR_FULL;
}

/**
 * Reads bytes from the block specified by <fd> into <buf>.
 *
 * This function copies the entire block from the NVM chip to <buf>. The
 * destination buffer must be large enough to prevent overflow!
 *
 * Returns 0 on success.
 */
uint8_t nvm_read(SPIConn* conn, uint8_t fd, uint8_t* buf) {
  _NvmNode node = superblock.nodes[fd];
  return nvm_read_data(conn, node.addr, node.size, buf);
}

/**
 * Writes bytes from <buf> into the block specified by <fd>.
 *
 * This function copies enough bytes from <buf> to fill the entire block. The
 * source buffer must be large enough for the read!
 *
 * Returns 0 on success.
 */
uint8_t nvm_write(SPIConn* conn, uint8_t fd, uint8_t* buf) {
  _NvmNode node = superblock.nodes[fd];
  return nvm_write_data(conn, node.addr, node.size, buf);
}

/**
 * Reads <size> bytes from <addr> onwards into <buf>.
 *
 * The data is read sequentially starting from the given address until size
 * bytes have been read and copied. If the highest memory address is reached,
 * the read will roll over to address 0x0 and will continue.
 *
 * This function is non-blocking. If a write is in progress, it returns an
 * error. No reads can happen until there is no write in progress.
 */
uint8_t nvm_read_data(SPIConn* conn, uint32_t addr, uint32_t size, void* buf) {
  if (_nvm_wip(conn)) { return NVM_ERR_WIP; }
  if (addr > _NVM_MAX_ADDR) { return NVM_ERR_SEGV; }
  //TODO: Check addr not in superblock (round up page)

  spi_select(conn);
  conn->send_fp(_NVM_IN_READ);
  conn->send_fp((addr >> 16) & 0xFF);
  conn->send_fp((addr >> 8) & 0xFF);
  conn->send_fp(addr & 0xFF);

  uint32_t i;
  for (i = 0; i < size; i++) {
    ((uint8_t*) buf)[i] = conn->send_fp(0x00);
  }
  spi_deselect(conn);

  return NVM_SUCCESS;
}

/**
 * Writes <size> bytes from <buf> to <addr>.
 *
 * On the 25LC1024, writes must happen on a page-by-page basis. Each page is 256
 * bytes. Writes must not cross page boundaries. Thus, this generic write
 * function calls the page write function as many times as needed.
 *
 * This function is non-blocking. If a write is in progress, it returns an
 * error. No writes can happen until there is no write in progress.
 */
uint8_t nvm_write_data(SPIConn* conn, uint32_t addr, uint32_t size, void* buf) {
  if (_nvm_wip(conn)) { return NVM_ERR_WIP; }
  if (addr > _NVM_MAX_ADDR) { return NVM_ERR_SEGV; }
  if ((addr + size) > (_NVM_MAX_ADDR + 1)) { return NVM_ERR_SEGV; }
  //TODO: Check addr not in superblock (round up page)

  uint32_t done = 0;
  while (done < size) {
    while (_nvm_wip(conn)); // Wait for previous write cycle to finish

    uint32_t itr = addr + done;
    uint8_t* bitr = &(((uint8_t*) buf)[done]);
    uint32_t left = size - done;

    done += _nvm_write_page(conn, itr, left, bitr);
  }

  return NVM_SUCCESS;
}

//============================= IMPLEMENTATION =================================

/**
 * Helper function for _nvm_write_data() that writes data to a single page.
 *
 * Returns number of bytes written to the page.
 *
 * Do not use this function from outside of _nvm_write_data. It does not
 * perform necessary safety checks before starting the write.
 *
 * Writes as many bytes as possible (or <size> if less than what's possible) to
 * the memory location starting at <addr>. The write will not cross page
 * boundaries.
 */
uint8_t _nvm_write_page(SPIConn* conn, uint32_t addr, uint32_t size, uint8_t* buf) {
  uint8_t offset = ((uint8_t) (addr & 0xFF));
  uint8_t num = (uint8_t) min(256 - ((uint32_t) offset), size);

  _nvm_write_enable(conn);

  spi_select(conn);
  conn->send_fp(_NVM_IN_WRITE);
  conn->send_fp((addr >> 16) & 0xFF);
  conn->send_fp((addr >> 8) & 0xFF);
  conn->send_fp(addr & 0xFF);

  uint32_t i;
  for (i = 0; i < num; i++) {
    conn->send_fp(buf[i]);
  }
  spi_deselect(conn);

  return i;
}

/**
 * Sets the write enable latch
 */
void _nvm_write_enable(SPIConn* conn) {
  send_spi(_NVM_IN_WREN, conn);
}

/**
 * Returns whether or not there is currently a write in progress.
 */
uint8_t _nvm_wip(SPIConn* conn) {
  _NvmStatusReg status = _nvm_read_status_reg(conn);
  return status.WIP;
}

/**
 * Reads the contents of the status register.
 */
_NvmStatusReg _nvm_read_status_reg(SPIConn* conn) {
  _NvmStatusReg status = { .reg = _nvm_send_two(conn, _NVM_IN_RDSR, 0x00) };
  return status;
}

/**
 * Writes the contents of the status register
 */
void _nvm_write_status_reg(SPIConn* conn, _NvmStatusReg status) {
  _nvm_send_two(conn, _NVM_IN_WRSR, status.reg);
}

/**
 * Sends two bytes via SPI and returns the response from the NVM
 * chip. The response is the 8 bits clocked in during the transmission
 * of the second byte.
 */
uint8_t _nvm_send_two(SPIConn* conn, uint8_t one, uint8_t two) {
  uint8_t resp = 0;
  spi_select(conn);
  conn->send_fp(one);
  resp = (uint8_t) conn->send_fp(two);
  spi_deselect(conn);
  return resp;
}
