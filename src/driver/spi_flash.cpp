#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "stm32f30x.h"

#include "spi.h"
#include "spi_flash.h"

#include "timer.h"

SPI_FLASH::SPI_FLASH():
    _couldBeBusy(false)
{
    _geometry.pageSize = M25P16_PAGESIZE;
    
    _spi = new SPI();
}


/**
 * Initialize the driver, must be called before any other routines.
 *
 * Attempts to detect a connected m25p16. If found, true is returned and device capacity can be fetched with
 * m25p16_getGeometry().
 */
bool SPI_FLASH::init()
{
    /* 
        if we have already detected a flash device we can simply exit 
        
        TODO: change the init param in favour of flash CFG when ParamGroups work is done
        then cs pin can be specified in hardware_revision.c or config.c (dependent on revision).
    */
    if (_geometry.sectors) {
        return true;
    }
    
    _spi->init();
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; // CS
    GPIO_Init(GPIOB, &GPIO_InitStructure);    

    DISABLE_M25P16;

    //Maximum speed for standard READ command is 20mHz, other commands tolerate 25mHz
    _spi->setDivisor(2);

    return readIdentification();
}

/**
 * Send the given command byte to the device.
 */
bool SPI_FLASH::performOneByteCommand(uint8_t command)
{
	bool ret;
    ENABLE_M25P16;

    ret = _spi->transferByte(NULL, command);

    DISABLE_M25P16;

    return ret;
}

/**
 * The flash requires this write enable command to be sent before commands that would cause
 * a write like program and erase.
 */
bool SPI_FLASH::writeEnable()
{
    if(performOneByteCommand(M25P16_INSTRUCTION_WRITE_ENABLE))
    {
        // Assume that we're about to do some writing, so the device is just about to become busy
        _couldBeBusy = true;
        return true;
    }
    else
    {
    	return false;
    }
}

uint8_t SPI_FLASH::readStatus()
{
    uint8_t command[2] = { M25P16_INSTRUCTION_READ_STATUS_REG, 0 };
    uint8_t in[2];

    ENABLE_M25P16;

    _spi->transfer(in, command, sizeof(command));

    DISABLE_M25P16;

    return in[1];
}

bool SPI_FLASH::isReady()
{
    // If couldBeBusy is false, don't bother to poll the flash chip for its status
    _couldBeBusy = _couldBeBusy && ((readStatus() & M25P16_STATUS_FLAG_WRITE_IN_PROGRESS) != 0);

    return !_couldBeBusy;
}

bool SPI_FLASH::waitForReady(uint32_t timeoutMillis)
{
    uint32_t time = Timer_getTime();
    while (!isReady()) {
        if (Timer_getTime() - time > timeoutMillis*1000) {
            return false;
        }
    }

    return true;
}

/**
 * Read chip identification and geometry information (into global `geometry`).
 *
 * Returns true if we get valid ident, false if something bad happened like there is no M25P16.
 */
bool SPI_FLASH::readIdentification()
{
    uint8_t out[] = { M25P16_INSTRUCTION_RDID, 0, 0, 0 };
    uint8_t in[4];
    uint32_t chipID;

    Timer_delayUs(50*1000); // short delay required after initialisation of SPI device instance.

    /* Just in case transfer fails and writes nothing, so we don't try to verify the ID against random garbage
     * from the stack:
     */
    in[1] = 0;

    ENABLE_M25P16;

    _spi->transfer(in, out, sizeof(out));

    // Clearing the CS bit terminates the command early so we don't have to read the chip UID:
    DISABLE_M25P16;

    // Manufacturer, memory type, and capacity
    chipID = (in[1] << 16) | (in[2] << 8) | (in[3]);

    // All supported chips use the same pagesize of 256 bytes

    switch (chipID) {
        case JEDEC_ID_MICRON_M25P16:
            _geometry.sectors = 32;
            _geometry.pagesPerSector = 256;
        break;
        case JEDEC_ID_MACRONIX_MX25L3206E:
            _geometry.sectors = 64;
            _geometry.pagesPerSector = 256;
        break;
        case JEDEC_ID_MICRON_N25Q064:
        case JEDEC_ID_WINBOND_W25Q64:
        case JEDEC_ID_MACRONIX_MX25L6406E:
            _geometry.sectors = 128*16;
            _geometry.pagesPerSector = 16;
        break;
        case JEDEC_ID_MICRON_N25Q128:
        case JEDEC_ID_WINBOND_W25Q128:
            _geometry.sectors = 256;
            _geometry.pagesPerSector = 256;
        break;
        default:
            // Unsupported chip or not an SPI NOR flash
            _geometry.sectors = 0;
            _geometry.pagesPerSector = 0;

            _geometry.sectorSize = 0;
            _geometry.totalSize = 0;
            return false;
    }

    _geometry.sectorSize = _geometry.pagesPerSector * _geometry.pageSize;
    _geometry.totalSize = _geometry.sectorSize * _geometry.sectors;

    _couldBeBusy = true; // Just for luck we'll assume the chip could be busy even though it isn't specced to be

    return true;
}



/**
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
bool SPI_FLASH::eraseSector(uint32_t address)
{
    uint8_t out[] = { M25P16_INSTRUCTION_SECTOR_ERASE, (uint8_t)((address >> 16) & 0xFF), (uint8_t)((address >> 8) & 0xFF), (uint8_t)(address & 0xFF)};
    bool ret;
    //waitForReady(SECTOR_ERASE_TIMEOUT_MILLIS);

    if(!isReady()) return false;


    if(!writeEnable()) return false;

    ENABLE_M25P16;

    ret = _spi->transfer(NULL, out, sizeof(out));

    DISABLE_M25P16;

    return ret;
}

bool SPI_FLASH::eraseCompletely()
{
    //waitForReady(BULK_ERASE_TIMEOUT_MILLIS);
	if(!isReady()) return false;

	if(!writeEnable()) return false;

    if(performOneByteCommand(M25P16_INSTRUCTION_BULK_ERASE)) return true;
    else return false;
}

bool SPI_FLASH::pageProgramBegin(uint32_t address)
{
    uint8_t command[] = { M25P16_INSTRUCTION_PAGE_PROGRAM, (uint8_t)((address >> 16) & 0xFF), (uint8_t)((address >> 8) & 0xFF), (uint8_t)(address & 0xFF)};

    //waitForReady(DEFAULT_TIMEOUT_MILLIS);
    if(!isReady()) return false;

    if(!writeEnable()) return false;

    ENABLE_M25P16;

    if(_spi->transfer(NULL, command, sizeof(command))) return true;
    else return false;
}

bool SPI_FLASH::pageProgramContinue(const uint8_t *data, int length)
{
    return _spi->transfer(NULL, data, length);
}

void SPI_FLASH::pageProgramFinish()
{
    DISABLE_M25P16;
}

/**
 * Write bytes to a flash page. Address must not cross a page boundary.
 *
 * Bits can only be set to zero, not from zero back to one again. In order to set bits to 1, use the erase command.
 *
 * Length must be smaller than the page size.
 *
 * This will wait for the flash to become ready before writing begins.
 *
 * Datasheet indicates typical programming time is 0.8ms for 256 bytes, 0.2ms for 64 bytes, 0.05ms for 16 bytes.
 * (Although the maximum possible write time is noted as 5ms).
 *
 * If you want to write multiple buffers (whose sum of sizes is still not more than the page size) then you can
 * break this operation up into one beginProgram call, one or more continueProgram calls, and one finishProgram call.
 */
bool SPI_FLASH::pageProgram(uint32_t address, const uint8_t *data, int length)
{
	bool ret;
    if(pageProgramBegin(address))
    {
        ret = pageProgramContinue(data, length);
    }
    else
    {
    	ret = false;
    }

    pageProgramFinish();

    return ret;
}

/**
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * Waits up to DEFAULT_TIMEOUT_MILLIS milliseconds for the flash to become ready before reading.
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */
int SPI_FLASH::readBytes(uint32_t address, uint8_t *buffer, int length)
{
    uint8_t command[] = { M25P16_INSTRUCTION_READ_BYTES, (uint8_t)((address >> 16) & 0xFF), (uint8_t)((address >> 8) & 0xFF), (uint8_t)(address & 0xFF)};

 //    if (!waitForReady(DEFAULT_TIMEOUT_MILLIS)) {
//        return 0;
//    }
    if(!isReady()) return 0;

    ENABLE_M25P16;

    if(_spi->transfer(NULL, command, sizeof(command)))
    {
        if(!_spi->transfer(buffer, NULL, length)) length = 0;
    }
    else
    {
    	length = 0;
    }

    DISABLE_M25P16;

    return length;
}

/**
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling m25p16_init() (the result would have totalSize = 0).
 */
const flashGeometry_t* SPI_FLASH::getGeometry()
{
    return &_geometry;
}

