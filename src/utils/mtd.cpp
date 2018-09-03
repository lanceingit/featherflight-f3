#include <stdint.h>
#include <string.h>

#include "mtd.h"

MTD::MTD(SPI_FLASH* flash):
	_flash(flash),
	_write_addr(0),
	_read_addr(0),
	_status(MTD_IDLE),
	_has_erase(false),
	_full(false)
{

}

void MTD::init()
{
	Fifo_Create(&_write_fifo, _buf, BUF_SIZE);
}

void MTD::test()
{
	if(_flash->eraseSector(0))
//	while(1)
	{
		for(uint16_t i=0; i<_flash->getGeometry()->pageSize; i++)
		{
			_page_buf[i] = i;
		}

		if(_flash->pageProgram(0, _page_buf, _flash->getGeometry()->pageSize))
		{
			memset(_page_buf, 0, _flash->getGeometry()->pageSize);

			uint16_t read_len = _flash->readBytes(0, _page_buf, _flash->getGeometry()->pageSize);

			while(!_flash->eraseSector(_read_addr));
 
            memset(_page_buf, 0, _flash->getGeometry()->pageSize);
            _flash->readBytes(0, _page_buf, _flash->getGeometry()->pageSize);

		}
	}
}

void MTD::write(uint8_t* data, uint16_t len)
{
	if(get_space() > len)
	{
		for(uint16_t i=0; i<len; i++)
		{
			Fifo_Write(&_write_fifo, data[i]);
		}
		_full = false;
	}
	else
	{
		_full = true;
	}
}

uint16_t MTD::read(uint32_t offset, uint8_t* data, uint16_t len)
{
	uint16_t read_len = _flash->readBytes(offset, data, len);

	_read_addr += read_len;

	if(_read_addr == _write_addr) _read_addr = 0;

	return read_len;
}


void MTD::sync()
{
	if(!_has_erase && _write_addr % _flash->getGeometry()->sectorSize ==0)
	{
		_status = MTD_ERASE;
	}
	else if(Fifo_GetCount(&_write_fifo) > _flash->getGeometry()->pageSize)
	{
		if(_status != MTD_PROGRAM_CONTINUE)
		{
			_status = MTD_PROGRAM;
		}
	}


	if(_status == MTD_ERASE)
	{
		if(_flash->eraseSector(_write_addr))
		{
			_has_erase = true;
            _status = MTD_IDLE;
//			memset(_page_buf, 0, _flash->getGeometry()->pageSize);
//
//			uint16_t read_len = _flash->readBytes(0, _page_buf, _flash->getGeometry()->pageSize);

		}
	}
	else if(_status == MTD_PROGRAM)
	{
		for(uint16_t i=0; i<_flash->getGeometry()->pageSize; i++)
		{
			 Fifo_Read(&_write_fifo, &_page_buf[i]);
		}

		if(_flash->pageProgram(_write_addr, _page_buf, _flash->getGeometry()->pageSize))
		{
			_write_addr += _flash->getGeometry()->pageSize;
			_has_erase = false;
            _status = MTD_IDLE;
            
//			memset(_page_buf, 0x5C, _flash->getGeometry()->pageSize);

//			uint16_t read_len = _flash->readBytes(0, _page_buf, _flash->getGeometry()->pageSize);
            

		}
		else
		{
			_status = MTD_PROGRAM_CONTINUE;
		}
	}
	else if(_status == MTD_PROGRAM_CONTINUE)
	{
		if(_flash->pageProgram(_write_addr, _page_buf, _flash->getGeometry()->pageSize))
		{
			_write_addr += _flash->getGeometry()->pageSize;
			_has_erase = false;
            _status = MTD_IDLE;
		}
	}
}

uint32_t MTD::get_space()
{
	return _flash->getGeometry()->totalSize - _write_addr;
}

bool MTD::is_full()
{
	return _full;
}

