#pragma once

#include "spi_flash.h"
#include "fifo.h"

enum mtd_status
{
	MTD_ERASE,
	MTD_PROGRAM,
	MTD_PROGRAM_CONTINUE,
	MTD_IDLE,
};


class MTD
{
public:
	MTD(SPI_FLASH* flash);

    void init();
    void test();
	void write(uint8_t* data, uint16_t len);
	uint16_t read(uint32_t offset, uint8_t* data, uint16_t len);
	void sync();
	uint32_t get_space();
	bool is_full();
	uint32_t get_store() {return _write_addr;}

private:
	SPI_FLASH* _flash;

	#define BUF_SIZE 4096

	Fifo _write_fifo;
	uint8_t _buf[BUF_SIZE];
	uint8_t _page_buf[M25P16_PAGESIZE];

	uint32_t _write_addr;
	uint32_t _read_addr;

	enum mtd_status _status;

	bool _has_erase;
	bool _full;

};

