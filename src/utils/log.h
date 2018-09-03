#pragma once

#include "log_messages.h"
#include "att_est_q.h"
#include "sensors.h"
#include "mtd.h"



class LOG
{
public:
    LOG(MTD* mtd);

    void init();
    void write(void* pkt, uint16_t len);
    uint16_t read(uint32_t offset, uint8_t* data, uint16_t len);

    void write_att(Att_Est_Q &att, uint16_t rate);
    void write_imu(SENSORS &sens, uint16_t rate);
    void write_sens(SENSORS &sens, uint16_t rate);

    uint32_t get_size() {return _mtd->get_store();}

    void stop() {_record = false;}
    bool need_record() {return _record;}


private:
    MTD* _mtd;
    uint64_t _timer[log_formats_num];
    bool _record;
};
