#include "AP_RangeFinder_Benewake_TFA1500.h"
#if AP_RANGEFINDER_BENEWAKE_TFA1500_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <ctype.h>

extern const AP_HAL::HAL &hal;
#define TFA1500_FRAME_HEADER 0x5C
#define TFA1500_FRAME_LENGTH 5
#define TFA1500_DIST_MAX_CM 130000                                                     // todo:具体数值待确认
static uint8_t TFA1500_CMD_START[] = {0x55, 0xAA, 0xCB, 0xCC, 0xCC, 0xCC, 0xCC, 0xFB}; // 启动命令
static uint8_t TFA1500_CMD_STOP[] = {0x55, 0xAA, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xFC};  // 停止命令

bool AP_RangeFinder_Benewake_TFA1500::get_reading(float &reading_m)
{
    if (uart == nullptr)
    {
        return false;
    }
    uart->write(TFA1500_CMD_START, sizeof(TFA1500_CMD_START));
    hal.console->printf("read start\n");
    float sum_cm = 0;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;
    TF_linebuf_len = 0;
    // read any available lines from the lidar
    for (auto j = 0; j < 8192; j++)
    {
        uint8_t c;
        if (!uart->read(c))
        {
            break;
        }
        if (TF_linebuf_len == 0)
        {
            if (c == TFA1500_FRAME_HEADER)
            {
                TF_linebuf[TF_linebuf_len++] = c;
            }
        }
        else
        {
            // add character to buffer
            TF_linebuf[TF_linebuf_len++] = c;
            if (TF_linebuf_len == TFA1500_FRAME_LENGTH)
            {
                // calculate checksum
                uint8_t checksum = 0;
                // 计算校验和,校验字节数已确认
                for (uint8_t i = 1; i < TFA1500_FRAME_LENGTH - 1; i++)
                {
                    checksum += TF_linebuf[i];
                }
                checksum = ~checksum;
                if (checksum == TF_linebuf[TFA1500_FRAME_LENGTH - 1])
                {
                    // calculate distance
                    uint32_t dist = (TF_linebuf[3] << 16) | (TF_linebuf[2] << 8) | TF_linebuf[1];
                    //  hal.console->printf("read dist%dcm ",dist);
                    if (dist >= TFA1500_DIST_MAX_CM || dist == uint32_t(model_dist_max_cm()))
                    {
                        count_out_of_range++;
                    }
                    else
                    {
                        // add distance to sum
                        sum_cm += dist;
                        count++;
                    }
                }
                // clear buffer
                TF_linebuf_len = 0;
            }
        }
    }

    if (count > 0)
    {
        // return average distance of readings
        reading_m = (sum_cm * 0.01f) / count;
        hal.console->printf("read :%fm\n", reading_m);
        return true;
    }

    if (count_out_of_range > 0)
    {
        // if only out of range readings return larger of
        // driver defined maximum range for the model and user defined max range + 1m
        reading_m = MAX(model_dist_max_cm() * 0.01, max_distance());
        hal.console->printf("outof range%fm\n", reading_m);
        return true;
    }
    uart->write(TFA1500_CMD_STOP, sizeof(TFA1500_CMD_STOP));
    // no readings so return false
    return false;
}

#endif