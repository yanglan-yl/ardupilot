#include "AP_RangeFinder_Benewake_TFA1500.h"
#if AP_RANGEFINDER_BENEWAKE_TFA1500_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <ctype.h>

extern const AP_HAL::HAL &hal;
#define TFA1500_FRAME_HEADER 0x5C
#define TFA1500_FRAME_LENGTH 5
#define TFA1500_DIST_MAX_CM 130000                                     
static uint8_t TFA1500_CMD_START[] = {0x55, 0xAA, 0xCB, 0xCC, 0xCC, 0xCC, 0xCC, 0xFB}; 
static uint8_t TFA1500_CMD_STOP[] = {0x55, 0xAA, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xFC};  

void AP_RangeFinder_Benewake_TFA1500::init_serial(uint8_t serial_instance)
{
    AP_RangeFinder_Backend_Serial::init_serial(serial_instance);
    // give the sensor time to start up
    hal.scheduler->delay(100);
    hal.console->printf("TFA1500 sensor started \n");
    if (uart == nullptr)
    {
        return ;
    }
    uart->write(TFA1500_CMD_START, sizeof(TFA1500_CMD_START));
    hal.console->printf("TFA1500 sensor started ok\n");
}
AP_RangeFinder_Benewake_TFA1500::~AP_RangeFinder_Benewake_TFA1500()
{
    if (uart != nullptr)
    {
        uart->write(TFA1500_CMD_STOP, sizeof(TFA1500_CMD_STOP));
        hal.console->printf("TFA1500 sensor stopped\n");
    }
}

bool AP_RangeFinder_Benewake_TFA1500::get_reading(float &reading_m)
{
    if (uart == nullptr)
    {
        return false;
    }
    float sum_cm = 0;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;
    tf_linebuf_len = 0;
    // read any available lines from the lidar
    for (auto j = 0; j < 8192; j++)
    {
        uint8_t c;
        if (!uart->read(c))
        {
            break;
        }
        if (tf_linebuf_len == 0)
        {
            if (c == TFA1500_FRAME_HEADER)
            {
                tf_linebuf[tf_linebuf_len++] = c;
            }
        }
        else
        {
            // add character to buffer
            tf_linebuf[tf_linebuf_len++] = c;
            if (tf_linebuf_len == TFA1500_FRAME_LENGTH)
            {
                // calculate checksum
                uint8_t checksum = 0;
                for (uint8_t i = 1; i < TFA1500_FRAME_LENGTH - 1; i++)
                {
                    checksum += tf_linebuf[i];
                }
                checksum = ~checksum;
                if (checksum == tf_linebuf[TFA1500_FRAME_LENGTH - 1])
                {
                    // calculate distance
                    uint32_t dist = (tf_linebuf[3] << 16) | (tf_linebuf[2] << 8) | tf_linebuf[1];
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
                tf_linebuf_len = 0;
            }
        }
    }

    if (count > 0)
    {
        // return average distance of readings
        reading_m = (sum_cm * 0.01f) / count;
        return true;
    }

    if (count_out_of_range > 0)
    {
        // if only out of range readings return larger of
        // driver defined maximum range for the model and user defined max range + 1m
        reading_m = MAX(model_dist_max_cm() * 0.01, max_distance());
        return true;
    }
    // no readings so return false
    return false;
}

#endif