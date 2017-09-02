/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "microRTPS_transport.h"
#include "microRTPS_client.h"

#include <cinttypes>
#include <cstdio>
#include <ctime>
#include <pthread.h>

#include <microcdr/microCdr.h>
#include <px4_time.h>
#include <uORB/uORB.h>

#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_combined.h>

void* send(void *data);

void* send(void* /*unused*/)
{
    char data_buffer[BUFFER_SIZE] = {};
    uint64_t sent = 0, total_sent = 0;
    int loop = 0, read = 0;
    uint32_t length = 0;

    /* subscribe to topics */
    int fds[1] = {};

    // orb_set_interval statblish an update interval period in milliseconds.
    fds[0] = orb_subscribe(ORB_ID(sensor_baro));
    orb_set_interval(fds[0], _options.update_time_ms);

    // microBuffer to serialized using the user defined buffer
    struct microBuffer microBufferWriter;
    initStaticAlignedBuffer(data_buffer, BUFFER_SIZE, &microBufferWriter);
    // microCDR structs for managing the microBuffer
    struct microCDR microCDRWriter;
    initMicroCDR(&microCDRWriter, &microBufferWriter);

    struct timespec begin;
    px4_clock_gettime(CLOCK_REALTIME, &begin);

    while (!_should_exit_task)
    {
        bool updated;
        orb_check(fds[0], &updated);
        if (updated)
        {
            // obtained data for the file descriptor
            struct sensor_baro_s data;
            // copy raw data into local buffer
            if (orb_copy(ORB_ID(sensor_baro), fds[0], &data) == 0) {
                serialize_sensor_baro(&data, data_buffer, &length, &microCDRWriter);
                if (0 < (read = transport_node->write((char)57, data_buffer, length)))
                {
                    total_sent += read;
                    ++sent;
                }
            }
        }

        usleep(_options.sleep_ms*1000);
        ++loop;
    }

    struct timespec end;
    px4_clock_gettime(CLOCK_REALTIME, &end);
    double elapsed_secs = double(end.tv_sec - begin.tv_sec) + double(end.tv_nsec - begin.tv_nsec)/double(1000000000);
    printf("\nSENT:     %" PRIu64 " messages in %d LOOPS, %" PRIu64 " bytes in %.03f seconds - %.02fKB/s\n",
            sent, loop, total_sent, elapsed_secs, (double)total_sent/(1000*elapsed_secs));

    return nullptr;
}

static int launch_send_thread(pthread_t &sender_thread)
{
    pthread_attr_t sender_thread_attr;
    pthread_attr_init(&sender_thread_attr);
    pthread_attr_setstacksize(&sender_thread_attr, PX4_STACK_ADJUSTED(4000));
    struct sched_param param;
    (void)pthread_attr_getschedparam(&sender_thread_attr, &param);
    param.sched_priority = SCHED_PRIORITY_DEFAULT;
    (void)pthread_attr_setschedparam(&sender_thread_attr, &param);
    pthread_create(&sender_thread, &sender_thread_attr, send, nullptr);
    pthread_attr_destroy(&sender_thread_attr);

    return 0;
}

void micrortps_start_topics(struct timespec &begin, int &total_read, uint32_t &received, int &loop)
{

    char data_buffer[BUFFER_SIZE] = {};
    int read = 0;
    uint8_t topic_ID = 255;

    // Declare received topics
    struct sensor_combined_s sensor_combined_data;
    orb_advert_t sensor_combined_pub = nullptr;

    // microBuffer to deserialized using the user defined buffer
    struct microBuffer microBufferReader;
    initDeserializedAlignedBuffer(data_buffer, BUFFER_SIZE, &microBufferReader);
    // microCDR structs for managing the microBuffer
    struct microCDR microCDRReader;
    initMicroCDR(&microCDRReader, &microBufferReader);

    px4_clock_gettime(CLOCK_REALTIME, &begin);
    _should_exit_task = false;

    // create a thread for sending data to the simulator
    pthread_t sender_thread;
    launch_send_thread(sender_thread);

    while (!_should_exit_task)
    {
        while (0 < (read = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE)))
        {
            total_read += read;
            switch (topic_ID)
            {
                case 58:
                {
                    deserialize_sensor_combined(&sensor_combined_data, data_buffer, &microCDRReader);
                    if (!sensor_combined_pub) {
                        sensor_combined_pub = orb_advertise(ORB_ID(sensor_combined), &sensor_combined_data);
                    } else {
                        orb_publish(ORB_ID(sensor_combined), sensor_combined_pub, &sensor_combined_data);
                    }
                    ++received;
                }
                break;
                default:
                    printf("Unexpected topic ID\n");
                break;
            }
        }

        // loop forever if informed loop number is negative
        if (_options.loops >= 0 && loop >= _options.loops) break;

        usleep(_options.sleep_ms*1000);
        ++loop;
    }
    _should_exit_task = true;
    pthread_join(sender_thread, nullptr);
}
