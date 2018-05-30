///////////////////////////////////////////////////////////////////////////////////
// SDRdaemon - send I/Q samples read from a SDR device over the network via UDP. //
//                                                                               //
// Copyright (C) 2015 Edouard Griffiths, F4EXB                                   //
// Copyright (C) 2018 Jared Boone, AF7SO                                         //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_AIRSPYHFDEVICESOURCE_H_
#define INCLUDE_AIRSPYHFDEVICESOURCE_H_

#include <cstdint>
#include <string>
#include <vector>
#include "libairspyhf/airspyhf.h"

#include "DeviceSource.h"

#define AIRSPYHF_MAX_DEVICE (32)

class AirspyHFSource : public DeviceSource
{
public:

    /** Open AirspyHF device. */
    AirspyHFSource(int dev_index);

    /** Close AirspyHF device. */
    virtual ~AirspyHFSource();

    /** Return sample size in bits */
    virtual std::uint32_t get_sample_bits() { return 12; }

    /** Return current sample frequency in Hz. */
    virtual std::uint32_t get_sample_rate();

    /** Return device current center frequency in Hz. */
    virtual std::uint32_t get_frequency();

    /** Print current parameters specific to device type */
    virtual void print_specific_parms();

    virtual bool start(DataBuffer<IQSample> *buf, std::atomic_bool *stop_flag);
    virtual bool stop();

    /** Return true if the device is OK, return false if there is an error. */
    virtual operator bool() const
    {
        return m_dev && m_error.empty();
    }

    /** Return a list of supported devices. */
    static void get_device_names(std::vector<std::string>& devices);

private:

    /** Configure AirspyHF tuner from a list of key=values */
    virtual bool configure(parsekv::pairs_type& m);

    /**
     * Configure AirspyHF tuner and prepare for streaming.
     *
     * changeFlags     :: horrible hack to notify which fields have changed
     * sampleRateIndex :: desired sample rate index in the sample rates enumeration list.
     * frequency       :: desired center frequency in Hz.
     * hf_agc          :: HF mode AGC (1: on, 0: off)
     * hf_att          :: HF mode attenuator (1: on, 0: off)
     * hf_lna          :: HF mode LNA (1: on, 0: off)
     *
     * Return true for success, false if an error occurred.
     */
    bool configure(std::uint32_t changeFlags,
                   int sampleRateIndex,
                   uint32_t frequency,
                   int hf_agc,
                   int hf_att,
                   int hf_lna
    );

    void callback(const short* buf, int len);
    static int rx_callback(airspyhf_transfer_t* transfer);
    static void run(airspyhf_device* dev, std::atomic_bool *stop_flag);

    struct airspyhf_device* m_dev;
    uint32_t m_sampleRate;
    uint32_t m_frequency;
    float m_ppm;
    bool m_hfAGC;
    bool m_hfATT;
    bool m_hfLNA;
    bool m_running;
    std::thread *m_thread;
    static AirspyHFSource *m_this;
    std::vector<int> m_srates;
    std::string m_sratesStr;
};

#endif /* INCLUDE_AIRSPYHFDEVICESOURCE_H_ */
