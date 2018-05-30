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

#include <cstring>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <cstdlib>
#include <cerrno>

#include "AirspyHFSource.h"
#include "util.h"
#include "parsekv.h"

AirspyHFSource *AirspyHFSource::m_this = 0;

// Open AirspyHF device.
AirspyHFSource::AirspyHFSource(int dev_index) :
    m_dev(0),
    m_sampleRate(768000),
    m_frequency(10000000),
	m_ppm(0),
    m_hfAGC(false),
    m_hfATT(false),
    m_hfLNA(false),
    m_running(false),
    m_thread(0)
{
    uint64_t serials[AIRSPYHF_MAX_DEVICE];
    int count = airspyhf_list_devices(serials, AIRSPYHF_MAX_DEVICE);
    if (count > 0)
    {
        if (dev_index < count)
        {
            airspyhf_error rc = (airspyhf_error) airspyhf_open_sn(&m_dev, serials[dev_index]);

            if (rc != AIRSPYHF_SUCCESS)
            {
                std::ostringstream err_ostr;
                err_ostr << "Failed to open AirspyHF device at index " << dev_index;
                m_error = err_ostr.str();
                m_dev = 0;
            }
        }
        else
        {
            std::ostringstream err_ostr;
            err_ostr << "Failed to open AirspyHF device, index > count";
            m_error = err_ostr.str();
        }
    }
    else
    {
        std::ostringstream err_ostr;
        err_ostr << "Failed to list AirspyHF devices";
        m_error = err_ostr.str();
    }

    if (m_dev)
    {
        uint32_t nbSampleRates;
        uint32_t *sampleRates;

        airspyhf_get_samplerates(m_dev, &nbSampleRates, 0);

        sampleRates = new uint32_t[nbSampleRates];

        airspyhf_get_samplerates(m_dev, sampleRates, nbSampleRates);

        if (nbSampleRates == 0)
        {
            m_error = "Failed to get AirspyHF device sample rate list";
            airspyhf_close(m_dev);
            m_dev = 0;
        }
        else
        {
            for (uint32_t i=0; i<nbSampleRates; i++)
            {
                m_srates.push_back(sampleRates[i]);
            }
        }

        delete[] sampleRates;

        std::ostringstream srates_ostr;

        for (int s: m_srates) {
            srates_ostr << s << " ";
        }

        m_sratesStr = srates_ostr.str();
    }

    m_this = this;
}

AirspyHFSource::~AirspyHFSource()
{
    if (m_dev) {
        airspyhf_close(m_dev);
    }

    m_this = 0;
}

void AirspyHFSource::get_device_names(std::vector<std::string>& devices)
{
    uint64_t serials[AIRSPYHF_MAX_DEVICE];
    int count = airspyhf_list_devices(serials, AIRSPYHF_MAX_DEVICE);
    if (count < 0)
    {
        std::cerr << "AirspyHFSource::get_device_names: failed to enumerate devices" << std::endl;
        return;
    }

    for (int i=0; i < count; i++)
    {
        std::ostringstream devname_ostr;
        devname_ostr << "Serial " << std::hex << std::setw(16) << std::setfill('0') << serials[i];
        devices.push_back(devname_ostr.str());
    }
}

std::uint32_t AirspyHFSource::get_sample_rate()
{
    return m_sampleRate;
}

std::uint32_t AirspyHFSource::get_frequency()
{
    return m_frequency;
}

void AirspyHFSource::print_specific_parms()
{
	fprintf(stderr, "LO correction:     %.1f\n", m_ppm);
    /*
    fprintf(stderr, "HF AGC:            %s\n", m_hfAGC ? "enabled" : "disabled");
    fprintf(stderr, "HF attenuator:     %s\n", m_hfATT ? "enabled" : "disabled");
    fprintf(stderr, "HF LNA:            %s\n", m_hfLNA ? "enabled" : "disabled");
    */
}

bool AirspyHFSource::configure(std::uint32_t changeFlags,
        int sampleRateIndex,
        uint32_t frequency,
        int hf_agc,
        int hf_att,
        int hf_lna
)
{
    airspyhf_error rc;

    if (!m_dev) {
        return false;
    }

    if (changeFlags & 0x1)
    {
        m_frequency = frequency;

        rc = (airspyhf_error) airspyhf_set_freq(m_dev, m_frequency);

        if (rc != AIRSPYHF_SUCCESS)
        {
            std::ostringstream err_ostr;
            err_ostr << "Could not set center frequency to " << m_frequency << " Hz";
            m_error = err_ostr.str();
            std::cerr << "AirspyHFSource::configure(flags): " << m_error << std::endl;
            return false;
        }
        else
        {
            std::cerr << "AirspyHFSource::configure(flags): center frequency set to " << m_frequency << " Hz" << std::endl;;
        }
    }

    if (changeFlags & 0x2)
    {
        rc = (airspyhf_error) airspyhf_set_samplerate(m_dev, m_srates[sampleRateIndex]);

        if (rc != AIRSPYHF_SUCCESS)
        {
            std::ostringstream err_ostr;
            err_ostr << "Could not set center sample rate to " << m_srates[sampleRateIndex] << " Hz";
            m_error = err_ostr.str();
            std::cerr << "AirspyHFSource::configure(flags): " << m_error << std::endl;
            return false;
        }
        else
        {
            m_sampleRate = m_srates[sampleRateIndex];
            std::cerr << "AirspyHFSource::configure(flags): sample rate set to " << m_sampleRate << " S/s" << std::endl;;
        }
    }
/*
    if (changeFlags & 0x20)
    {
        m_hfAGC = hf_agc;

        rc = (airspyhf_error) airspyhf_set_hf_agc(m_dev, (m_hfAGC ? 1 : 0));

        if (rc != AIRSPYHF_SUCCESS)
        {
            std::ostringstream err_ostr;
            err_ostr << "Could not set HF AGC to " << m_hfAGC;
            m_error = err_ostr.str();
            std::cerr << "AirspyHFSource::configure(flags): " << m_error << std::endl;
            return false;
        }
        else
        {
            std::cerr << "AirspyHFSource::configure(flags): HF AGC set to " << m_hfAGC << std::endl;;
        }
    }

    if (changeFlags & 0x40)
    {
        m_hfATT = hf_att;

        rc = (airspyhf_error) airspyhf_set_hf_att(m_dev, (m_hfATT ? 1 : 0));

        if (rc != AIRSPYHF_SUCCESS)
        {
            std::ostringstream err_ostr;
            err_ostr << "Could not set HF attenuator to " << m_hfATT;
            m_error = err_ostr.str();
            std::cerr << "AirspyHFSource::configure(flags): " << m_error << std::endl;
            return false;
        }
        else
        {
            std::cerr << "AirspyHFSource::configure(flags): HF attenuator set to " << m_hfATT << std::endl;;
        }
    }

    if (changeFlags & 0x80)
    {
        m_hfLNA = hf_lna;

        rc = (airspyhf_error) airspyhf_set_hf_lna(m_dev, (m_hfLNA ? 1 : 0));

        if (rc != AIRSPYHF_SUCCESS)
        {
            std::ostringstream err_ostr;
            err_ostr << "Could not set HF LNA to " << m_hfLNA;
            m_error = err_ostr.str();
            std::cerr << "AirspyHFSource::configure(flags): " << m_error << std::endl;
            return false;
        }
        else
        {
            std::cerr << "AirspyHFSource::configure(flags): HF LNA set to " << m_hfLNA << std::endl;;
        }
    }
*/
    return true;
}

bool AirspyHFSource::configure(parsekv::pairs_type& m)
{
    int sampleRateIndex = 0;
    uint32_t frequency = m_confFreq;
    float ppm = m_ppm;
    int hfAGC = 0;
    int hfATT = 0;
    int hfLNA = 0;
    std::uint32_t changeFlags = 0;

    if (m.find("freq") != m.end())
	{
		std::cerr << "AirspyHFSource::configure: freq: " << m["freq"] << std::endl;
		frequency = atoi(m["freq"].c_str());

		if ((frequency < 24000000) || (frequency > 1800000000))
		{
			m_error = "Invalid frequency";
            std::cerr << "AirspyHFSource::configure: " << m_error << std::endl;
			return false;
		}

        changeFlags |= 0x1;
	}

	if (m.find("srate") != m.end())
	{
		std::cerr << "AirspyHFSource::configure: srate: " << m["srate"] << std::endl;

		if (strcasecmp(m["srate"].c_str(), "list") == 0)
		{
			m_error = "Available sample rates (Hz): " + m_sratesStr;
            std::cerr << "AirspyHFSource::configure: " << m_error << std::endl;
			return false;
		}

		m_sampleRate = atoi(m["srate"].c_str());
		uint32_t i;

		for (i = 0; i < m_srates.size(); i++)
		{
			if (m_srates[i] == static_cast<int>(m_sampleRate))
			{
				sampleRateIndex = i;
				break;
			}
		}

		if (i == m_srates.size())
		{
			m_error = "Invalid sample rate";
			m_sampleRate = 0;
            std::cerr << "AirspyHFSource::configure: " << m_error << std::endl;
			return false;
		}

        changeFlags |= 0x2;
	}
/*
	if (m.find("hfagc") != m.end())
	{
		hfAGC = atoi(m["hfagc"].c_str());
        std::cerr << "AirspyHFSource::configure: hfagc " << (hfAGC == 1 ? "on" : "off") << std::endl;
        changeFlags |= 0x20;
	}

	if (m.find("hfatt") != m.end())
	{
		hfATT = atoi(m["hfatt"].c_str());
        std::cerr << "AirspyHFSource::configure: hfatt " << (hfATT == 1 ? "on" : "off") << std::endl;
        changeFlags |= 0x40;
	}

	if (m.find("hflna") != m.end())
	{
		hfLNA = atoi(m["hflna"].c_str());
        std::cerr << "AirspyHFSource::configure: hflna " << (hfLNA == 1 ? "on" : "off") << std::endl;
        changeFlags |= 0x80;
	}
*/
    if (m.find("ppmp") != m.end())
	{
		std::cerr << "AirspyHFSource::configure: ppmp: " << m["ppmp"] << std::endl;
		errno = 0;
		char * e;
		ppm = std::strtod(m["ppmp"].c_str(), &e);

		if (*e == '\0' && errno == 0) // Conversion to float OK
		{
			m_ppm = ppm;
			changeFlags |= 0x1;
		}
	}
    else if (m.find("ppmn") != m.end())
	{
		std::cerr << "AirspyHFSource::configure: ppmn: " << m["ppmn"] << std::endl;
		errno = 0;
		char * e;
		ppm = std::strtod(m["ppmn"].c_str(), &e);

		if (*e == '\0' && errno == 0) // Conversion to float OK
		{
			m_ppm = -ppm;
			changeFlags |= 0x1;
		}
	}

	if (m.find("decim") != m.end())
	{
		std::cerr << "AirspyRFSource::configure: decim: " << m["decim"] << std::endl;
		int log2Decim = atoi(m["decim"].c_str());

		if ((log2Decim < 0) || (log2Decim > 6))
		{
			m_error = "Invalid log2 decimation factor";
            std::cerr << "AirspyHFSource::configure: " << m_error << std::endl;
			return false;
		}
		else
		{
			m_decim = log2Decim;
		}
	}

	m_confFreq = frequency;
	double tuner_freq;

	tuner_freq = frequency;
	tuner_freq += tuner_freq * m_ppm * 1e-6;

    return configure(changeFlags, sampleRateIndex, tuner_freq, hfAGC, hfATT, hfLNA);
}

bool AirspyHFSource::start(DataBuffer<IQSample> *buf, std::atomic_bool *stop_flag)
{
    m_buf = buf;
    m_stop_flag = stop_flag;

    if (m_thread == 0)
    {
        std::cerr << "AirspyHFSource::start: starting" << std::endl;
        m_running = true;
        m_thread = new std::thread(run, m_dev, stop_flag);
        sleep(1);
        return *this;
    }
    else
    {
        std::cerr << "AirspyHFSource::start: error" << std::endl;
        m_error = "Source thread already started";
        return false;
    }
}

void AirspyHFSource::run(airspyhf_device* dev, std::atomic_bool *stop_flag)
{
    std::cerr << "AirspyHFSource::run" << std::endl;
    void *msgBuf = 0;

    airspyhf_error rc = (airspyhf_error) airspyhf_start(dev, rx_callback, 0);

    if (rc == AIRSPYHF_SUCCESS)
    {
        while (!stop_flag->load() && (airspyhf_is_streaming(dev) == true))
        {
            sleep(1);

            int len = nn_recv(m_this->m_nnReceiver, &msgBuf, NN_MSG, NN_DONTWAIT);

            if ((len > 0) && msgBuf)
            {
                std::string msg((char *) msgBuf, len);
                std::cerr << "AirspyHFSource::run: received: " << msg << std::endl;
                m_this->DeviceSource::configure(msg);
                nn_freemsg(msgBuf);
                msgBuf = 0;
            }
        }

        rc = (airspyhf_error) airspyhf_stop(dev);

        if (rc != AIRSPYHF_SUCCESS)
        {
            std::cerr << "AirspyHFSource::run: Cannot stop AirspyHF Rx" << std::endl;
        }
    }
    else
    {
        std::cerr << "AirspyHFSource::run: Cannot start AirspyHF Rx" << std::endl;
    }
}

bool AirspyHFSource::stop()
{
    std::cerr << "AirspyHFSource::stop" << std::endl;

    m_thread->join();
    delete m_thread;
    return true;
}

int AirspyHFSource::rx_callback(airspyhf_transfer_t* transfer)
{
    int len = transfer->sample_count * 2; // interleaved I/Q samples

    if (m_this)
    {
        m_this->callback((short *) transfer->samples, len);
    }

    return 0;
}

void AirspyHFSource::callback(const short* buf, int len)
{
    IQSampleVector iqsamples;

    iqsamples.resize(len/2);

    for (int i = 0, j = 0; i < len; i+=2, j++)
    {
        int16_t re = buf[i];
        int16_t im = buf[i+1];
        iqsamples[j] = IQSample(re, im);
    }

    m_buf->push(move(iqsamples));
}
