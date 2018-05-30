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

#include "AirspySource.h"
#include "util.h"
#include "parsekv.h"

AirspySource *AirspySource::m_this = 0;
const std::vector<int> AirspySource::m_lgains({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14});
const std::vector<int> AirspySource::m_mgains({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15});
const std::vector<int> AirspySource::m_vgains({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15});

// Open Airspy device.
AirspySource::AirspySource(int dev_index) :
    m_dev(0),
    m_sampleRate(10000000),
    m_frequency(100000000),
	m_ppm(0),
    m_lnaGain(8),
    m_mixGain(8),
    m_vgaGain(0),
    m_biasAnt(false),
    m_lnaAGC(false),
    m_mixAGC(false),
    m_running(false),
    m_thread(0)
{
    airspy_error rc = (airspy_error) airspy_init();

    if (rc != AIRSPY_SUCCESS)
    {
        std::ostringstream err_ostr;
        err_ostr << "Failed to open Airspy library (" << rc << ": " << airspy_error_name(rc) << ")";
        m_error = err_ostr.str();
        m_dev = 0;
    }
    else
    {
        for (int i = 0; i < AIRSPY_MAX_DEVICE; i++)
        {
            rc = (airspy_error) airspy_open(&m_dev);

            if (rc == AIRSPY_SUCCESS)
            {
                if (i == dev_index)
                {
                    break;
                }
            }
            else
            {
                std::ostringstream err_ostr;
                err_ostr << "Failed to open Airspy device at sequence " << i;
                m_error = err_ostr.str();
                m_dev = 0;
            }
        }
    }

    if (m_dev)
    {
        uint32_t nbSampleRates;
        uint32_t *sampleRates;

        airspy_get_samplerates(m_dev, &nbSampleRates, 0);

        sampleRates = new uint32_t[nbSampleRates];

        airspy_get_samplerates(m_dev, sampleRates, nbSampleRates);

        if (nbSampleRates == 0)
        {
            m_error = "Failed to get Airspy device sample rate list";
            airspy_close(m_dev);
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

        rc = (airspy_error) airspy_set_sample_type(m_dev, AIRSPY_SAMPLE_INT16_IQ);

        if (rc != AIRSPY_SUCCESS)
        {
            m_error = "AirspyInput::start: could not set sample type to INT16_IQ";
        }
    }

    std::ostringstream lgains_ostr;

    for (int g: m_lgains) {
        lgains_ostr << g << " ";
    }

    m_lgainsStr = lgains_ostr.str();

    std::ostringstream mgains_ostr;

    for (int g: m_mgains) {
        mgains_ostr << g << " ";
    }

    m_mgainsStr = mgains_ostr.str();

    std::ostringstream vgains_ostr;

    for (int g: m_vgains) {
        vgains_ostr << g << " ";
    }

    m_vgainsStr = vgains_ostr.str();

    m_this = this;
}

AirspySource::~AirspySource()
{
    if (m_dev) {
        airspy_close(m_dev);
    }

    airspy_error rc = (airspy_error) airspy_exit();
    std::cerr << "AirspySource::~AirspySource: Airspy library exit: " << rc << ": " << airspy_error_name(rc) << std::endl;

    m_this = 0;
}

void AirspySource::get_device_names(std::vector<std::string>& devices)
{
    airspy_device *airspy_ptr;
    airspy_read_partid_serialno_t read_partid_serialno;
    uint32_t serial_msb = 0;
    uint32_t serial_lsb = 0;
    airspy_error rc;
    int i;

    rc = (airspy_error) airspy_init();

    if (rc != AIRSPY_SUCCESS)
    {
        std::cerr << "AirspySource::get_device_names: Failed to open Airspy library: " << rc << ": " << airspy_error_name(rc) << std::endl;
        return;
    }

    for (i=0; i < AIRSPY_MAX_DEVICE; i++)
    {
        rc = (airspy_error) airspy_open(&airspy_ptr);
        std::cerr << "AirspySource::get_device_names: try to get device " << i << " serial number" << std::endl;

        if (rc == AIRSPY_SUCCESS)
        {
            std::cerr << "AirspySource::get_device_names: device " << i << " open OK" << std::endl;

            rc = (airspy_error) airspy_board_partid_serialno_read(airspy_ptr, &read_partid_serialno);

            if (rc == AIRSPY_SUCCESS)
            {
                serial_msb = read_partid_serialno.serial_no[2];
                serial_lsb = read_partid_serialno.serial_no[3];
                std::ostringstream devname_ostr;
                devname_ostr << "Serial " << std::hex << std::setw(8) << std::setfill('0') << serial_msb << serial_lsb;
                devices.push_back(devname_ostr.str());
            }
            else
            {
                std::cerr << "AirspySource::get_device_names: failed to get device " << i << " serial number: " << rc << ": " << airspy_error_name(rc) << std::endl;
            }

            airspy_close(airspy_ptr);
        }
        else
        {
            std::cerr << "AirspySource::get_device_names: enumerated " << i << " Airspy devices: " << airspy_error_name(rc) << std::endl;
            break; // finished
        }
    }

    rc = (airspy_error) airspy_exit();
    std::cerr << "AirspySource::get_device_names: Airspy library exit: " << rc << ": " << airspy_error_name(rc) << std::endl;
}

std::uint32_t AirspySource::get_sample_rate()
{
    return m_sampleRate;
}

std::uint32_t AirspySource::get_frequency()
{
    return m_frequency;
}

void AirspySource::print_specific_parms()
{
	fprintf(stderr, "LO correction:     %.1f\n", m_ppm);
    fprintf(stderr, "LNA gain:          %d\n", m_lnaGain);
    fprintf(stderr, "Mixer gain:        %d\n", m_mixGain);
    fprintf(stderr, "VGA gain:          %d\n", m_vgaGain);
    fprintf(stderr, "Antenna bias       %s\n", m_biasAnt ? "enabled" : "disabled");
    fprintf(stderr, "LNA AGC            %s\n", m_lnaAGC ? "enabled" : "disabled");
    fprintf(stderr, "Mixer AGC          %s\n", m_mixAGC ? "enabled" : "disabled");
}

bool AirspySource::configure(std::uint32_t changeFlags,
        int sampleRateIndex,
        uint32_t frequency,
        int bias_ant,
        int lna_gain,
        int mix_gain,
        int vga_gain,
        int lna_agc,
        int mix_agc
)
{
    airspy_error rc;

    if (!m_dev) {
        return false;
    }

    if (changeFlags & 0x1)
    {
        m_frequency = frequency;

        rc = (airspy_error) airspy_set_freq(m_dev, m_frequency);

        if (rc != AIRSPY_SUCCESS)
        {
            std::ostringstream err_ostr;
            err_ostr << "Could not set center frequency to " << m_frequency << " Hz";
            m_error = err_ostr.str();
            std::cerr << "AirspySource::configure(flags): " << m_error << std::endl;
            return false;
        }
        else
        {
            std::cerr << "AirspySource::configure(flags): center frequency set to " << m_frequency << " Hz" << std::endl;;
        }
    }

    if (changeFlags & 0x2)
    {
        rc = (airspy_error) airspy_set_samplerate(m_dev, static_cast<airspy_samplerate_t>(sampleRateIndex));

        if (rc != AIRSPY_SUCCESS)
        {
            std::ostringstream err_ostr;
            err_ostr << "Could not set center sample rate to " << m_srates[sampleRateIndex] << " Hz";
            m_error = err_ostr.str();
            std::cerr << "AirspySource::configure(flags): " << m_error << std::endl;
            return false;
        }
        else
        {
            m_sampleRate = m_srates[sampleRateIndex];
            std::cerr << "AirspySource::configure(flags): sample rate set to " << m_sampleRate << " S/s" << std::endl;;
        }
    }

    if (changeFlags & 0x4)
    {
        m_lnaGain = lna_gain;

        rc = (airspy_error) airspy_set_lna_gain(m_dev, m_lnaGain);

        if (rc != AIRSPY_SUCCESS)
        {
            std::ostringstream err_ostr;
            err_ostr << "Could not set LNA gain to " << m_lnaGain << " dB";
            m_error = err_ostr.str();
            std::cerr << "AirspySource::configure(flags): " << m_error << std::endl;
            return false;
        }
        else
        {
            std::cerr << "AirspySource::configure(flags): LNA gain set to " << m_lnaGain << " dB" << std::endl;;
        }
    }

    if (changeFlags & 0x8)
    {
        m_mixGain = mix_gain;

        rc = (airspy_error) airspy_set_mixer_gain(m_dev, m_mixGain);

        if (rc != AIRSPY_SUCCESS)
        {
            std::ostringstream err_ostr;
            err_ostr << "Could not set mixer gain to " << m_mixGain << " dB";
            m_error = err_ostr.str();
            std::cerr << "AirspySource::configure(flags): " << m_error << std::endl;
            return false;
        }
        else
        {
            std::cerr << "AirspySource::configure(flags): mixer gain set to " << m_mixGain << " dB" << std::endl;;
        }
    }

    if (changeFlags & 0x10)
    {
        m_vgaGain = vga_gain;

        rc = (airspy_error) airspy_set_vga_gain(m_dev, m_vgaGain);

        if (rc != AIRSPY_SUCCESS)
        {
            std::ostringstream err_ostr;
            err_ostr << "Could not set VGA gain to " << m_vgaGain << " dB";
            m_error = err_ostr.str();
            std::cerr << "AirspySource::configure(flags): " << m_error << std::endl;
            return false;
        }
        else
        {
            std::cerr << "AirspySource::configure(flags): VGA gain set to " << m_vgaGain << " dB" << std::endl;;
        }
    }

    if (changeFlags & 0x20)
    {
        m_biasAnt = bias_ant;

        rc = (airspy_error) airspy_set_rf_bias(m_dev, (m_biasAnt ? 1 : 0));

        if (rc != AIRSPY_SUCCESS)
        {
            std::ostringstream err_ostr;
            err_ostr << "Could not set bias antenna to " << m_biasAnt;
            m_error = err_ostr.str();
            std::cerr << "AirspySource::configure(flags): " << m_error << std::endl;
            return false;
        }
        else
        {
            std::cerr << "AirspySource::configure(flags): Bias antenna set to " << m_biasAnt << std::endl;;
        }
    }

    if (changeFlags & 0x40)
    {
        m_lnaAGC = lna_agc;

        rc = (airspy_error) airspy_set_lna_agc(m_dev, (m_lnaAGC ? 1 : 0));

        if (rc != AIRSPY_SUCCESS)
        {
            std::ostringstream err_ostr;
            err_ostr << "Could not set LNA AGC to " << m_lnaAGC;
            m_error = err_ostr.str();
            std::cerr << "AirspySource::configure(flags): " << m_error << std::endl;
            return false;
        }
        else
        {
            std::cerr << "AirspySource::configure(flags): LNA AGC set to " << m_lnaAGC << std::endl;;
        }
    }

    if (changeFlags & 0x80)
    {
        m_mixAGC = mix_agc;

        rc = (airspy_error) airspy_set_mixer_agc(m_dev, (m_mixAGC ? 1 : 0));

        if (rc != AIRSPY_SUCCESS)
        {
            std::ostringstream err_ostr;
            err_ostr << "Could not set mixer AGC to " << m_mixAGC;
            m_error = err_ostr.str();
            std::cerr << "AirspySource::configure(flags): " << m_error << std::endl;
            return false;
        }
        else
        {
            std::cerr << "AirspySource::configure(flags): mixer AGC set to " << m_mixAGC << std::endl;;
        }
    }

    return true;
}

bool AirspySource::configure(parsekv::pairs_type& m)
{
    int sampleRateIndex = 0;
    uint32_t frequency = m_confFreq;
    float ppm = m_ppm;
    int lnaGain = 8;
    int mixGain = 8;
    int vgaGain = 0;
    int antBias = 0;
    int lnaAGC = 0;
    int mixAGC = 0;
    int fcpos = 2; // default centered
    std::uint32_t changeFlags = 0;

    if (m.find("freq") != m.end())
	{
		std::cerr << "AirspySource::configure: freq: " << m["freq"] << std::endl;
		frequency = atoi(m["freq"].c_str());

		if ((frequency < 24000000) || (frequency > 1800000000))
		{
			m_error = "Invalid frequency";
            std::cerr << "AirspySource::configure: " << m_error << std::endl;
			return false;
		}

        changeFlags |= 0x1;
	}

	if (m.find("srate") != m.end())
	{
		std::cerr << "AirspySource::configure: srate: " << m["srate"] << std::endl;

		if (strcasecmp(m["srate"].c_str(), "list") == 0)
		{
			m_error = "Available sample rates (Hz): " + m_sratesStr;
            std::cerr << "AirspySource::configure: " << m_error << std::endl;
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
            std::cerr << "AirspySource::configure: " << m_error << std::endl;
			return false;
		}

        changeFlags |= 0x2;

        if (fcpos != 2)
        {
            changeFlags |= 0x1; // need to adjust actual center frequency if not centered
        }
	}

	if (m.find("lgain") != m.end())
	{
		std::cerr << "AirspySource::configure: lgain: " << m["lgain"] << std::endl;

		if (strcasecmp(m["lgain"].c_str(), "list") == 0)
		{
			m_error = "Available LNA gains (dB): " + m_lgainsStr;
            std::cerr << "AirspySource::configure: " << m_error << std::endl;
			return false;
		}

		lnaGain = atoi(m["lgain"].c_str());

		if (find(m_lgains.begin(), m_lgains.end(), lnaGain) == m_lgains.end())
		{
			m_error = "LNA gain not supported. Available gains (dB): " + m_lgainsStr;
            std::cerr << "AirspySource::configure: " << m_error << std::endl;
			return false;
		}

        changeFlags |= 0x4;
	}

	if (m.find("mgain") != m.end())
	{
		std::cerr << "AirspySource::configure: mgain: " << m["mgain"] << std::endl;

		if (strcasecmp(m["mgain"].c_str(), "list") == 0)
		{
			m_error = "Available mixer gains (dB): " + m_mgainsStr;
            std::cerr << "AirspySource::configure: " << m_error << std::endl;
			return false;
		}

		mixGain = atoi(m["mgain"].c_str());

		if (find(m_mgains.begin(), m_mgains.end(), mixGain) == m_mgains.end())
		{
			m_error = "Mixer gain not supported. Available gains (dB): " + m_mgainsStr;
            std::cerr << "AirspySource::configure: " << m_error << std::endl;
			return false;
		}

        changeFlags |= 0x8;
	}

	if (m.find("vgain") != m.end())
	{
		std::cerr << "AirspySource::configure: vgain: " << m["vgain"] << std::endl;
		vgaGain = atoi(m["vgain"].c_str());

		if (strcasecmp(m["vgain"].c_str(), "list") == 0)
		{
			m_error = "Available VGA gains (dB): " + m_vgainsStr;
            std::cerr << "AirspySource::configure: " << m_error << std::endl;
			return false;
		}

		if (find(m_vgains.begin(), m_vgains.end(), vgaGain) == m_vgains.end())
		{
			m_error = "VGA gain not supported. Available gains (dB): " + m_vgainsStr;
            std::cerr << "AirspySource::configure: " << m_error << std::endl;
			return false;
		}

        changeFlags |= 0x10;
	}

	if (m.find("antbias") != m.end())
	{
		antBias = atoi(m["antbias"].c_str());
        std::cerr << "AirspySource::configure: antbias " << (antBias == 1 ? "on" : "off") << std::endl;
        changeFlags |= 0x20;
	}

	if (m.find("lagc") != m.end())
	{
		lnaAGC = atoi(m["lagc"].c_str());
        std::cerr << "AirspySource::configure: lagc " << (lnaAGC == 1 ? "on" : "off") << std::endl;
        changeFlags |= 0x40;
	}

	if (m.find("magc") != m.end())
	{
		mixAGC = atoi(m["magc"].c_str());
        std::cerr << "AirspySource::configure: magc " << (mixAGC == 1 ? "on" : "off") << std::endl;
        changeFlags |= 0x80;
	}

    if (m.find("ppmp") != m.end())
	{
		std::cerr << "AirspySource::configure: ppmp: " << m["ppmp"] << std::endl;
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
		std::cerr << "AirspySource::configure: ppmn: " << m["ppmn"] << std::endl;
		errno = 0;
		char * e;
		ppm = std::strtod(m["ppmn"].c_str(), &e);

		if (*e == '\0' && errno == 0) // Conversion to float OK
		{
			m_ppm = -ppm;
			changeFlags |= 0x1;
		}
	}

	if (m.find("fcpos") != m.end())
	{
		std::cerr << "AirspySource::configure: fcpos: " << m["fcpos"] << std::endl;
		fcpos = atoi(m["fcpos"].c_str());

		if ((fcpos < 0) || (fcpos > 2))
		{
			m_error = "Invalid center frequency position";
            std::cerr << "AirspySource::configure: " << m_error << std::endl;
			return false;
		}
		else
		{
			m_fcPos = fcpos;
		}

        changeFlags |= 0x1; // need to adjust actual center frequency if not centered
	}

	if (m.find("decim") != m.end())
	{
		std::cerr << "AirspySource::configure: decim: " << m["decim"] << std::endl;
		int log2Decim = atoi(m["decim"].c_str());

		if ((log2Decim < 0) || (log2Decim > 6))
		{
			m_error = "Invalid log2 decimation factor";
            std::cerr << "AirspySource::configure: " << m_error << std::endl;
			return false;
		}
		else
		{
			m_decim = log2Decim;
		}
	}

	m_confFreq = frequency;
	double tuner_freq;

	if (m_fcPos == 0) { // Infradyne
		tuner_freq = frequency + 0.25 * m_srates[sampleRateIndex];
	} else if (m_fcPos == 1) { // Supradyne
		tuner_freq = frequency - 0.25 * m_srates[sampleRateIndex];
	} else { // Centered
		tuner_freq = frequency;
	}

	tuner_freq += tuner_freq * m_ppm * 1e-6;

    return configure(changeFlags, sampleRateIndex, tuner_freq, antBias, lnaGain, mixGain, vgaGain, lnaAGC, mixAGC);
}

bool AirspySource::start(DataBuffer<IQSample> *buf, std::atomic_bool *stop_flag)
{
    m_buf = buf;
    m_stop_flag = stop_flag;

    if (m_thread == 0)
    {
        std::cerr << "AirspySource::start: starting" << std::endl;
        m_running = true;
        m_thread = new std::thread(run, m_dev, stop_flag);
        sleep(1);
        return *this;
    }
    else
    {
        std::cerr << "AirspySource::start: error" << std::endl;
        m_error = "Source thread already started";
        return false;
    }
}

void AirspySource::run(airspy_device* dev, std::atomic_bool *stop_flag)
{
    std::cerr << "AirspySource::run" << std::endl;
    void *msgBuf = 0;

    airspy_error rc = (airspy_error) airspy_start_rx(dev, rx_callback, 0);

    if (rc == AIRSPY_SUCCESS)
    {
        while (!stop_flag->load() && (airspy_is_streaming(dev) == AIRSPY_TRUE))
        {
            sleep(1);

            int len = nn_recv(m_this->m_nnReceiver, &msgBuf, NN_MSG, NN_DONTWAIT);

            if ((len > 0) && msgBuf)
            {
                std::string msg((char *) msgBuf, len);
                std::cerr << "AirspySource::run: received: " << msg << std::endl;
                m_this->DeviceSource::configure(msg);
                nn_freemsg(msgBuf);
                msgBuf = 0;
            }
        }

        rc = (airspy_error) airspy_stop_rx(dev);

        if (rc != AIRSPY_SUCCESS)
        {
            std::cerr << "AirspySource::run: Cannot stop Airspy Rx: " << rc << ": " << airspy_error_name(rc) << std::endl;
        }
    }
    else
    {
        std::cerr << "AirspySource::run: Cannot start Airspy Rx: " << rc << ": " << airspy_error_name(rc) << std::endl;
    }
}

bool AirspySource::stop()
{
    std::cerr << "AirspySource::stop" << std::endl;

    m_thread->join();
    delete m_thread;
    return true;
}

int AirspySource::rx_callback(airspy_transfer_t* transfer)
{
    int len = transfer->sample_count * 2; // interleaved I/Q samples

    if (m_this)
    {
        m_this->callback((short *) transfer->samples, len);
    }

    return 0;
}

void AirspySource::callback(const short* buf, int len)
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
