///////////////////////////////////////////////////////////////////////////////////
// SDRdaemon - send I/Q samples read from a SDR device over the network via UDP. //
//                                                                               //
// Copyright (C) 2015 Edouard Griffiths, F4EXB                                   //
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

#include <sys/time.h>
#include <iostream>
#include <lz4.h>

#include "UDPSink.h"

UDPSink::UDPSink(const std::string& address, unsigned int port, unsigned int udpSize) :
        m_address(address),
		m_port(port),
		m_udpSize(udpSize),
		m_centerFrequency(100000000),
		m_sampleRate(48000),
		m_sampleBytes(1),
		m_sampleBits(8),
		m_nbSamples(0),
		m_lz4MinInputSize(0),
		m_lz4BufSize(0),
		m_lz4Buffer(0)
{
	m_currentMeta.init();
	m_bufMeta = new uint8_t[m_udpSize];
	m_buf = new uint8_t[m_udpSize];
}

UDPSink::~UDPSink()
{
	delete[] m_buf;
	delete[] m_bufMeta;

	if (m_lz4Buffer) {
		delete[] m_lz4Buffer;
	}
}

void UDPSink::write(const IQSampleVector& samples_in)
{
	MetaData *metaData = (MetaData *) m_bufMeta;
    struct timeval tv;
    uint16_t samplesPerBlock = m_udpSize / (2 * m_sampleBytes);

    gettimeofday(&tv, 0);

    metaData->m_centerFrequency = m_centerFrequency;
    metaData->m_sampleRate = m_sampleRate;
    metaData->m_sampleBytes = m_sampleBytes;
    metaData->m_sampleBits = m_sampleBits;
    metaData->m_blockSize = m_udpSize;
    metaData->m_nbSamples = samples_in.size();
    metaData->m_nbBlocks = 1;
    metaData->m_remainderSamples = samples_in.size() % samplesPerBlock;
    metaData->m_nbCompleteBlocks = samples_in.size() / samplesPerBlock;
    metaData->m_tv_sec = tv.tv_sec;
    metaData->m_tv_usec = tv.tv_usec;
	metaData->m_crc = m_crc64.calculate_crc(m_bufMeta, sizeof(MetaData) - 8);

	if (metaData->m_nbSamples != m_nbSamples)
	{
		uint32_t bytesPerFrame = metaData->m_nbSamples * 2 * m_sampleBytes;

		if (m_lz4MinInputSize > 0)
		{
			uint32_t inputLZ4Size = ((m_lz4MinInputSize / bytesPerFrame) + 1)*bytesPerFrame;
			m_lz4BufSize = LZ4_compressBound(inputLZ4Size);
			delete[] m_lz4Buffer;
			m_lz4Buffer = new uint8_t[m_lz4BufSize];
		}

		m_nbSamples = metaData->m_nbSamples;

		std::cerr << "UDPSink::write: "
				<< m_nbSamples << " samples, "
				<< bytesPerFrame << " bytes per frame" << std::endl;
	}

	if (!(*metaData == m_currentMeta))
	{
		std::cerr << "UDPSink::write: meta: " << metaData->m_tv_sec
				<< ":" << metaData->m_tv_usec
				<< ":" << metaData->m_centerFrequency
				<< ":" << metaData->m_sampleRate
				<< ":" << (int) (metaData->m_sampleBytes & 0xF)
				<< ":" << (int) metaData->m_sampleBits
				<< ":" << metaData->m_blockSize
				<< ":" << metaData->m_nbSamples
				<< "|" << metaData->m_nbBlocks
				<< ":" << metaData->m_remainderSamples
				<< ":" << metaData->m_nbCompleteBlocks
				<< " samplesPerBlock: " << samplesPerBlock
				<< " bytesPerFrame: " << metaData->m_nbSamples * 2 * (metaData->m_sampleBytes & 0xF)
				<< std::endl;

		m_currentMeta = *metaData;
	}

	m_socket.SendDataGram((const void *) m_bufMeta, (int) m_udpSize, m_address, m_port);

	for (unsigned int i = 0; i < metaData->m_nbCompleteBlocks; i++)
	{
		m_socket.SendDataGram((const void *) &samples_in[i*samplesPerBlock], (int) m_udpSize, m_address, m_port);
	}

	if (metaData->m_remainderSamples > 0)
	{
		memcpy((void *) m_buf, (const void *) &samples_in[metaData->m_nbCompleteBlocks*samplesPerBlock], 2 * metaData->m_remainderSamples * (metaData->m_sampleBytes & 0xF));
		m_socket.SendDataGram((const void *) m_buf, (int) m_udpSize, m_address, m_port);
	}
}

void UDPSink::setLZ4MinInputSize(std::size_t lz4MinInputSize)
{
	if (!lz4MinInputSize)
	{
		if (m_lz4Buffer) {
			delete[] m_lz4Buffer;
		}

		m_lz4BufSize = 0;
	}

	m_lz4MinInputSize = lz4MinInputSize;
}
