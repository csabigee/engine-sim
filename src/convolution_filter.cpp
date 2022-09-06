#include "../include/convolution_filter.h"

#include <algorithm>
#include <assert.h>
#include <string.h>

ConvolutionFilter::ConvolutionFilter() {
    m_shiftRegister = nullptr;
    m_impulseResponse = nullptr;

    m_shiftOffset = 0;
    m_sampleCount = 0;
}

ConvolutionFilter::~ConvolutionFilter() {
    assert(m_shiftRegister == nullptr);
    assert(m_impulseResponse == nullptr);
}

void ConvolutionFilter::initialize(int samples) {
    m_sampleCount = samples;
    m_shiftOffset = 0;
    m_shiftRegister = new float[samples];
    m_impulseResponse = new float[samples];

    memset(m_shiftRegister, 0, sizeof(float) * samples);
    memset(m_impulseResponse, 0, sizeof(float) * samples);
}

void ConvolutionFilter::destroy() {
    delete[] m_shiftRegister;
    delete[] m_impulseResponse;

    m_shiftRegister = nullptr;
    m_impulseResponse = nullptr;
}

float ConvolutionFilter::f(float sample) {
    m_shiftRegister[m_shiftOffset] = sample;

#if 0 // 70fps -> 100fps in WASM builds
    float result = 0;
    for (int i = 0; i < m_sampleCount - m_shiftOffset; ++i) {
        result += m_impulseResponse[i] * m_shiftRegister[i + m_shiftOffset];
    }

    for (int i = m_sampleCount - m_shiftOffset; i < m_sampleCount; ++i) {
        result += m_impulseResponse[i] * m_shiftRegister[i - (m_sampleCount - m_shiftOffset)];
    }
#else
    const int offset = m_shiftOffset;
    const int count = m_sampleCount;
    const int size = count - offset;
    const float *__restrict__ const impulseResponse = m_impulseResponse;
    const float *__restrict__ const shiftRegister = m_shiftRegister;

    // Split across multiple accumulators to reduce data dependencies per loop
    constexpr int accumulators = 4;
    float acc0 = 0;
    float acc1 = 0;
    float acc2 = 0;
    float acc3 = 0;
    float residual = 0;
    int i = 0;

    // How much inner unwinding to do
    constexpr int unwindFactor = 2;
    constexpr unsigned loop = accumulators * unwindFactor;
    constexpr unsigned loopMask = ~(loop - 1);
    static_assert((loop & ~loopMask) == 0);

    // Unroll as many as we can
    while (i < (size & loopMask)) {
        for (int j = 0; j < unwindFactor; j++) {
            acc0 += impulseResponse[i] * shiftRegister[i + offset]; i++;
            acc1 += impulseResponse[i] * shiftRegister[i + offset]; i++;
            acc2 += impulseResponse[i] * shiftRegister[i + offset]; i++;
            acc3 += impulseResponse[i] * shiftRegister[i + offset]; i++;
        }
    }
    // Residuals that wouldn't fit in a full unroll
    while (i < size) {
        residual += impulseResponse[i] * shiftRegister[i + offset]; i++;
    }

    // Residuals to read until next alignment
    const int nextUnroll = std::min<int>((size + ~loopMask) & loopMask, count);
    while (i < nextUnroll) {
        residual += impulseResponse[i] * shiftRegister[i - size]; i++;
    }
    // Unroll as many as we can
    while (i < (count & loopMask)) {
        for (int j = 0; j < unwindFactor; j++) {
            acc0 += impulseResponse[i] * shiftRegister[i - size]; i++;
            acc1 += impulseResponse[i] * shiftRegister[i - size]; i++;
            acc2 += impulseResponse[i] * shiftRegister[i - size]; i++;
            acc3 += impulseResponse[i] * shiftRegister[i - size]; i++;
        }
    }
    // Residuals that wouldn't fit in a full unroll
    while (i < count) {
        residual += impulseResponse[i] * shiftRegister[i - size]; i++;
    }

    // Sum the accumulators
    const float result = acc0 + acc1 + acc2 + acc3 + residual;
#endif

    m_shiftOffset = (m_shiftOffset - 1 + m_sampleCount) % m_sampleCount;

    return result;
}
