/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

#include <math.h>
#include <string.h>

extern "C" {
    #include "common/filter.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(FilterUnittest, TestPt1FilterInit)
{
    pt1Filter_t filter;
    pt1FilterInit(&filter, 0.0f);
    EXPECT_EQ(0, filter.k);

    pt1FilterInit(&filter, 1.0f);
    EXPECT_EQ(1.0, filter.k);
}

TEST(FilterUnittest, TestPt1FilterGain)
{
    EXPECT_FLOAT_EQ(0.999949, pt1FilterGain(100.0f, 31.25f));
    // handle cases over uint8_t boundary
    EXPECT_FLOAT_EQ(0.99998301, pt1FilterGain(300.0f, 31.25f));
}

TEST(FilterUnittest, TestPt1FilterApply)
{
    pt1Filter_t filter;
    pt1FilterInit(&filter, pt1FilterGain(100.0f, 31.25f));
    EXPECT_EQ(0, filter.state);

    pt1FilterApply(&filter, 1800.0f);
    EXPECT_FLOAT_EQ(1799.9083, filter.state);

    pt1FilterApply(&filter, -1800.0f);
    EXPECT_FLOAT_EQ(-1799.8165, filter.state);

    pt1FilterApply(&filter, -200.0f);
    EXPECT_FLOAT_EQ(-200.08142, filter.state);
}

TEST(FilterUnittest, TestSlewFilterInit)
{
    slewFilter_t filter;

    slewFilterInit(&filter, 0.0f, 0.0f);
    EXPECT_EQ(0, filter.state);
    EXPECT_EQ(0, filter.slewLimit);
    EXPECT_EQ(0, filter.threshold);

    slewFilterInit(&filter, 1800.0f, 1900.0f);
    EXPECT_EQ(0, filter.state);
    EXPECT_EQ(1800, filter.slewLimit);
    EXPECT_EQ(1900, filter.threshold);
}

TEST(FilterUnittest, TestSlewFilter)
{
    slewFilter_t filter;
    slewFilterInit(&filter, 2000.0f, 1900.0f);
    EXPECT_EQ(0, filter.state);
    EXPECT_EQ(2000, filter.slewLimit);
    EXPECT_EQ(1900, filter.threshold);

    slewFilterApply(&filter, 1800.0f);
    EXPECT_EQ(1800, filter.state);
    slewFilterApply(&filter, -1800.0f);
    EXPECT_EQ(-1800, filter.state);
    slewFilterApply(&filter, -200.0f);
    EXPECT_EQ(-200, filter.state);

    slewFilterApply(&filter, 1900.0f);
    EXPECT_EQ(1900, filter.state);
    slewFilterApply(&filter, -2000.0f);
    EXPECT_EQ(1900, filter.state);
    slewFilterApply(&filter, -200.0f);
    EXPECT_EQ(1900, filter.state);
    slewFilterApply(&filter, 1800.0f);
    EXPECT_EQ(1800, filter.state);
    slewFilterApply(&filter, -200.0f);
    EXPECT_EQ(-200, filter.state);

    slewFilterApply(&filter, -1900.0f);
    EXPECT_EQ(-1900, filter.state);
    slewFilterApply(&filter, 2000.0f);
    EXPECT_EQ(-1900, filter.state);
    slewFilterApply(&filter, 200.0f);
    EXPECT_EQ(-1900, filter.state);
    slewFilterApply(&filter, -1800.0f);
    EXPECT_EQ(-1800, filter.state);
    slewFilterApply(&filter, 200.0f);
    EXPECT_EQ(200, filter.state);
}

// The callback signature is erased, but its state updates and floating-point
// results must stay bit-for-bit identical to the typed filter entry point.
template <typename Filter>
static void expectIdenticalFilterDispatch(Filter initial, float (*typed)(Filter *, float), filterApplyFnPtr callback)
{
    Filter direct = initial;
    Filter generic = initial;
    for (unsigned i = 0; i < 256; i++) {
        float input = (int)((i * 37) % 257) - 128;
        input *= (i % 31 == 0) ? 0.5f : 0.03125f;
        const float expected = typed(&direct, input);
        const float actual = callback(reinterpret_cast<filter_t *>(&generic), input);
        uint32_t expectedBits, actualBits;
        memcpy(&expectedBits, &expected, sizeof(expectedBits));
        memcpy(&actualBits, &actual, sizeof(actualBits));
        ASSERT_EQ(expectedBits, actualBits) << "sample " << i;
    }
}

TEST(FilterUnittest, Pt1CallbackPreservesTypedResults)
{
    pt1Filter_t state = {};
    pt1FilterInit(&state, 0.23f);
    expectIdenticalFilterDispatch(state, pt1FilterApply, pt1FilterApplyFn);
}

TEST(FilterUnittest, Pt2CallbackPreservesTypedResults)
{
    pt2Filter_t state = {};
    pt2FilterInit(&state, 0.23f);
    expectIdenticalFilterDispatch(state, pt2FilterApply, pt2FilterApplyFn);
}

TEST(FilterUnittest, Pt3CallbackPreservesTypedResults)
{
    pt3Filter_t state = {};
    pt3FilterInit(&state, 0.23f);
    expectIdenticalFilterDispatch(state, pt3FilterApply, pt3FilterApplyFn);
}

TEST(FilterUnittest, SvfLowpassCallbackPreservesTypedResults)
{
    svfLowpassFilter_t state = {};
    svfLowpassFilterInit(&state, 123.0f, 0.000125f);
    expectIdenticalFilterDispatch(state, svfLowpassFilterApply, svfLowpassFilterApplyFn);
}

TEST(FilterUnittest, SvfNotchCallbackPreservesTypedResults)
{
    svfNotchFilter_t state = {};
    svfNotchInit(&state, 180.0f, 0.000125f, 1.3f);
    expectIdenticalFilterDispatch(state, svfNotchApply, svfNotchApplyFn);
}
