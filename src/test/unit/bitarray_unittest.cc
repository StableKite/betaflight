/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stddef.h>
#include <stdint.h>

extern "C" {
#include "common/bitarray.h"
}

#include "gtest/gtest.h"

TEST(BitArrayTest, HighBitsPreserveNeighboringWords)
{
    uint32_t words[] = {0x12345678, 0, 0, 0xabcdef01};
    uint32_t *bits = &words[1];

    EXPECT_FALSE(bitArrayGet(bits, 31));
    EXPECT_FALSE(bitArrayGet(bits, 63));
    bitArraySet(bits, 31);
    bitArraySet(bits, 63);
    EXPECT_TRUE(bitArrayGet(bits, 31));
    EXPECT_TRUE(bitArrayGet(bits, 63));
    EXPECT_EQ(0x80000000U, words[1]);
    EXPECT_EQ(0x80000000U, words[2]);

    bitArrayClr(bits, 31);
    EXPECT_FALSE(bitArrayGet(bits, 31));
    EXPECT_TRUE(bitArrayGet(bits, 63));
    bitArrayClr(bits, 63);
    EXPECT_EQ(0U, words[1]);
    EXPECT_EQ(0U, words[2]);
    EXPECT_EQ(0x12345678U, words[0]);
    EXPECT_EQ(0xabcdef01U, words[3]);
}

TEST(BitArrayTest, CopyAcrossWordBoundaryKeepsOtherBits)
{
    uint32_t bits[] = {0x80000001, 0x80000000};

    bitArrayCopy(bits, 31, 32);
    EXPECT_EQ(0x80000001U, bits[0]);
    EXPECT_EQ(0x80000001U, bits[1]);
    bitArrayClr(bits, 31);
    bitArrayCopy(bits, 31, 63);
    EXPECT_EQ(1U, bits[0]);
    EXPECT_EQ(1U, bits[1]);
}
