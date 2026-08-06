/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2022, Basler AG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * No contributors' name may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace pylon_ros2_camera
{

/**
 * One entry of the gain/exposure cycle. A value is only meant to be written to
 * the camera when its 'has_' flag is set, so a caller can cycle gain alone,
 * exposure alone, or both.
 */
struct SequenceStep
{
    double gain{0.0};
    double exposure{0.0};
    bool has_gain{false};
    bool has_exposure{false};
};

/**
 * Walks the cartesian product of a gain list and an exposure list, one entry
 * per frame, so consecutive frames of the same scene are captured at different
 * brightness levels. Gain varies fastest, exposure slowest, and a full cycle
 * visits every pair exactly once.
 *
 * Either list may be empty (that parameter is then left alone) or hold a single
 * entry (that parameter is then constant). Deliberately free of any pylon or
 * ROS dependency so it can be unit tested on its own.
 */
class ParameterSequencer
{
public:
    ParameterSequencer() = default;

    ParameterSequencer(std::vector<double> gains, std::vector<double> exposures)
        : gains_(std::move(gains))
        , exposures_(std::move(exposures))
    {}

    /**
     * True when neither parameter is cycled and the sequencer has nothing to apply.
     */
    bool empty() const
    {
        return this->gains_.empty() && this->exposures_.empty();
    }

    /**
     * Number of frames in one full cycle.
     */
    std::size_t size() const
    {
        if (this->empty())
        {
            return 0;
        }

        return this->gainCount() * this->exposureCount();
    }

    /**
     * Entry to apply before grabbing frame number 'step'. The step is taken
     * modulo the cycle length, so a caller can just keep incrementing it.
     */
    SequenceStep at(std::size_t step) const
    {
        SequenceStep result;
        if (this->empty())
        {
            return result;
        }

        const std::size_t wrapped = step % this->size();

        if (!this->gains_.empty())
        {
            result.gain = this->gains_[wrapped % this->gainCount()];
            result.has_gain = true;
        }

        if (!this->exposures_.empty())
        {
            // wrapped < gainCount() * exposureCount(), so the division alone is in range
            result.exposure = this->exposures_[wrapped / this->gainCount()];
            result.has_exposure = true;
        }

        return result;
    }

private:
    // an absent list still counts as one, so it contributes a single pass
    // through the other list instead of collapsing the product to zero
    std::size_t gainCount() const
    {
        return this->gains_.empty() ? 1 : this->gains_.size();
    }

    std::size_t exposureCount() const
    {
        return this->exposures_.empty() ? 1 : this->exposures_.size();
    }

    std::vector<double> gains_;
    std::vector<double> exposures_;
};

}  // namespace pylon_ros2_camera
