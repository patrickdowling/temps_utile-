// Copyright 2021 Patrick Dowling
// Author: Patrick Dowling (pld@gurkenkiste.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// See http://creativecommons.org/licenses/MIT/ for more information.
//
#ifndef UTIL_EDGE_DETECTOR_
#define UTIL_EDGE_DETECTOR_

#include <stdint.h>

namespace util {

// "rising" and "falling" are relative to cmp, i.e. false->true and true->false transitions.
template <typename T>
class EdgeDetector {
public:
  using State = uint32_t;
  static constexpr uint32_t kEdgeMask = 0xf;
  static constexpr uint32_t kRisingEdge = 0x3;   // 0011
  static constexpr uint32_t kFallingEdge = 0xc;  // 1100

  EdgeDetector(T threshold, State state) : threshold_(threshold), state_(state) {}

  template <typename cmp>
  void Update(T value)
  {
    state_ = (state_ << 1) | cmp{}(value, threshold_);
  }
  void Reset() { state_ = 0; }

  inline bool rising_edge() const { return (state_ & kEdgeMask) == kRisingEdge; }
  inline bool falling_edge() const { return (state_ & kEdgeMask) == kFallingEdge; }

  inline State state() const { return state_; }

private:
  const T threshold_ = 0;
  State state_ = 0;
};

}  // namespace util

#endif  // UTIL_EDGE_DETECTOR_
