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

#ifndef UTIL_CIRCULAR_SAMPLE_BUFFER_H_
#define UTIL_CIRCULAR_SAMPLE_BUFFER_H_

#include <stdint.h>

namespace util {

template <typename T, size_t chunk_size, size_t num_chunks>
class CircularSampleBuffer {
public:
  static constexpr size_t kChunkSize = chunk_size;
  static constexpr size_t kNumChunks = num_chunks;
  static constexpr size_t kBufferSize = kChunkSize * kNumChunks;

  void advance()
  {
    ++head_;
    ++tail_;
  }

  const T *head_buffer() const { return buffer_ + (head_ % kNumChunks) * kChunkSize; }
  const T *head_buffer(size_t i) const { return buffer_ + ((head_ + 1) % kNumChunks) * kChunkSize; }

  T *tail_buffer() { return buffer_ + (tail_ % kNumChunks) * kChunkSize; }

private:
  T buffer_[kBufferSize];
  size_t head_ = 0;
  size_t tail_ = kNumChunks - 1;
};

}  // namespace util

#endif  // UTIL_CIRCULAR_SAMPLE_BUFFER_H_

