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

  // Head is read-only
  const T *head_buffer() const { return buffer_ + (head_ % kNumChunks) * kChunkSize; }

  // Extract data from head
  void ReadHead(T *buffer, int32_t start_offset, size_t length) const
  {
    auto src = head_buffer() + start_offset;
    auto end = src + length;

    if (src < buffer_) {
      buffer = std::copy(end_ - (buffer_ - src), end_, buffer);
      src = buffer_;
    }
    if (end > end_) {
      buffer = std::copy(src, end_, buffer);
      std::copy(buffer_, buffer_ + (end - end_), buffer);
    } else {
      std::copy(src, end, buffer);
    }
  }

  // Tail is writeable
  T *tail_buffer() { return buffer_ + (tail_ % kNumChunks) * kChunkSize; }

private:
  T buffer_[kBufferSize];
  const T *end_ = buffer_ + kBufferSize;

  size_t head_ = 0;
  size_t tail_ = kNumChunks - 1;
};

}  // namespace util

#endif  // UTIL_CIRCULAR_SAMPLE_BUFFER_H_

