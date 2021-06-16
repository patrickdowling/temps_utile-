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

template <typename T, size_t buffer_size>
class CircularSampleBuffer {
public:
  using value_type = T;
  static constexpr size_t kBufferSize = buffer_size;

  class Writer {
  public:
    Writer(CircularSampleBuffer *owner, T *buffer, size_t write_pos)
        : owner_(owner), buffer_(buffer), write_pos_(write_pos)
    {}

    T &operator*() { return buffer_[write_pos_ % kBufferSize]; }

    Writer &operator++(int)
    {
      ++write_pos_;
      return *this;
    }

    void Commit() { owner_->Commit(write_pos_); }

  private:
    CircularSampleBuffer *owner_;
    T *buffer_;
    size_t write_pos_;
  };

  Writer writer() { return {this, buffer_, write_pos_}; }

  void Read(T *buffer, size_t length) const
  {
    auto src = buffer_ + (read_pos_ % kBufferSize);
    auto end = src + length;
    if (end > end_) {
      buffer = std::copy(src, end_, buffer);
      std::copy(buffer_, buffer_ + (end - end_), buffer);
    } else {
      std::copy(src, end, buffer);
    }
  }

  size_t available() const { return write_pos_ - read_pos_; }

  void Consume() { read_pos_ = write_pos_; }
  void SetReadOffset(int32_t offset) { read_pos_ = (size_t)(write_pos_ + offset); }

  size_t write_pos() const { return write_pos_; }
  size_t read_pos() const { return read_pos_; }

private:
  T buffer_[kBufferSize];
  const T *const end_ = buffer_ + kBufferSize;
  size_t write_pos_ = 0;
  size_t read_pos_ = 0;

  friend class Writer;

  void Commit(size_t write_pos) { write_pos_ = write_pos; }
};

}  // namespace util

#endif  // UTIL_CIRCULAR_SAMPLE_BUFFER_H_

