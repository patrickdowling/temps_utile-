#ifndef DRIVERS_FRAMEBUFFER_H_
#define DRIVERS_FRAMEBUFFER_H_

#include <array>

#include "../util/util_macros.h"

namespace util {

// - This could be specialized for frames == 2 (i.e. double-buffer)
// - Takes some short-cuts so assumes correct order of calls
// - Takes an optional 'meta' type that can be used to add data for the frame
// - Yeah, the naming is increasingly awkward
// TODO(ish) Make the Frame type "smart" and auto-release the buffers

// Initial version used a different ring-buffer implementation that used
// frames - 1 elements to be able to distinguish between empty/full, but
// which also meant that it wasn't properly double-buffering.
// This uses an alternate approach that relies on unsigned integer wrap,
// but allows a new frame to be written while the old one is being
// transferred.
// See https://gist.github.com/patrickdowling/0029f58fb20e63d7db9d

template <typename T>
struct FrameImpl {
  T *buffer = nullptr;
};

template <typename T, typename InfoType>
struct FrameType : public FrameImpl<T> {
  InfoType info = {};

  constexpr FrameType() = default;
  constexpr FrameType(T *b, const InfoType &i) : FrameImpl<T>{b}, info(i) {}
};

template <typename T>
struct FrameType<T, void> : public FrameImpl<T> {};

template <size_t frame_size, size_t num_frames, typename T = uint8_t, typename InfoType = void>
class FrameBuffer {
public:
  static const size_t kFrameSize = frame_size;

  using Frame = FrameType<T, InfoType>;

  FrameBuffer() {}

  void Init()
  {
    memset(frame_memory_, 0, sizeof(frame_memory_));

    auto buffer = frame_memory_;
    for (auto &f : frames_) {
      f.buffer = buffer;
      buffer += kFrameSize;
    }
  }

  size_t writeable() const { return num_frames - readable(); }

  size_t readable() const { return write_ptr_ - read_ptr_; }

  // @return readable frame (assumes one exists)
  const Frame *readable_frame() const { return &frames_[read_ptr_ % num_frames]; }

  // @return next writeable frame (assumes one exists)
  Frame *writeable_frame() { return &frames_[write_ptr_ % num_frames]; }

  void read() { ++read_ptr_; }

  void written() { ++write_ptr_; }

private:
  T frame_memory_[kFrameSize * num_frames] __attribute__((aligned(4)));

  std::array<Frame, num_frames> frames_;

  volatile size_t write_ptr_ = 0;
  volatile size_t read_ptr_ = 0;

  DISALLOW_COPY_AND_ASSIGN(FrameBuffer);
};

}  // namespace util
#endif  // DRIVERS_FRAMEBUFFER_H_
