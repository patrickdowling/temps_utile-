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

#ifndef UTIL_POPUP_H_
#define UTIL_POPUP_H_

#include <stdint.h>

#include "../TU_ui.h"

namespace util {

class PopupElement {
public:
  static constexpr uint32_t kDefaultTimeoutTicks = 5000;

  bool visible() const { return visible_; }

  void Tick(uint32_t ticks)
  {
    if (visible_) {
      if (ticks - start_ticks_ > timeout_) visible_ = false;
    }
  }

  void hide() { visible_ = false; };
  void show()
  {
    visible_ = true;
    start_ticks_ = TU::ui.ticks();
  }

  void set_timeout(uint32_t timeout) { timeout_ = timeout; }

private:
  bool visible_ = false;
  uint32_t start_ticks_ = 0;
  uint32_t timeout_ = kDefaultTimeoutTicks;
};

}  // namespace util

#endif  // UTIL_POPUP_H_

