// Copyright (c) 2021 Patrick Dowling
//
// Author: Patrick Dowling (pld@gurkenkiste.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef UI_EVENT_DISPATCHER_H_
#define UI_EVENT_DISPATCHER_H_

#include "ui_events.h"

namespace UI {

template <typename T>
class EventDispatcher {
public:
  void DispatchEvent(const UI::Event &event)
  {
    auto event_handlers = static_cast<const T *>(this)->get_event_handlers();
    while (event_handlers->fn) {
      if (event_handlers->event_type == event.type && event_handlers->control == event.control) {
        (static_cast<T *>(this)->*event_handlers->fn)(event.type, event.value);
        break;
      }
      ++event_handlers;
    }
  }

protected:
  using EventHandlerFn = void (T::*)(UI::EventType, int16_t);

  struct EventHandler {
    const UI::EventType event_type = UI::EVENT_NONE;
    const uint16_t control = 0;
    const EventHandlerFn fn = nullptr;
  };
};

}  // namespace UI

#define EVENT_DISPATCH_DECLARE_HANDLER(fn) void fn(UI::EventType, int16_t)
#define EVENT_DISPATCH_DEFINE_HANDLER(cls, fn) \
  void cls::fn(UI::EventType event_type, int16_t event_value)
#define EVENT_DISPATCH_HANDLER_STUB() \
  {                                   \
    do {                              \
      (void)event_type;               \
      (void)event_value;              \
    } while (0);                      \
  }

#endif  // UI_EVENT_DISPATCHER_H_
