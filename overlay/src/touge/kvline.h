#pragma once
//
// Named counters written two ways from one list: the serial line
// "ll li=5000 la=5012" and the phone's JSON {"ll":{"li":5000,"la":5012}}.
// One call site per report means the log and the phone cannot drift apart on a
// name, which is what makes a drive's serial log and its app log line up.
//
// Header-only and platform-free.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

namespace touge {

class KvLine {
 public:
  KvLine(char* out, size_t cap, const char* report, bool json) : out_(out), cap_(cap), json_(json) {
    if (out_ == nullptr || cap_ == 0) {
      failed_ = true;
      return;
    }
    if (json_) {
      text("{\"");
      text(report);
      text("\":{");
    } else {
      text(report);
    }
  }

  void add(const char* key, uint32_t value) {
    if (json_) {
      text(first_ ? "\"" : ",\"");
      text(key);
      text("\":");
    } else {
      text(" ");
      text(key);
      text("=");
    }
    number(value);
    first_ = false;
  }

  // Bytes written, 0 if anything did not fit.
  size_t finish() {
    if (json_) text("}}");
    return failed_ ? 0 : at_;
  }

 private:
  void text(const char* s) {
    if (!failed_) wrote(snprintf(out_ + at_, cap_ - at_, "%s", s));
  }
  void number(uint32_t v) {
    if (!failed_) wrote(snprintf(out_ + at_, cap_ - at_, "%lu", (unsigned long)v));
  }
  void wrote(int n) {
    if (n < 0 || (size_t)n >= cap_ - at_) {
      failed_ = true;
      return;
    }
    at_ += (size_t)n;
  }

  char* out_;
  size_t cap_;
  size_t at_ = 0;
  bool json_;
  bool first_ = true;
  bool failed_ = false;
};

}  // namespace touge
