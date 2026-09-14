#include "test_framework.h"

#include <atomic>

static std::atomic<bool> s_abort{false};
static std::atomic<bool> s_done{false};

bool test_should_abort() { return s_abort.load(); }

void test_flags_reset()     { s_abort = false; s_done = false; }
void test_flags_set_abort() { s_abort = true; }
void test_flags_set_done()  { s_done = true; }
bool test_flags_done()      { return s_done.load(); }
