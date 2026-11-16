#pragma once
#include <mutex>
struct StaticSemaphore_t { std::timed_mutex mutex; };
