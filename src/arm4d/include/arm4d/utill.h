#pragma once
#include <chrono>

namespace arm4d {

template<typename T>
void enqueue_blocking(std::queue<T>& q, std::mutex& mtx, std::condition_variable& cv, const T& item, size_t max_size) {
	for (;;) {
		std::unique_lock<std::mutex> lk(mtx);
		if (q.size() < max_size) {
			q.push(item);
			cv.notify_one();
			return;
		}
		lk.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
};

}