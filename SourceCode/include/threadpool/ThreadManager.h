#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#pragma once
class ThreadManager {
 private:
  std::mutex mutex;
  std::condition_variable conditionVariableNewTask;
  std::condition_variable conditionVariableAllFinished;
  bool isWorking = false;
  int activeThreads = 0;
  std::queue<std::function<void()>> jobs;
  std::vector<std::thread> threads;

 public:
  explicit ThreadManager(unsigned int numberOfThreads) : isWorking(true) {
    threads.reserve(numberOfThreads);
    for (unsigned int i = 0; i < numberOfThreads; i++) {
      threads.emplace_back([this, i] { threadProcess(); });
    }
  }
  ThreadManager(const ThreadManager &other) = delete;
  ThreadManager &operator=(const ThreadManager &other) = delete;
  ThreadManager(ThreadManager &&other) = delete;
  ThreadManager &&operator=(ThreadManager &&other) = delete;
  ~ThreadManager() {
    {
      std::unique_lock<std::mutex> lock(this->mutex);
      isWorking = false;
      conditionVariableNewTask.notify_all();
    }
    for (auto &thread : threads) {
      thread.join();
    }
  }

  void doJob(const std::function<void()> &func) {
    std::unique_lock<std::mutex> lock(mutex);
    jobs.emplace(func);
    conditionVariableNewTask.notify_one();
  }
  // this function makes the ThreadManager block until
  // there are no scheduled and no running tasks
  void waitForAll() {
    std::unique_lock<std::mutex> lock(this->mutex);
    conditionVariableAllFinished.wait(lock, [this]() { return this->jobs.empty() && this->activeThreads == 0; });
  }

 private:
  // in C++, a thread can do only one job, there can't be new jobs added
  // but we can define a thread's job as "constantly waiting for a new job"
  // and fetch that new job from a queue
  void threadProcess() {
    while (true) {
      std::unique_lock<std::mutex> lock(mutex);
      conditionVariableNewTask.wait(lock, [this]() { return !isWorking || !jobs.empty(); });
      if (!jobs.empty()) {
        auto job = jobs.front();
        jobs.pop();
        this->activeThreads++;
        lock.unlock();
        job();
        lock.lock();
        this->activeThreads--;
        conditionVariableAllFinished.notify_one();
      } else if (!isWorking) {
        break;
      }
    }
  }
};