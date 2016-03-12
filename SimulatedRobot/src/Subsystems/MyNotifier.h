
#include "WPILib.h"
#include <atomic>
#include <functional>
#include <list>
#include <thread>

#include "ErrorBase.h"
//#include "HAL/cpp/std::mutex.h"

typedef std::function<void()> TimerEventHandler;

class MyNotifier : public ErrorBase {
 public:
  explicit MyNotifier(TimerEventHandler handler);

  template <typename Callable, typename Arg, typename... Args>
  MyNotifier(Callable&& f, Arg&& arg, Args&&... args)
      : MyNotifier(std::bind(std::forward<Callable>(f),
                           std::forward<Arg>(arg),
                           std::forward<Args>(args)...)) {}
  virtual ~MyNotifier();

  MyNotifier(const MyNotifier&) = delete;
  MyNotifier& operator=(const MyNotifier&) = delete;

  void StartSingle(double delay);
  void StartPeriodic(double period, int i);
  void Stop();
  int id;

 private:
  static std::list<MyNotifier*> timerQueue;
  static std::recursive_mutex queueMutex;
  static std::mutex halMutex;
  static void *m_notifier;
  static std::atomic<int> refcount;

  // Process the timer queue on a timer event
  static void ProcessQueue(uint32_t mask, void *params);

  // Update the FPGA alarm since the queue has changed
  static void UpdateAlarm();

  // Insert the MyNotifier in the timer queue
  void InsertInQueue(bool reschedule);

  // Delete this MyNotifier from the timer queue
  void DeleteFromQueue();

  // Address of the handler
  TimerEventHandler m_handler;
  // The relative time (either periodic or single)
  double m_period = 0;
  // Absolute expiration time for the current event
  double m_expirationTime = 0;
  // True if this is a periodic event
  bool m_periodic = false;
  // Indicates if this entry is queued
  bool m_queued = false;
  // Held by interrupt manager task while handler call is in progress
  std::mutex m_handlerMutex;
  static std::thread m_task;
  static std::atomic<bool> m_stopped;
  static void Run();
};
