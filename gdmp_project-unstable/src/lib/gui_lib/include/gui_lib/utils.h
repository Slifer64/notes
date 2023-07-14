#ifndef GUI_LIB_UTILS_H
#define GUI_LIB_UTILS_H

#include <memory>
#include <functional>

#include <QLineEdit>
#include <QApplication>
#include <QMainWindow>
#include <QThread>

#include <mutex>
#include <condition_variable>

namespace as64_
{

namespace gui_
{

class MyLineEdit: public QLineEdit
{
public:
    MyLineEdit(QWidget *parent=0):QLineEdit(parent) { size_hint = QSize(60,30); }
    MyLineEdit(const QString &contents, QWidget *parent = nullptr):QLineEdit(contents,parent) { size_hint = QSize(60,30); }

    void setSizeHint(int w, int h) { size_hint.setWidth(w); size_hint.setHeight(h); }
    QSize sizeHint() const override { return size_hint; }
private:
    QSize size_hint;
};

class Semaphore
{
private:
  std::mutex mutex_;
  std::condition_variable condition_;
  // unsigned long count_ = 0; // Initialized as locked.
  bool count_ = false;  // Initialized as locked.

public:

  void notify()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    // ++count_;
    count_ = true;
    condition_.notify_one();
  }

  void wait()
  {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    while(!count_) // Handle spurious wake-ups.
      condition_.wait(lock);
    // --count_;
    count_ = false;
  }

  bool wait_for(double time_ms)
  {
    std::unique_lock<decltype(mutex_)> lock(mutex_);

    int wait_time = time_ms * 1e6;

    while(!count_) // Handle spurious wake-ups.
    {
      if (condition_.wait_for(lock,std::chrono::nanoseconds(wait_time))==std::cv_status::timeout)
        return false;
    }
    // --count_;
    count_ = false;

    return true;
  }

  bool try_wait()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    if(count_)
    {
      // --count_;
      count_ = false;
      return true;
    }
    return false;
  }
};


void launchGui(std::function<QMainWindow *()> create_mainwin_fun_, bool *gui_finished, int thr_priority=QThread::LowestPriority, std::function<void()>cookie_fun=nullptr);

} // namespace gui_

} // namespace as64_

#endif // GUI_LIB_UTILS_H
