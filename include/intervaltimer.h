#ifndef INTERVAL_TIMER_H
#define INTERVAL_TIMER_H

class IntervalTimer
{
private:
  unsigned long interval;
  unsigned long lastTime;
  unsigned long delta;

public:
  IntervalTimer(unsigned long interval) : interval(interval), lastTime(0), delta(0) {}

  void setLastTime(unsigned long lastTime) { this->lastTime = lastTime; }

  unsigned long getLastDelta() { return delta; }

  // The boolean operator contains the most pertinent logic for the IntervalTimer class.
  // It is "called" whenever the timer is used in a conditional expression.
  operator bool()
  {
    unsigned long now = millis();
    delta = now - lastTime;
    if (delta >= interval)
    {
      lastTime = now;
      return true;
    }
    return false;
  }
};

#endif