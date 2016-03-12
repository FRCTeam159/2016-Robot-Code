/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "MyNotifier.h"
#include "Timer.h"
#include "Utility.h"
#include "WPIErrors.h"

//#define DEBUG_NOTIFIER

std::list<MyNotifier*> MyNotifier::timerQueue;
std::recursive_mutex MyNotifier::queueMutex;
std::atomic<int> MyNotifier::refcount{0};
std::thread MyNotifier::m_task;
std::atomic<bool> MyNotifier::m_stopped(false);

/**
 * Create a MyNotifier for timer event notification.
 * @param handler The handler is called at the notification time which is set
 * using StartSingle or StartPeriodic.
 */
MyNotifier::MyNotifier(TimerEventHandler handler)
{
	if (handler == nullptr)
		wpi_setWPIErrorWithContext(NullParameter, "handler must not be nullptr");
	m_handler = handler;
	m_periodic = false;
	m_expirationTime = 0;
	m_period = 0.1;
	id=0;
	m_queued = false;
#ifdef DEBUG_NOTIFIER
	std::cout<<"MyNotifier::MyNotifier"<<std::endl;
#endif
	{
		std::lock_guard<std::recursive_mutex> sync(queueMutex);
		// do the first time initialization of static variables
		if (refcount.fetch_add(1) == 0) {
			m_task = std::thread(Run); // start the timer thread when the first notifier is created
		}
	}
}

/**
 * Free the resources for a timer event.
 * All resources will be freed and the timer event will be removed from the
 * queue if necessary.
 */
MyNotifier::~MyNotifier()
{
	{
		std::lock_guard<std::recursive_mutex> sync(queueMutex);
#ifdef DEBUG_NOTIFIER
		std::cout<<"MyNotifier::~MyNotifier("<<id<<")"<<std::endl;
#endif
		DeleteFromQueue();

		// Delete the static variables when the last one is going away
		if (refcount.fetch_sub(1) == 1)
		{
			m_stopped = true;
			m_task.join();
		}
	}

	// Acquire the semaphore; this makes certain that the handler is
	// not being executed by the interrupt manager.
	std::lock_guard<std::mutex> lock(m_handlerMutex);
}

/**
 * Update the alarm hardware to reflect the current first element in the queue.
 * Compute the time the next alarm should occur based on the current time and the
 * period for the first element in the timer queue.
 * WARNING: this method does not do synchronization! It must be called from somewhere
 * that is taking care of synchronizing access to the queue.
 */
void MyNotifier::UpdateAlarm()
{
	// nothing to do in simulation
}

/**
 * ProcessQueue is called whenever there is a timer interrupt.
 * We need to wake up and process the current top item in the timer queue as long
 * as its scheduled time is after the current time. Then the item is removed or
 * rescheduled (repetitive events) in the queue.
 *
 * note: This function is static (all others are called for each instance)
 */
void MyNotifier::ProcessQueue(uint32_t mask, void *params)
{
	MyNotifier *current;
	while (true)				// keep processing past events until no more
	{
		{
			std::lock_guard<std::recursive_mutex> sync(queueMutex);
			double currentTime = GetClock();

			if (timerQueue.empty())
			{
				break;
			}
			current = timerQueue.front();  // front should be shortest time left in queue
			if (current->m_expirationTime > currentTime)
			{
				break;		// wait for next timer event
			}
			// remove next entry before processing it
			timerQueue.pop_front();

			current->m_queued = false;
			if (current->m_periodic)
			{
				// if periodic, requeue the event
				// compute when to put into queue

				current->InsertInQueue(true);
			}
			else
			{
				// not periodic; removed from queue
				current->m_queued = false;
			}
			// Take handler mutex while holding queue semaphore to make sure
			//  the handler will execute to completion in case we are being deleted.
			current->m_handlerMutex.lock();
#ifdef DEBUG_NOTIFIER
			std::cout<<currentTime<< " MyNotifier::ProcessQueue("<<current->id<<") Processing"<<std::endl;
#endif
			current->m_handler();	// call the event handler (PIDController.Calculate)
			current->m_handlerMutex.unlock();
		}

	}
	// reschedule the first item in the queue
	std::lock_guard<std::recursive_mutex> sync(queueMutex);
	UpdateAlarm();
}

/**
 * Insert this MyNotifier into the timer queue in right place.
 * WARNING: this method does not do synchronization! It must be called from somewhere
 * that is taking care of synchronizing access to the queue.
 * @param reschedule If false, the scheduled alarm is based on the curent time and UpdateAlarm
 * method is called which will enable the alarm if necessary.
 * If true, update the time by adding the period (no drift) when rescheduled periodic from ProcessQueue.
 * This ensures that the public methods only update the queue after finishing inserting.
 */
//#define OFFSET_INSERT_TIMES
void MyNotifier::InsertInQueue(bool reschedule)
{
	double ct=GetClock();
	if (reschedule)
	{
		m_expirationTime += m_period;
	}
	else
	{
#ifdef OFFSET_INSERT_TIMES // skew insertion times for new periodics
		static int pcnt=1;
		m_expirationTime = ct +(1.2*m_period*pcnt++);
#else
		m_expirationTime = ct;
#endif
	}

	// Attempt to insert new entry into queue
	for (auto i = timerQueue.begin(); i != timerQueue.end(); i++)
	{
		if ((*i)->m_expirationTime > m_expirationTime)
		{
			timerQueue.insert(i, this);
			m_queued = true;
			// BUG 1: wpi version doesn't break here so keeps inserting in front of all elements
			// with expiration times > current element
			break;
		}
	}

	/* If the new entry wasn't queued, either the queue was empty or all
	 * elements are LESS than the new entry (new element is longer than all elements in the queue)
	 */
	if (!m_queued)
	{
		// BUG 2: wpi version uses "push_front" which is wrong since it adds the longest time to the front
		timerQueue.push_back(this);
#ifdef DEBUG_NOTIFIER
		if (!reschedule)
		{
			/* Since the first element changed, update alarm, unless we already
			 * plan to
			 */
			std::cout<<ct<<" MyNotifier::InsertInQueue("<<id<<") First:"<<m_expirationTime<<std::endl;
			UpdateAlarm();
		}
		else
			std::cout<<ct<<" MyNotifier::InsertInQueue("<<id<<") Inserting to front:"<<m_expirationTime<<std::endl;
#endif

		m_queued = true;
	}
#ifdef DEBUG_NOTIFIER
	if(reschedule)
		std::cout<<ct<<" MyNotifier::InsertInQueue("<<id<<") Next:"<<m_expirationTime<<std::endl;
#endif
}

/**
 * Delete this MyNotifier from the timer queue.
 * WARNING: this method does not do synchronization! It must be called from somewhere
 * that is taking care of synchronizing access to the queue.
 * Remove this MyNotifier from the timer queue and adjust the next interrupt time to reflect
 * the current top of the queue.
 */
void MyNotifier::DeleteFromQueue()
{
	if (m_queued)
	{
		m_queued = false;
		wpi_assert(!timerQueue.empty());
		if (timerQueue.front() == this)
		{
			// remove the first item in the list - update the alarm
			timerQueue.pop_front();
			UpdateAlarm();
		}
		else
		{
			timerQueue.remove(this);
		}
	}
}

/**
 * Register for single event notification.
 * A timer event is queued for a single event after the specified delay.
 * @param delay Seconds to wait before the handler is called.
 */
void MyNotifier::StartSingle(double delay)
{
	std::lock_guard<std::recursive_mutex> sync(queueMutex);
	m_periodic = false;
	m_period = delay;
	DeleteFromQueue();
	InsertInQueue(false);
}

/**
 * Register for periodic event notification.
 * A timer event is queued for periodic event notification. Each time the interrupt
 * occurs, the event will be immediately requeued for the same time interval.
 * @param period Period in seconds to call the handler starting one period after the call to this method.
 */
void MyNotifier::StartPeriodic(double period, int i)
{
	std::lock_guard<std::recursive_mutex> sync(queueMutex);
	m_periodic = true;
	m_period = period;
	id=i;
	//m_queued=false;

	DeleteFromQueue();
	InsertInQueue(false); // note: false ensures that first time called will be > current time
}

/**
 * Stop timer events from occuring.
 * Stop any repeating timer events from occuring. This will also remove any single
 * notification events from the queue.
 * If a timer-based call to the registered handler is in progress, this function will
 * block until the handler call is complete.
 */
void MyNotifier::Stop()
{
	{
		std::lock_guard<std::recursive_mutex> sync(queueMutex);
		DeleteFromQueue();
	}
	// Wait for a currently executing handler to complete before returning from Stop()
#ifdef DEBUG_NOTIFIER
	std::cout<<"MyNotifier::Stop("<<id<<")"<<std::endl;
#endif
	std::lock_guard<std::mutex> sync(m_handlerMutex);
}

void MyNotifier::Run() {
    while (!m_stopped) {
        MyNotifier::ProcessQueue(0, nullptr);
        bool isEmpty;
        {
            std::lock_guard<std::recursive_mutex> sync(queueMutex);
            isEmpty = timerQueue.empty();
        }
        if (!isEmpty)
        {
            double expirationTime;
            {
                std::lock_guard<std::recursive_mutex> sync(queueMutex);
                expirationTime = timerQueue.front()->m_expirationTime;
            }
            Wait(expirationTime - GetClock());
        }
        else
        {
            Wait(0.05);
        }
    }
}
