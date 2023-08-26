#ifndef CONCURRENTQUEUE_H
#define CONCURRENTQUEUE_H

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

template <typename T>
class ConcurrentQueue
{
    public:

        ConcurrentQueue()
        {
            m_signalAbort = false;
        }

        bool Dequeue(T& data)
        {
            if (m_signalAbort == true)
                return false;

            std::unique_lock<std::mutex> lock(m_mutex);
            while (m_queue.empty())
            {
                m_cond.wait(lock);
                if (m_signalAbort == true)
                {
                    return false;
                }
            }
            data = m_queue.front();
            m_queue.pop();

            return true;
        }

        bool TryDequeue(T& data)
        {
            bool success = false;
            std::unique_lock<std::mutex> lock(m_mutex);
            if (m_queue.empty() == false)
            {
                data = m_queue.front();
                m_queue.pop();
                success = true;
            }
            return success;
        }

/*
        bool TryDequeue(T& data)
        {
            std::unique_lock<std::mutex> mlock(m_mutex);

            if (m_queue.empty())
            {
                return false;
            }
            else
            {
                data = m_queue.front();
                m_queue.pop();
                return true;
            }
        }
*/
        void Enqueue(const T& data)
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_queue.push(data);
            m_cond.notify_one();
        }


        void SignalAbort()
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_signalAbort = true;
            m_cond.notify_one();
        }


        void FlushQueue()
        {
             std::lock_guard<std::mutex> lock(m_mutex);
             while (m_queue.empty() == false)
             {
                T data = m_queue.front();
                delete data;
                m_queue.pop();
             }
        }


    private:
        std::queue<T> m_queue;
        std::mutex m_mutex;
        std::condition_variable m_cond;

        bool m_signalAbort;

};

#endif // CONCURRENTQUEUE_H
