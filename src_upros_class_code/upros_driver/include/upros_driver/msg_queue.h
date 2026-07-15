#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class MsgQueue
{
private:
    std::mutex mut;
    std::condition_variable cond;
    std::queue<T> q;

public:
    bool isEmpty()
    {
        return q.empty();
    }

    void push(T new_value)
    {
        std::lock_guard<std::mutex> lock(mut);
        q.push(new_value);
        cond.notify_one();
    }

    T get_and_pop()
    {
        std::unique_lock<std::mutex> lock(mut);
        cond.wait(lock, [&]
                  { return !q.empty(); });
        T value = q.front();
        q.pop();
        return value;
    }

    bool try_pop(T &value)
    {
        std::lock_guard<std::mutex> lock(mut);
        if (q.empty())
            return false;
        value = q.front();
        q.pop();
        return true;
    }
};
