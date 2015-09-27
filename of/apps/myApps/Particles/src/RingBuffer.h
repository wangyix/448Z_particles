#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdlib.h>

template <class T, size_t N>
class RingBuffer {
public:
    class Iterator {
    public:
        Iterator(T* values, size_t n, size_t index) : values(values), n(n), index(index) {}
        Iterator& operator++() {
            index = (index + 1) % (n + 1);
            return *this;
        }
        Iterator& operator--() {
            index = (index + n) % (n + 1);
            return *this;
        }
        Iterator& operator+=(int d) {
            index = (index + d) % (n + 1);
            if (index < 0) {
                index += (n + 1);
            }
            return *this;
        }
        Iterator& operator-=(int d) {
            index = (index - d) % (n + 1);
            if (index < 0) {
                index += (n + 1);
            }
            return *this;
        }
        Iterator operator+(int d) const {
            return (Iterator(*this) += d);
        }
        Iterator operator-(int d) const {
            return (Iterator(*this) -= d);
        }
        T& operator*() const {
            return values[index];
        }
        T& operator[](int d) const {
            return *(*this + d);
        }
        bool operator==(const Iterator& iter) {
            return ((values + index) == (iter.values + iter.index));
        }
        bool operator!=(const Iterator& iter) {
            return !(*this == iter);
        }
    private:
        T* values;
        size_t n;
        size_t index;
    };

public:
    RingBuffer() : rIndex(0), wIndex(0) {}

    size_t push(const T* buffer, size_t n) {
        n = std::min(n, capacityRemaining());
        size_t toEnd = N + 1 - wIndex;
        size_t n1 = std::min(toEnd, n);
        memcpy(&values[wIndex], buffer, n1 * sizeof(T));
        memcpy(values, buffer + n1, (n - n1) * sizeof(T));
        wIndex = (wIndex + n) % (N + 1);
        return n;
    }
    size_t push(const T& value, size_t n) {
        n = std::min(n, capacityRemaining());
        for (int i = 0; i < n; i++) {
            values[wIndex] = value;
            wIndex = (wIndex + 1) % (N + 1);
        }
    }
    size_t push(const T& value) {
        if (capacityRemaining() == 0) {
            return 0;
        }
        values[wIndex] = value;
        wIndex = (wIndex + 1) % (N + 1);
        return 1;
    }
    size_t peek(T* buffer, size_t n) const {
        n = std::min(n, size());
        size_t toEnd = N + 1 - rIndex;
        size_t n1 = std::min(toEnd, n);
        memcpy(buffer, &values[rIndex], n1 * sizeof(T));
        memcpy(buffer + n1, values, (n - n1) * sizeof(T));
        return n;
    }
    size_t pop(T* buffer, size_t n) {
        n = peek(buffer, n);
        rIndex = (rIndex + n) % (N + 1);
        return n;
    }
    size_t pop(size_t n) {
        n = std::min(n, size());
        rIndex = (rIndex + n) % (N + 1);
        return n;
    }

    size_t size() const {
        return (N + 1 + wIndex - rIndex) % (N + 1);
    }
    size_t capacityRemaining() const {
        return (N + rIndex - wIndex) % (N + 1);
    }

    Iterator begin() {
        return Iterator(values, N, rIndex);
    }
    Iterator end() {
        return Iterator(values, N, wIndex);
    }

    size_t capacity() const {
        return N;
    }
private:
    T values[N + 1];
    size_t rIndex;
    size_t wIndex;
};

#endif
