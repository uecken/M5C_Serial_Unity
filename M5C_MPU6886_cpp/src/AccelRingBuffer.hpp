#include <cmath>

struct AccelData {
    float x, y, z;
    float absAccel;

    AccelData() : x(0), y(0), z(0), absAccel(0) {}
    
    AccelData(float x, float y, float z) : x(x), y(y), z(z) {
        absAccel = sqrt(x * x + y * y + z * z);
    }
};

class AccelRingBuffer {
private:
    AccelData *buffer;
    int capacity;
    int head;
        int count;

    public:
        AccelRingBuffer(int size) : capacity(size), head(0), count(0) {
            buffer = new AccelData[size];
        }

        ~AccelRingBuffer() {
            delete[] buffer;
        }



    void add(float x, float y, float z) {
        buffer[head] = AccelData(x, y, z);
        head = (head + 1) % capacity;
        if (count < capacity) count++;
    }

    float getMinAbsAccel(int n) {
        float minVal = INFINITY;
        for (int i = 0; i < n && i < count; ++i) {
            int index = (head - 1 - i + capacity) % capacity;
            if (buffer[index].absAccel < minVal) {
                minVal = buffer[index].absAccel;
            }
        }
        return minVal;
    }

    float getMaxAbsAccel(int n) {
        float maxVal = -INFINITY;
        for (int i = 0; i < n && i < count; ++i) {
            int index = (head - 1 - i + capacity) % capacity;
            if (buffer[index].absAccel > maxVal) {
                maxVal = buffer[index].absAccel;
            }
        }
        return maxVal;
    }

    float getAverageAbsAccel(int n) {
        float sum = 0.0;
        int validCount = 0;
        for (int i = 0; i < n && i < count; ++i) {
            int index = (head - 1 - i + capacity) % capacity;
            sum += buffer[index].absAccel;
            validCount++;
        }
        return validCount > 0 ? sum / validCount : 0.0;
    }
};