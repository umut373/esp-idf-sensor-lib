#ifndef CALIBRATE_H_
#define CALIBRATE_H_

template <typename T>
class Calibrate {
public:
    virtual void calibrate() = 0;
    virtual void calibrate(const T& calib_data) = 0;
};

#endif // CALIBRATE_H_