#ifndef PID_CONTROL_H
#define PID_CONTROL_H

template <typename T>

class PID_Control{
    public:
    enum class Mode{
        POSITION,//位置型
        SPEED//速度型
    };

    PID_Control(T kp, T ki, T kd, T dt, T min_out, T max_out, T alpha = 0.2f);

    void set_mode(Mode mode);
    void set_lpf_enabled(bool enabled);
    void set_alpha(T alpha);
    void update(T target, T current);
    void reset();

    private:

    T _dt,_kp,_ki,_kd;
    T _min_out,_max_out;
    T _alpha;
    Mode _mode;
    bool _use_lpf;

    T _integral;
    T _prev_error;
    T _prev_prev_error;
    T _low_pass_deriv;
    T _prev_output;

};

#endif
