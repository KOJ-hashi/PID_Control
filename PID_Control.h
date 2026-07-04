#ifndef PID_CONTROL_H
#define PID_CONTROL_H

template <typename T>
class PID_Control {
public:
    enum class Mode {
        POSITION,
        VELOCITY
    };

    PID_Control(T kp, T ki, T kd, T dt, T min_out, T max_out, T alpha = 0.2f, T tolerance = 0);

    void set_mode(Mode mode);
    void set_lpf_enabled(bool enabled);//不確定微分
    void set_shortest_path(bool enabled); //最短距離設定
    void set_alpha(T alpha);
    void set_tolerance(T tolerance);//許容誤差
    T update(T target, T current);
    void reset();

private:
    T _kp, _ki, _kd, _dt;
    T _min_out, _max_out, _alpha;
    Mode _mode;
    bool _use_lpf;
    bool _use_shortest_path; 

    T _integral;
    T _prev_error;
    T _prev_prev_error;
    T _low_pass_deriv;
    T _prev_output;
    T _tolerance;
};

#endif
