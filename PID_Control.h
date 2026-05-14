#ifndef PID_CONTROL_H
#define PID_CONTROL_H

template <typename T>
class PID_Control {
public:
    enum class Mode {
        POSITION,
        VELOCITY
    };

    PID_Control(T kp, T ki, T kd, T dt, T min_out, T max_out, T alpha = 0.2f);

    void set_mode(Mode mode);
    void set_lpf_enabled(bool enabled);
    void set_shortest_path(bool enabled); 
    void set_alpha(T alpha);
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
};

#endif
