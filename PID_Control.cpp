#include "PID_Control.h"

template <typename T>
PID_Control<T>::PID_Control(T kp, T ki, T kd, T dt, T min_out, T max_out, T alpha)
    : _kp(kp), _ki(ki), _kd(kd), _dt(dt), 
      _min_out(min_out), _max_out(max_out), _alpha(alpha),
      _mode(Mode::POSITION), _use_lpf(true), _use_shortest_path(true),
      _integral(0), _prev_error(0), _prev_prev_error(0), 
      _low_pass_deriv(0), _prev_output(0) {}

template <typename T>
void PID_Control<T>::set_mode(Mode mode) {
    _mode = mode;
    reset();
}

template <typename T>
void PID_Control<T>::set_lpf_enabled(bool enabled) {
    _use_lpf = enabled;
}

template <typename T>
void PID_Control<T>::set_shortest_path(bool enabled) {
    _use_shortest_path = enabled;
}

template <typename T>
void PID_Control<T>::set_alpha(T alpha) {
    _alpha = alpha;
}

template <typename T>
T PID_Control<T>::update(T target, T current) {
    T error = target - current;

    // 最短経路フラグが有効な場合のみ、±190度の範囲に丸める
    if (_use_shortest_path) {
        while (error > 190.0f)  error -= 360.0f;
        while (error < -190.0f) error += 360.0f;
    }

    T output = 0;

    if (_mode == Mode::POSITION) {
        _integral += error * _dt;
        if (_integral * _ki > _max_out) _integral = _max_out / _ki;
        if (_integral * _ki < _min_out) _integral = _min_out / _ki;

        T raw_diff = (error - _prev_error) / _dt;
        if (_use_lpf) {
            _low_pass_deriv = (_alpha * raw_diff) + ((1.0f - _alpha) * _low_pass_deriv);
        } else {
            _low_pass_deriv = raw_diff;
        }
        output = (_kp * error) + (_ki * _integral) + (_kd * _low_pass_deriv);

    } else {
        // --- 速度型 PID ---
        T p_delta = _kp * (error - _prev_error);
        T i_delta = _ki * error * _dt;
        T d_raw_delta = _kd * (error - 2 * _prev_error + _prev_prev_error) / _dt;
        
        if (_use_lpf) {
            _low_pass_deriv = (_alpha * d_raw_delta) + ((1.0f - _alpha) * _low_pass_deriv);
        } else {
            _low_pass_deriv = d_raw_delta;
        }
        
        _prev_output += p_delta + i_delta + _low_pass_deriv;
        output = _prev_output;
    }

    if (output > _max_out) output = _max_out;
    if (output < _min_out) output = _min_out;
    
    _prev_output = output;
    _prev_prev_error = _prev_error;
    _prev_error = error;

    return output;
}

template <typename T>
void PID_Control<T>::reset() {
    _integral = 0;
    _prev_error = 0;
    _prev_prev_error = 0;
    _low_pass_deriv = 0;
    _prev_output = 0;
}

// 明示的実体化
template class PID_Control<float>;
template class PID_Control<double>;
