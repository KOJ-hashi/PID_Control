#include "PID_Control.h"
#include <cmath> 

template <typename T>
PID_Control<T>::PID_Control(T kp, T ki, T kd, T dt, T min_out, T max_out, T alpha)
    : _kp(kp), _ki(ki), _kd(kd), _dt(dt), 
      _min_out(min_out), _max_out(max_out), _alpha(alpha),
      _mode(Mode::POSITION), _use_lpf(true), _use_shortest_path(true),
      _integral(0), _prev_error(0), _prev_prev_error(0), 
      _low_pass_deriv(0), _prev_output(0),
      // 初期値を追記
      _use_scurve(false), _max_jerk(40000.0f), _max_accel(2000.0f),
      _scurve_speed(0), _scurve_accel(0) {}

// S字制御の設定関数
template <typename T>
void PID_Control<T>::set_scurve_enabled(bool enabled) {
    _use_scurve = enabled;
    if (!enabled) {
        // 無効化されたときは内部状態をクリアしておく
        _scurve_speed = 0;
        _scurve_accel = 0;
    }
}

template <typename T>
void PID_Control<T>::set_scurve_params(T max_jerk, T max_accel) {
    _max_jerk = max_jerk;
    _max_accel = max_accel;
}

template <typename T>
T PID_Control<T>::update(T target, T current) {
    
    // S字フラグが真なら、入力された target をS字に加工する
    T final_target = target;
    if (_use_scurve) {
        // 1. 必要加速度の計算と制限
        T target_accel = (target - _scurve_speed) / _dt;
        if (target_accel > _max_accel)  target_accel = _max_accel;
        if (target_accel < -_max_accel) target_accel = -_max_accel;

        // 2. ジャーク（加速度の変化量）の計算と制限
        T jerk = (target_accel - _scurve_accel) / _dt;
        if (jerk > _max_jerk)  jerk = _max_jerk;
        if (jerk < -_max_jerk) jerk = -_max_jerk;

        // 3. 積分して新たな滑らかターゲット速度を算出
        _scurve_accel += jerk * _dt;
        _scurve_speed += _scurve_accel * _dt;
        
        final_target = _scurve_speed; // PIDの計算にはこのS字速度を使う
    }

    // 従来の target を使用していた部分を、すべて final_target に置き換える
    T error = final_target - current;

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
        
        // 前回の回答通りのワインドアップ対策
        if (_prev_output > _max_out) _prev_output = _max_out;
        if (_prev_output < _min_out) _prev_output = _min_out;
        
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
    // S字状態もリセット
    _scurve_speed = 0;
    _scurve_accel = 0;
}

// 明示的実体化
template class PID_Control<float>;
template class PID_Control<double>;
