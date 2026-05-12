#include "mbed.h"
#include "PID_Control.hpp"
#include <algorithm>

PID_Control::PID_Control(float dt,float kp,float ki,float kd,Mode mode )
:_dt(dt),_kp(kp),_ki(ki),_kd(kd),_mode(mode),integral(0),pre_error(0),pre2_error(0),pre_mv(0),_min_out(-10000.0f),_max_out(10000.f){}

void PID_Control::set_gains(float kp, float ki, float kd){//mainでゲインを入力する
    _kp = kp , _ki = ki , _kd = kd;
}

void PID_Control::Output_limit(float min, float max){
        _min_out = min;     _max_out = max;
}

void PID_Control::Integral_limit(float limit){
    i_limit = limit;
}

void PID_Control::setMode(Mode mode){
    if(_mode != mode){
        _mode = mode;
        reset();
    }
}

float PID_Control::update(float target, float current) {
    float error = target - current;
    float output_val = 0;

    if (_mode == Mode::SPEED) {
        // 速度型（増分型）PID
        float dmv = _kp * (error - pre_error) 
                  + _ki * error * _dt 
                  + _kd * (error - 2.0f * pre_error + pre2_error) / _dt;
        
        output_val = pre_mv + dmv;
    } else {
        // 位置型PID
        integral += (error + pre_error) * _dt / 2.0f;
        
        // Iリミット
        if(i_limit > 0) {
            if(integral > i_limit) integral = i_limit;
            if(integral < -i_limit) integral = -i_limit;
        }
        
        float derivative = (error - pre_error) / _dt;
        output_val = (error * _kp) + (integral * _ki) + (derivative * _kd);
    }

    // 出力リミッター
    if (output_val > _max_out) output_val = _max_out;
    if (output_val < _min_out) output_val = _min_out;

    // 履歴の更新
    pre2_error = pre_error;
    pre_error = error;
    pre_mv = output_val; 

    return output_val;
}

void PID_Control::reset(){
    integral = 0.0f;
    pre_error = 0.0f;
    pre_mv = 0.0f;
}
