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

float PID_Control::update(float target,float current){
    float error = target - current;

    integral += (error + pre_error) * _dt / 2.0f;

    if(integral > i_limit) integral = i_limit;
    if(integral < -i_limit)integral = -i_limit;

    float derivative = (error - pre_error) / _dt;
    float output_val = 0;

    if (_mode == Mode::SPEED) {
        pre_mv = (error * _kp) + (integral * _ki) + (derivative * _kd);
        output_val = pre_mv;
    }else{
        pre_mv=(error * _kp) + (integral * _ki) + (derivative * _kd);
    }

    if (pre_mv > _max_out) pre_mv = _max_out;
    if (pre_mv < _min_out) pre_mv = _min_out;

    pre_mv = output_val;

    pre_error = error;
    return output_val;
}

void PID_Control::reset(){
    integral = 0.0f;
    pre_error = 0.0f;
    pre_mv = 0.0f;
}
