#ifndef PID_CONTROL_HPP
#define PID_CONTROL_HPP

class PID_Control{
    public:
    enum class Mode{
        POSITION,
        SPEED
    };

    PID_Control(float dt, float kp, float ki, float kd, Mode mode =Mode::POSITION);

    void set_gains(float kp, float ki, float kd);//ゲイン調整
    void Output_limit(float min, float max);//リミット制限
    void Integral_limit(float limit);
    void setMode(Mode mode);
    void reset();//リセット

    float update(float target,float current);//計算の実行

    private:

    float _dt,_kp,_ki,_kd;
    float _min_out,_max_out;
    float i_limit;//積分のリミット
    Mode _mode;

    float integral;
    float pre_error;
    float pre2_error;
    float pre_mv;//操作量

};

#endif
