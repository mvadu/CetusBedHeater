//Copyright @ Adys Tech
//Author : mvadu@adystech.com

#include "PidPwm.h"
#include <driver/gpio.h>

PidPwm::PidPwm(uint8_t pwmPin, uint8_t channel, double freq, uint8_t resolution_bits, double target, PidParam param, CurrentValueFunction onCompute)
{
    _pwmChannel = channel;
    _pwmRes = resolution_bits;
    _pwmFrequency = freq;
    _pin = (gpio_num_t)pwmPin;
    _sp = target;
    _curValFun = onCompute;
    setTuningParams(param);

    gpio_pad_select_gpio(pwmPin);
    pinMode(pwmPin, OUTPUT);
    ledcAttachPin(pwmPin, _pwmChannel);
    ledcSetup(_pwmChannel, _pwmFrequency, _pwmRes);

    _outPwm = _minOut = 0;
    _maxOut = (((uint32_t)1) << _pwmRes) - 1;

    //store current value
    _lastVal = onCompute();

    _computeTimer = xTimerCreate(
        "computeTimer",
        pdMS_TO_TICKS(_computeInterval),
        pdTRUE,
        (void *)this,
        computeCallback);
    _running = false;
}

void PidPwm::computeCallback(TimerHandle_t xTimer)
{
    yield();
    PidPwm *pid = (PidPwm *)pvTimerGetTimerID(xTimer);
    uint32_t ts = millis();
    double current = pid->_curValFun();
    //make sure the current value function is fast enough. If not expand the interval with 100ms buffer.
    if (millis() - ts > pid->_computeInterval)
    {
        pid->setComputeInterval((millis() - ts) + 100);
    }
    if (!isnan(current))
    {
        double dVal = (current - pid->_lastVal);
        double error = pid->_sp - current;

        pid->_lastVal = current;
        //over time accumulated error
        pid->_intgErrSum += pid->_ki * error;
        pid->_intgErrSum -= pid->_kp * dVal;
        
        //restrict the error to be within the maximum swing we can have with PWM out
        if (pid->_intgErrSum > pid->_maxOut)
        {
            pid->_intgErrSum = pid->_maxOut;
        }
        else if (pid->_intgErrSum < pid->_minOut)
            pid->_intgErrSum = pid->_minOut;

        /*Compute PID Output*/
        pid->_outPwm = pid->_intgErrSum - pid->_kd * dVal;
       
        //store the last successful outcome of output
        if (dVal != 0)
        {
            pid->_lastValTs = ts;
        }
        //restrict the output to be within the maximum swing we can have with PWM out
        if (pid->_outPwm > pid->_maxOut)
            pid->_outPwm = pid->_maxOut;
        else if (pid->_outPwm < pid->_minOut)
            pid->_outPwm = pid->_minOut;
        // Serial.printf("S:%.2f, C:%.2f, O:%.3f, error:%.3f, Changes dVal:%.3f, intgErrSum:%.2f, Param kp:%.2f, ki:%.2f, kd:%.2f \n",
        //               pid->_sp, current, pid->getOutputDS(), error, dVal, pid->_intgErrSum,
        //               pid->_kp, pid->_ki, pid->_kd);
        //detect stall or sensor failure conditions.
        //with at least 10% output, if no change is detected in current value over 10 sample periods shut down the output
        if (pid->getOutputDS() > 0.1 && ts - pid->_lastValTs > 100 * pid->getSamplePeriod())
        {
            ledcWrite(pid->_pwmChannel, pid->_minOut);
            return;
        }
        ledcWrite(pid->_pwmChannel, pid->_outPwm);
    }
    //just to be safe, if we can't measure current value, shut off output
    else
    {
        ledcWrite(pid->_pwmChannel, pid->_minOut);
    }
}

void PidPwm::setTarget(double target)
{
    _sp = target;
}

PidPwm::PidParam PidPwm::getTuningParams()
{
    return _param;
}

void PidPwm::setTuningParams(PidParam param)
{
    double SampleTimeInSec = ((double)_computeInterval) / 1000;
    _param = param;
    _kp = param.proportionalGain;
    _ki = param.integralGain * SampleTimeInSec;
    _kd = param.derivativeGain / SampleTimeInSec;
}

void PidPwm::setLimits(uint32_t min, uint32_t max)
{
    _minOut = min;
    _maxOut = max;
}

void PidPwm::setLimits(double minDS, double maxDS)
{
    _minOut = ((((uint32_t)1) << _pwmRes) - 1) * minDS;
    _maxOut = ((((uint32_t)1) << _pwmRes) - 1) * maxDS;
}

unsigned long PidPwm::getSamplePeriod()
{
    return _computeInterval;
}

double PidPwm::getTarget()
{
    return _sp;
}

double PidPwm::getCurrent()
{
    return _lastVal;
}

uint32_t PidPwm::getOutput()
{
    return _outPwm;
}

double PidPwm::getOutputDS()
{
    return (((double)_outPwm) / ((((uint32_t)1) << _pwmRes) - 1));
}

void PidPwm::setComputeInterval(unsigned long ts)
{
    if (ts > 0)
    {
        double ratio = (double)ts / (double)_computeInterval;
        _ki *= ratio;
        _kd /= ratio;
        _computeInterval = ts;
        xTimerChangePeriod(_computeTimer,
                           pdMS_TO_TICKS(_computeInterval),
                           10);
    }
}

void PidPwm::shutdown()
{
    if (isRunning())
    {
        xTimerStop(_computeTimer, 0);
        ledcWrite(_pwmChannel, 0);
        digitalWrite(_pwmChannel, LOW);
        _running = false;
    }
}
void PidPwm::begin()
{
    if (!isRunning())
    {
        xTimerStart(_computeTimer, 100);
        _running = true;
        _lastValTs = millis();
        _intgErrSum = 0;
        _outPwm = 0;
        _lastVal = 0;
    }
}

bool PidPwm::isRunning()

{
    return _running;
}
//////////////////////////////////////////////////////////////