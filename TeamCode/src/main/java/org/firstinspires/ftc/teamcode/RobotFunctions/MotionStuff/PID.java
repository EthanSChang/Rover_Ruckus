package org.firstinspires.ftc.teamcode.RobotFunctions.MotionStuff;

/**
 * This class provides methods for a custom pid controller
 * This controller has the capability to limit integral build up, and maximum integral output
 * To disable a limit, set the limit to zero
 * Based off mini pid library (https://github.com/tekdemo/MiniPID-Java)
 *
 * TODO: maybe add pidva functionality
 * @author ethan
 */

public class PID {
    double Kp, Ki, Kd;
    double tgt, actual, lastActual;
    double Ilim = 0, outMin = 0, outMax = 0;
    double errorSum;
    boolean reversed;
    boolean firstRun = true;
    double output, error, Pout, Iout, Dout;

    public PID(double Kp, double Ki, double Kd, double ILimit, double OutputMin, double OutputMax){ //set limits to 0 for no limit
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        Ilim = ILimit; //set limits to 0 to disable
        outMin = OutputMin;
        outMax = OutputMax;
    }

    public PID(double Kp, double Ki, double Kd, double ILimit){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        Ilim = ILimit; //set limits to 0 to disable
    }

    public PID(double Kp, double Ki, double Kd, double OutputMin, double OutputMax){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        outMin = OutputMin;
        outMax = OutputMax;
    }

    public PID(){
    }

    public PID(double Kp, double Ki, double Kd){ //set limits to 0 for no limit
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void setP(double Kp){this.Kp = Kp;}

    public void setI(double Ki){this.Ki = Ki;}

    public void setD(double Kd){this.Kd = Kd;}

    public void setPID(double Kp, double Ki, double Kd){this.Kp = Kp; this.Ki = Ki; this.Kd = Kd;}

    public void setMaxI(double maxI){Ilim = maxI;}

    public void setOutputLimits(double maxOutput, double minOutput){outMax = maxOutput; outMin = minOutput;}

    public void reverse(boolean reversed){this.reversed = reversed;}

    public void setTarget(double Target){tgt = Target;}

    public double getError(){return error;}

    public double getOutput(double actual, double target){return target - actual;}


    private double time, preTime, deltaTime;
    public double getOutput(double error){
        this.error = error;
        time = System.currentTimeMillis() / 1000.0;
        deltaTime = time - preTime;
        error = tgt - actual;

        if(firstRun){
            lastActual = actual;
            firstRun = false;
        }

        Pout = Kp * error;

        Dout = -Kd * ((actual - lastActual) / deltaTime);

        Iout = Ki * errorSum;
        if(Ilim != 0){
            if(Iout > Ilim){
                Iout = Ilim;
            } else if(Iout < -Ilim){
                Iout = -Ilim;
            }
        }

        output = Pout + Iout + Dout;

        if(outMin != outMax && (output < outMin || output > outMax)){ //resets error sum if the output is too large
            errorSum = error;
        } else {
            errorSum += error;
        }

        if(outMin != outMax){
            if(output > outMax){
                output = outMax;
            } else if(output < outMin){
                output = outMin;
            }
        }

        lastActual = actual;

        preTime = time;

        return output;

    }

}
