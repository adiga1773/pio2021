package org.firstinspires.ftc.teamcode;

public class PIDLoop {
    public double kp, ki, kd, goal, outMin, outMax, minError = 0;
    private double pTerm, iTerm, dTerm, preInput = 0, preTime = 0;

    //CONSTRUCTORS
    public PIDLoop(){
        this(0, 0, 0, 0, -1, 1);
    }
    public PIDLoop(double kp, double ki, double kd){
        this(kp, ki, kd, 0, -1, 1);
    }
    public PIDLoop(double kp, double ki, double kd, double goal){
        this(kp, ki, kd, goal, -1, 1);
    }
    public PIDLoop(double kp, double ki, double kd, double goal, double outMin, double outMax){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.goal = goal;
        this.outMin = outMin;
        this.outMax = outMax;
    }

    public void reset(){
        iTerm = 0;
    }

    public double update(double input, double time){
        if(Math.abs(pTerm) <= minError){
            pTerm = 0;
        }else{
            pTerm = goal - input;
        }
        if(pTerm * kp > outMax || pTerm * kp < outMin){
            iTerm = 0;
        } else {
            iTerm = Math.min(outMax, Math.max(outMin, ki * (iTerm + pTerm * (time - preTime)))) / ki;
        }
        if(time != preTime) {
            dTerm = -(input - preInput) / (time - preTime);
        }
        preInput = input;
        preTime = time;
        return this.output();
    }

    public double output(){
        return Math.min(outMax, Math.max(outMin, kp * pTerm + ki * iTerm + kd * dTerm));
    }
    public double p(){
        return pTerm;
    }
    public double i(){
        return iTerm;
    }
    public double d(){
        return dTerm;
    }
}
