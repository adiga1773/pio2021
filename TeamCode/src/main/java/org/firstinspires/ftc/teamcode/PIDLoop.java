package org.firstinspires.ftc.teamcode;

public class PIDLoop {
    public double kp, ki, kd, goal, outMin, outMax, minError = 0;
    protected double pTerm, iTerm, dTerm, preInput = 0, iPreTime = 0, dPreTime = 0, gPreTime = 0;

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

    //Use this to get results
    public double update(double input, double time){
        calculateP(input, time);
        calculateI(input, time);
        calculateD(input, time);
        return this.output();
    }

    public double calculateP(double input, double time){
        pTerm = goal - input;
        if(Math.abs(pTerm) < minError){
            pTerm = 0;
        }
        return pTerm;
    }

    public double calculateI(double input, double time){
        if(pTerm * kp > outMax || pTerm * kp < outMin){
            iTerm = 0;
        } else {
            iTerm = Math.min(outMax, Math.max(outMin, ki * (iTerm + pTerm * (time - iPreTime)))) / ki;
        }
        iPreTime = time;
        return iTerm;
    }

    public double calculateD(double input, double time){
        if(time != dPreTime) {
            dTerm = -(input - preInput) / (time - dPreTime);
        }
        preInput = input;
        dPreTime = time;
        return dTerm;
    }

    //Use this to get previous results
    public double output(){
        return Math.min(outMax, Math.max(outMin, kp * pTerm + ki * iTerm + kd * dTerm));
    }

    //Method to move the goal by a certain amount
    //Uses include controlling goal with a joystick on a gamepad
    public void moveGoal(double goalChange, double time){
        goal += goalChange * (time - gPreTime);
        gPreTime = time;
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
