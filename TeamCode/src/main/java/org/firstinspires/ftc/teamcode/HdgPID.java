package org.firstinspires.ftc.teamcode;

//This PID loop is only to be used with RADIANS

public class HdgPID extends PIDLoop{
    public HdgPID(){
        this(0, 0, 0, 0, -1, 1);
    }
    public HdgPID(double kp, double ki, double kd){
        this(kp, ki, kd, 0, -1, 1);
    }
    public HdgPID(double kp, double ki, double kd, double goal){
        this(kp, ki, kd, goal, -1, 1);
    }
    public HdgPID(double kp, double ki, double kd, double goal, double outMin, double outMax){

        super(kp, ki, kd, goal, outMin, outMax);
    }

    public double update(double input, double time){
        calculateP(input, time);
        super.calculateI(input, time);
        super.calculateD(input, time);
        return super.output();
    }

    public double calculateP(double input, double time){
        if(Math.abs(goal-input) < Math.min(Math.abs(goal + 2 * Math.PI - input), Math.abs(goal - 2 * Math.PI - input))){
            pTerm = goal-input;
        }else if(Math.abs(goal + 2 * Math.PI -input) < Math.min(Math.abs(goal - input), Math.abs(goal - 2 * Math.PI - input))){
            pTerm = goal + 2 * Math.PI - input;
        }else{
            pTerm = goal - 2 * Math.PI - input;
        }
        return pTerm;
    }

    //Method to move the goal by a certain amount
    //Uses include controlling heading goal with a joystick on a gamepad
    public void moveGoal(double goalChange, double time){
        goal += goalChange * (time - gPreTime);
        if(goal < -Math.PI){
            goal += 2 * Math.PI;
        }else if(goal > Math.PI){
            goal -= 2 * Math.PI;
        }
        gPreTime = time;
    }

}
