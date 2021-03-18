package org.firstinspires.ftc.teamcode;

//This PID loop is only to be used with RADIANS
//This PID loop accounts for the fact that moving past PI will go to -PI
//This PID loop may not work as intended if refresh rate is low
public class HdgPID extends PIDLoop{
    protected PIDLoop turn = new PIDLoop(0.5, 2, 0.07);

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
        pTerm = calculateP(input, time);
        iTerm = calculateI(time);
        dTerm = calculateD(input, time);
        return output();
    }

    public double calculateP(double input, double time){
        if(Math.abs(goal-input) < Math.min(Math.abs(goal + 2 * Math.PI - input), Math.abs(goal - 2 * Math.PI - input))){
            return goal-input;
        }else if(Math.abs(goal + 2 * Math.PI -input) < Math.abs(goal - 2 * Math.PI - input)){
            return goal + 2 * Math.PI - input;
        }else{
            return goal - 2 * Math.PI - input;
        }
    }

    public double calculateD(double input, double time){
        double tempD = dTerm;
        if(time != dPreTime) {
            if (Math.abs((input - preInput) / (time - dPreTime)) < Math.min(Math.abs((input + 2 * Math.PI - preInput) / (time - dPreTime)), Math.abs((input - 2 * Math.PI - preInput) / (time - dPreTime)))) {
                tempD = -(input - preInput) / (time - dPreTime);
            }else if(Math.abs((input + 2 * Math.PI - preInput) / (time - dPreTime)) < Math.abs((input - 2 * Math.PI - preInput) / (time - dPreTime))){
                tempD = -(input + 2 * Math.PI - preInput) / (time - dPreTime);
            }else{
                tempD = -(input - 2 * Math.PI - preInput) / (time - dPreTime);
            }
        }
        preInput = input;
        dPreTime = time;
        return tempD;
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

    public double commandTurn(double input, double turnRate, double time){
        reset(time);
        goal = input;
        turn.goal = turnRate;
        return turn.update(this.calculateD(input, time), time);
    }
}
