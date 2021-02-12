package org.firstinspires.ftc.teamcode;

public class TwoMDValue {
    private double leftRange;
    private double rightRange;
    private double frontRange;
    private double backRange;
    private double readTime;

    public TwoMDValue(double frontRange, double backRange, double leftRange, double rightRange, double readTime){
        this.leftRange = leftRange;
        this.rightRange = rightRange;
        this.frontRange = frontRange;
        this.backRange = backRange;
        this.readTime = readTime;
    }

    double getLeftRange(){
        return leftRange;
    }
    double getRightRange(){
        return  rightRange;
    }
    double getFrontRange(){
        return frontRange;
    }
    double getBackRange(){
        return backRange;
    }
    double getReadTime(){
        return readTime;
    }
}
