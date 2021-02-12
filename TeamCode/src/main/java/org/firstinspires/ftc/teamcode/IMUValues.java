package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class IMUValues {
    private Orientation angles;
    private Acceleration gravity;
    private AngularVelocity angularVelocity;
    private Position position;
    private double readTime;

    public IMUValues(Orientation angles, Acceleration gravity, AngularVelocity angularVelocity, Position position, double readTime){
        this.angles = angles;
        this.gravity = gravity;
        this.angularVelocity = angularVelocity;
        this.position = position;
        this.readTime = readTime;
    }
}
