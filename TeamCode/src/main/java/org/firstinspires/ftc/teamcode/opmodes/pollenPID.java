package org.firstinspires.ftc.teamcode.opmodes;

public class pollenPID {

    double kP = 0.0072;
    double kV = 0.0002;
    double kS = 0.1;

    public pollenPID(){}

    public double pollenOut(double targetVel, double curVel){
        double error = targetVel - curVel;
        double feedBack = error*kP;
        double feedForward = kV * targetVel + kS;
        return feedForward + feedBack;
    }
}
