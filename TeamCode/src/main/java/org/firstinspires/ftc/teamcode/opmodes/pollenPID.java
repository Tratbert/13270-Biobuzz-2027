package org.firstinspires.ftc.teamcode.opmodes;

public class pollenPID {

    double kP = 0.0135;
    double kV = 0.002;
    double kS = 0.07;

    public pollenPID(){}

    public double pollenOut(double targetVel, double curVel){
        double error = targetVel - curVel;
        double feedBack = error*kP;
        double feedForward = kV * targetVel + kS;
        return feedForward + feedBack;
    }
}
