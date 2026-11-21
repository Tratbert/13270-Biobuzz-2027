package org.firstinspires.ftc.teamcode.opmodes;

public class nectarPID {

    double kP = 0.0007;
    double  kV = 0.0003;
    double kS = 0.1;

    public nectarPID(){}

    public double nectarOut (double targetVel, double curVel){
        double error = targetVel-curVel;
        double feedBack = error * kP;
        double feedForward = kV * targetVel + kS;
        return feedForward + feedBack;
    }
}
