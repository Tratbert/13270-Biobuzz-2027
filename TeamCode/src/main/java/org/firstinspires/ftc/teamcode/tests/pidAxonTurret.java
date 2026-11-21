package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

@TeleOp
public class pidAxonTurret extends OpMode {

    Servo nectarTurret;
    AnalogInput nectarEncoder;
    double kp = 0.007;
    double ki = 0.0021;
    double kd = 0.0005;
    double kFriction = 0;
    double integralSum = 0;
    double lastError = 0;
    double lastAngle = 0;
    double[] steps = {0.00001,0.0001,0.001,0.01,0.1,1};
    int Step = 0;

    double maxIntegralSum = 250;
    double a = 1;
    double previousEstimate = 0;
    double currentEstimate = 0;



    ElapsedTime timer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();


    @Override
    public void init() {
        nectarTurret = hardwareMap.get(Servo.class, "turretPollen");
        nectarEncoder = hardwareMap.get(AnalogInput.class, "pollenEncoder");
    }


    @Override
    public void loop() {
        loopTimer.reset();

        if(gamepad1.rightBumperWasPressed()){
            Step++;
        }
        if(gamepad1.leftBumperWasPressed()){
            Step--;
        }
        if(gamepad1.yWasPressed()){
            kp += steps[Step%6];
        }
        if(gamepad1.aWasPressed()){
            kp -= steps[Step%6];
        }
        if(gamepad1.dpadUpWasPressed()){
            ki += steps[Step%6];
        }
        if(gamepad1.dpadDownWasPressed()){
            ki -= steps[Step%6];
        }
        if(gamepad1.dpadRightWasPressed()){
            kd += steps[Step%6];
        }
        if(gamepad1.dpadLeftWasPressed()){
            kd -= steps[Step%6];
        }
        if(gamepad1.xWasPressed()){
            kFriction += steps[Step%6];
        }
        if(gamepad1.bWasPressed()){
            kFriction-= steps[Step%6];
        }
        telemetry.addData("ki: ", ki);
        telemetry.addData("kp: ", kp);
        telemetry.addData("kd ", kd);
        telemetry.addData("a: ",a);
        telemetry.addData("Step: ", Step);
        telemetry.addData("Steps: ", steps[Step%6]);

        double voltage = nectarEncoder.getVoltage();

        double x = 72;
        double y = 72;

        double targetX = 96;
        double targetY = 96;

        double xDiff = targetX-x;
        double yDiff = targetY-y;

        double targetAngleRad = Math.atan2(yDiff,xDiff) - Math.PI;

        double targetAngleDeg = 180;

        double angle = (voltage/3.300538)*360.0;


        double error = targetAngleDeg-angle;
        double errorChange = (error - lastError);
        currentEstimate = (a*previousEstimate) + (1-a) * errorChange;
        previousEstimate = currentEstimate;
        double derivitive = (currentEstimate)/ timer.seconds();
        integralSum = integralSum+(error* timer.seconds());
        if(integralSum > maxIntegralSum){
            integralSum = maxIntegralSum;
        }
        if(integralSum < -maxIntegralSum){
            integralSum = -maxIntegralSum;
        }
//        if(angle != lastAngle){
//            integralSum = 0;
//        }
        double out = (kp*error) + (ki*integralSum) + (kd*derivitive);
        telemetry.addData("Out" , out);
        if(Math.abs(error) < 3){
            kFriction = 0;
        }

        if(out < 0){
            out -= kFriction;
        }
        if(out > 0){
            out += kFriction;
        }

        if(out < -0.5){
            out = -0.5;
        }
        if(out > 0.5){
            out = 0.5;
        }

        nectarTurret.setPosition(out+0.5);

        lastError = error;
        lastAngle = angle;

        timer.reset();

        telemetry.addData("Analog Voltage", "%.3f V", voltage);
        telemetry.addData("Estimated Angle", "%.1f deg", angle);
        telemetry.addData("Error: ", error);
        telemetry.addData("Target Angle: ", targetAngleDeg);
        telemetry.addData("Timer: ", timer.seconds());
        telemetry.addData("Integral Sum: ", integralSum);
        telemetry.addData("Derivative: ", derivitive);
        telemetry.addData("Loop Timer: ", loopTimer.milliseconds());
        telemetry.update();

    }
}
