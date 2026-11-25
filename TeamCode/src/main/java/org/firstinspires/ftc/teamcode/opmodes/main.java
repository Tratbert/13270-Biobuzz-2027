package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class main extends OpMode {

    nectarPID nectars = new nectarPID();
    pollenPID pollens = new pollenPID();

    Pose startPose = new Pose(72, 72 , Math.toRadians(90));
    Servo shooterGate;
    int shooterCounter = 0;

    double nectarkP = 0.0071;
    double nectarkI = 0.0002;
    double nectarkD = 0;
    double nectarkFriction = 0.0570;
    double pollenkP = 0.003;
    double pollenkI = 0.000;
    double pollenkD = 0;
    double pollenkFriction = 0.0450;
    Servo nectarTurret;
    AnalogInput nectarEncoder;
    Servo pollenTurret;
    AnalogInput pollenEncoder;

    Servo flowerSlope;
    int flowerSlopeIndicator = 0;
    double nectarOrPollen = 0.6;

    double x = 72;
    double y = 72;
    double heading = Math.toRadians(90);



    double pollenX = x + 7.81205*Math.cos(heading + Math.toRadians(140.194429));
    double pollenY = y + 7.81205*Math.sin(heading + Math.toRadians(140.194429));
    double nectarX = x + 6.800735*Math.cos(heading + Math.toRadians(216.0273734));
    double nectarY = y + 6.800735*Math.sin(heading + Math.toRadians(216.0273734));

    double targetX = 57.5;
    double targetY = 52.5;
    double distanceToGoalNectar = 0;
    double distanceToGoalPollen = 0;
    double nectarTargetVel = 0;
    double pollenTargetVel = 0;
    DcMotorEx nectarShooter;
    DcMotorEx pollenShooter;

    DcMotor sorter;
    DcMotor intake;
    double nectarIntegralSum = 0;
    double nectarLastError = 0;
    double nectarLastAngle = 0;
    double nectarMaxIntegralSum = 250;
    double nectarA = 1;
    double nectarPreviousEstimate = 0;
    double nectarCurrentEstimate = 0;
    double pollenIntegralSum = 0;
    double pollenLastError = 0;
    double pollenLastAngle = 0;
    double pollenMaxIntegralSum = 250;
    double pollenA = 1;
    double pollenPreviousEstimate = 0;
    double pollenCurrentEstimate = 0;

    private Follower follower;
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {

        pollenShooter = hardwareMap.get(DcMotorEx.class, "pollenShooter");
        pollenTurret = hardwareMap.get(Servo.class, "turretPollen");
        pollenEncoder = hardwareMap.get(AnalogInput.class, "pollenEncoder");
        nectarShooter = hardwareMap.get(DcMotorEx.class, "nectarShooter");
        nectarTurret = hardwareMap.get(Servo.class, "turretNectar");
        nectarEncoder = hardwareMap.get(AnalogInput.class, "nectarEncoder");
        sorter = hardwareMap.get(DcMotor.class, "sorter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterGate = hardwareMap.get(Servo.class, "shooterGate");
        flowerSlope = hardwareMap.get(Servo.class, "flowerSlope");
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void loop() {
        Pose robotPose = follower.pose();
        x = robotPose.x();
        y = robotPose.y();
        if(y > 72){
            targetY = 144-52.5;
        }
        if(y <= 72){
            targetY = 52.5;
        }
        heading = robotPose.heading();
        telemetry.addData("X: ", x);
        telemetry.addData("Y: ", y);
        telemetry.addData("Heading: ", heading);
        double pollenX = x + 7.81205*Math.cos(heading + Math.toRadians(140.194429));
        double pollenY = y + 7.81205*Math.sin(heading + Math.toRadians(140.194429));
        double nectarX = x + 6.5*Math.cos(heading + Math.toRadians(222));
        double nectarY = y + 6.5 * Math.sin(heading + Math.toRadians(222));
        double nectarXDiff = targetX-nectarX;
        double nectarYDiff = targetY-nectarY;
        double nectarToGoalRad = (Math.atan2(nectarYDiff,nectarXDiff)-heading);
        nectarToGoalRad = (nectarToGoalRad % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
        double targetNectarAngle = (Math.toDegrees(nectarToGoalRad))*-1 + 360;
        if(targetNectarAngle < 0){
            targetNectarAngle +=360;
        }
        double nectarVoltage = nectarEncoder.getVoltage();
        double nectarAngle = (nectarVoltage/3.300538)*360.0;
        double nectarError = targetNectarAngle-nectarAngle;
        double nectarErrorChange = nectarError-nectarLastError;
        nectarCurrentEstimate = (nectarA*nectarPreviousEstimate) + (1-nectarA) * nectarErrorChange;
        nectarPreviousEstimate = nectarCurrentEstimate;
        double nectarDerivitive = (nectarCurrentEstimate)/ timer.seconds();
        nectarIntegralSum = nectarIntegralSum+(nectarError* timer.seconds());
        if(nectarIntegralSum > nectarMaxIntegralSum){
            nectarIntegralSum = nectarMaxIntegralSum;
        }
        if(nectarIntegralSum < -nectarMaxIntegralSum){
            nectarIntegralSum = -nectarMaxIntegralSum;
        }

        double nectarOut = (nectarkP*nectarError) + (nectarkI*nectarIntegralSum) + (nectarkD* nectarDerivitive);
        telemetry.addData("Out" , nectarOut);
        if(Math.abs(nectarError) < 3){
            nectarkFriction = 0;
        } else {
            nectarkFriction = 0.057;
        }


        if(nectarOut < 0){
            nectarOut -= nectarkFriction;
        }
        if(nectarOut >= 0){
            nectarOut += nectarkFriction;
        }

        if(nectarOut < -0.5){
            nectarOut = -0.5;
        }
        if(nectarOut > 0.5){
            nectarOut = 0.5;
        }

        nectarTurret.setPosition(nectarOut +0.5);

        nectarLastError = nectarError;
        nectarLastAngle = nectarAngle;
        distanceToGoalPollen = Math.sqrt(Math.pow((57.5-pollenX), 2) + Math.pow((targetY-pollenY), 2));
        distanceToGoalNectar = Math.sqrt(Math.pow((57.5-nectarX), 2) + Math.pow((targetY-nectarY), 2));

        telemetry.addData("Distance to Goal", Math.sqrt(Math.pow((57.5-x), 2) + Math.pow((52.5-y), 2)));
        pollenTargetVel= 0.181509 * distanceToGoalPollen * distanceToGoalPollen - 13.53937 * distanceToGoalPollen + 1750.40352;
        nectarTargetVel = -0.13977 * distanceToGoalNectar * distanceToGoalNectar + 17.31034 * distanceToGoalNectar + 1028.93977;
        double pollenXDiff = targetX-pollenX;
        double pollenYDiff = targetY-pollenY;
        double pollenToGoalRad = Math.atan2(pollenYDiff, pollenXDiff) - heading;
        pollenToGoalRad = (pollenToGoalRad % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
        double targetPollenAngle = (Math.toDegrees(pollenToGoalRad))*-1 + 360;
        if(targetPollenAngle < 0){
            targetPollenAngle +=360;
        }
        double pollenVoltage = pollenEncoder.getVoltage();
        double pollenAngle = (pollenVoltage /3.300538)*360.0;
        double pollenError = targetPollenAngle -pollenAngle;
        double pollenErrorChange = pollenError-pollenLastError;
        pollenCurrentEstimate = (pollenA*pollenPreviousEstimate) + (1-pollenA) * pollenErrorChange;
        pollenPreviousEstimate = pollenCurrentEstimate;
        double pollenDerivitive = (pollenCurrentEstimate)/ timer.seconds();
        pollenIntegralSum = pollenIntegralSum+(pollenError* timer.seconds());
        if(pollenIntegralSum > pollenMaxIntegralSum){
            pollenIntegralSum = pollenMaxIntegralSum;
        }
        if(pollenIntegralSum < -pollenMaxIntegralSum){
            pollenIntegralSum = -pollenMaxIntegralSum;
        }

        double pollenOut = (pollenkP*pollenError) + (pollenkI*pollenIntegralSum) + (pollenkD* pollenDerivitive);
        telemetry.addData("pollenOut" , pollenOut);
        if(Math.abs(pollenError) < 3){
            pollenkFriction = 0;
        } else {
            pollenkFriction = 0.0450;
        }


        if(pollenOut < 0){
            pollenOut -= pollenkFriction;
        }
        if(pollenOut >= 0){
            pollenOut += pollenkFriction;
        }

        if(pollenOut < -0.5){
            pollenOut = -0.5;
        }
        if(pollenOut > 0.5){
            pollenOut = 0.5;
        }

        pollenTurret.setPosition(pollenOut +0.5);

        pollenLastError = pollenError;
        pollenLastAngle = pollenAngle;
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        follower.update();

        if(gamepad1.aWasPressed()){
            shooterGate.setPosition(shooterCounter%2);
            shooterCounter++;
        }
        if(gamepad1.left_trigger > 0){
            sorter.setPower(1);
            intake.setPower(1);
        }
        if(gamepad1.right_trigger > 0){
            sorter.setPower(-1);
            intake.setPower(-1);
        }
        if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
            sorter.setPower(0);
            intake.setPower(0);
        }
        if(gamepad1.bWasPressed()){
            flowerSlope.setPosition(flowerSlopeIndicator%2);
            flowerSlopeIndicator++;
        }



        nectarShooter.setPower(nectars.nectarOut(nectarTargetVel, nectarShooter.getVelocity()));
        pollenShooter.setPower(pollens.pollenOut(pollenTargetVel, pollenShooter.getVelocity()));
        timer.reset();

    }

}
