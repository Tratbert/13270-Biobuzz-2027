package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class main extends OpMode {

    nectarPID nectars = new nectarPID();
    pollenPID pollens = new pollenPID();

    Pose startPose = new Pose(72, 72 , Math.toRadians(90));
    Servo shooterGate;

    Servo nectarTurret;
    Servo pollenTurret;
    double nectarOrPollen = 0.6;

    double x = 72;
    double y = 72;
    double heading = Math.toRadians(90);



    double pollenX = x + 7.81205*Math.cos(heading + Math.toRadians(140.194429));
    double pollenY = y + 7.81205*Math.sin(heading + Math.toRadians(140.194429));
    double nectarX = x + 6.800735*Math.cos(heading + Math.toRadians(216.0273734));
    double nectarY = y + 6.800735*Math.sin(heading + Math.toRadians(216.0273734));

    double distanceToGoalNectar = 0;
    double distanceToGoalPollen = 0;
    double nectarTargetVel = 0;
    double pollenTargetVel = 0;
    DcMotorEx nectarShooter;
    DcMotorEx pollenShooter;

    DcMotor sorter;
    DcMotor intake;

    private Follower follower;


    @Override
    public void init() {

        pollenShooter = hardwareMap.get(DcMotorEx.class, "pollenShooter");
        sorter = hardwareMap.get(DcMotor.class, "sorter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pollenTurret = hardwareMap.get(Servo.class, "turretPollen");
        shooterGate = hardwareMap.get(Servo.class, "shooterGate");
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void loop() {
        double pollenX = x + 7.81205*Math.cos(heading + Math.toRadians(140.194429));
        double pollenY = y + 7.81205*Math.sin(heading + Math.toRadians(140.194429));
        Pose robotPose = follower.pose();
        x = robotPose.x();
        y = robotPose.y();
        heading = robotPose.heading();
        telemetry.addData("X: ", x);
        telemetry.addData("Y: ", y);
        telemetry.addData("Heading: ", heading);
        distanceToGoalPollen = Math.sqrt(Math.pow((57.5-pollenX), 2) + Math.pow((52.5-pollenY), 2));
        telemetry.addData("Distance to Goal", Math.sqrt(Math.pow((57.5-x), 2) + Math.pow((52.5-y), 2)));
        pollenTargetVel= 0.346891 * distanceToGoalPollen * distanceToGoalPollen - 32.90777 * distanceToGoalPollen + 2195.717263;
        DrivePowers powers = ManualDrive.fieldCentric(
                gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );
        follower.manual(powers);
        follower.update();

        if(gamepad1.aWasPressed()){
            shooterGate.setPosition(0);
        }
        if(gamepad1.bWasPressed()){
            shooterGate.setPosition(1);
        }

        if(gamepad1.left_trigger > 0){
            pollenTurret.setPosition(0.5-0.005*gamepad1.left_trigger);
        }
        if(gamepad1.right_trigger > 0){
            pollenTurret.setPosition(0.5 + 0.005 * gamepad1.right_trigger);
        }
        if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
            pollenTurret.setPosition(0.5);
        }
        if(gamepad2.aWasPressed()){
            intake.setPower(-1 * nectarOrPollen);
        }
        if(gamepad2.yWasPressed()){
            intake.setPower(1 * nectarOrPollen);
        }
        if(gamepad2.bWasPressed()){
            intake.setPower(0);
        }
        if(gamepad2.dpadUpWasPressed()){
            sorter.setPower(1);
        }
        if(gamepad2.dpadDownWasPressed()){
            sorter.setPower(-1);
        }
        if(gamepad2.dpadLeftWasPressed()){
            sorter.setPower(0);
        }
        if(gamepad2.rightBumperWasPressed()){
            nectarOrPollen = 1;
        }
        if(gamepad2.rightBumperWasPressed()){
            nectarOrPollen = 0.6;
        }
        pollenShooter.setPower(pollens.pollenOut(pollenTargetVel, pollenShooter.getVelocity()));


    }

}
