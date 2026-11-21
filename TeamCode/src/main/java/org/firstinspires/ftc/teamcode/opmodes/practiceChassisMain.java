package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.math.Pose;
@TeleOp
public class practiceChassisMain extends OpMode {
    private Follower follower;
    private Pose startPose = new Pose(72,72,Math.toRadians(90));
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

    }
    @Override
    public void start(){

    }
    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        follower.manual(forward,lateral,turn);
        follower.update();

    }
}
