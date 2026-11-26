package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class soloAuto extends OpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(57,8.5, 180);
    private final Pose readyFlower4 = p.of(17, 46, 180);
    private final Pose intakeFlower4 = p.of(13.5,46, 180 );
    private final Pose readyFlower1 = p.of(43, 118, 90);
    private final Pose intakeFlower1 = p.of(43, 122.5, 90);
    private final Pose intakeNectar = p.of(12, 106.5,180);
    private final Pose intakeNectarControl = new Pose(45.67, 107.4);

    private final Pose readyToShootNectar = p.of(56.5, 18, 270);
    private final Pose readyToShootNectarControl = new Pose (85.41, 149.71);

    private final  Pose readyToIntakeGarden = p.of(46.5, 16, 180);
    private final Pose intakeGarden = p.of(12, 9, 180);
    private final Pose park = p.of(12, 93.5, 180);
    private final Pose parkControl = new Pose(46.15, 45.51);

    private Path toFlower4(){
        return line(startPose, readyFlower4).linear(startPose, readyFlower4);
    }

    private Path intakingFlower4(){
        return line(readyFlower4, intakeFlower4).linear(readyFlower4, intakeFlower4);
    }

    private Path toFlower1(){
        return line(intakeFlower4, readyFlower1).linear(intakeFlower4, readyFlower1);
    }

    private Path intakingFlower1(){
        return line(readyFlower1, intakeFlower1).linear(readyFlower1, intakeFlower1);
    }

    private Path intakingNectar(){
        return curve(intakeFlower1, intakeNectarControl, intakeNectar).linear(intakeFlower1, intakeNectar);
    }

    private Path shootingNectar(){
        return curve(intakeNectar, readyToShootNectarControl,readyToShootNectar).linear(intakeNectar,readyToShootNectar);
    }
    private Path readyIntakeGarden(){
        return line(readyToShootNectar, readyToIntakeGarden).linear(readyToShootNectar, readyToIntakeGarden);
    }
    private Path intakingGarden(){
        return line(readyToIntakeGarden, intakeGarden).linear(readyToIntakeGarden, intakeGarden);
    }
    private Path parking(){
        return curve(intakeGarden, parkControl, park).linear(intakeGarden, park);
    }
    private Command autoRoutine(){
        return sequential(
                follow(follower, toFlower4()),
                follow(follower, intakingFlower4()),
                follow(follower, toFlower1()),
                follow(follower, intakingFlower1()),
                follow(follower, intakingNectar()),
                follow(follower,shootingNectar()),
                follow(follower, readyIntakeGarden()),
                follow(follower, intakingGarden()),
                follow(follower, parking())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void start(){
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}
