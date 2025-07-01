package org.firstinspires.ftc.teamcode.util.mechanisms;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;

import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;

import java.util.function.Supplier;

public class RobotActions {
    public static Action reachSample(Pose relative_sample, Intake intake, Follower follower){
        Vector movement_vector = new Vector(relative_sample.getY(), follower.getPose().getHeading()+PI/2);
        Point target_point = new Point(
                follower.getPose().getX() + movement_vector.getXComponent(),
                follower.getPose().getY() + movement_vector.getYComponent()
        );
        return new RaceAction(
            foreverAction(follower::update),
            new SequentialAction(
                new InstantAction(()->follower.followPath(
                    follower.pathBuilder()
                        .addPath(new Path(new BezierLine(
                            new Point(follower.getPose()),
                            target_point
                        )))
                        .setZeroPowerAccelerationMultiplier(5)
                        .setConstantHeadingInterpolation(follower.getPose().getHeading())
                        .build(),
                        true
                    )
                ),
                new InstantAction(() ->{
                        intake.claw.turret_angle = relative_sample.getHeading() + PI/2;
                        intake.readyGrab(intake.slide.trimIn(relative_sample.getX()));
                }),
                new RaceAction(
                    foreverAction(intake.slide::movementLoop),
                    new ParallelAction(triggerAction(()->(
                        !follower.isBusy() &&
                        follower.atPoint(target_point,1, 1) &&
                        intake.slide.within_error
                    )))
                )
            )
        );
    }
    public static Action fullTransferAction(Intake intake, Outtake outtake){
        return new SequentialAction(
                new ParallelAction(
                        intake.readyTransferAction(),
                        outtake.readyTransferAction(),
                        new InstantAction(outtake.claw::open)
                ),
                new InstantAction(outtake.claw::grab),
                new SleepAction(0.3),
                new InstantAction(intake.claw::open)
        );
    }
    public static Action specFullTransferAction(Intake intake, Outtake outtake){
        return new SequentialAction(
            new ParallelAction(
                intake.firmReadyTransferAction(),
                outtake.readySpecTransferAction()
            ),
            new InstantAction(outtake.claw::grab),
            new SleepAction(0.3),
            new InstantAction(intake.claw::open)
        );
    }
    public static Action pathAction(Follower follower, PathChain path){
        return pathAction(follower, path, () -> (!follower.isBusy())&&(follower.getVelocityMagnitude()<2)&&(follower.getHeadingError()<0.04));
    }

    public static Action pathAction(Follower follower, PathChain path, Supplier<Boolean> end_function){
        return new SequentialAction(
                new InstantAction(()->follower.followPath(path, true)),
                triggerAction(end_function)
        );
    }

    public static Action moveLineAction(Follower follower, Pose a, Pose b){
        return moveLineAction(follower, a, b, () -> (!follower.isBusy())&&(follower.getVelocityMagnitude()<2)&&(follower.getHeadingError()<0.04));
    }

    public static Action moveLineAction(Follower follower, Pose a, Pose b,  Supplier<Boolean> end_function) {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(a), new Point(b)))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
        return pathAction(follower, path, end_function);
    }
}
