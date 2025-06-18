package org.firstinspires.ftc.teamcode.util.customactions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;

import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;

public class RobotActions {
    public static Action reachSample(Pose relative_sample, Intake intake, Follower follower){
        return new RaceAction(
            new ForeverAction(follower::update),
            new SequentialAction(
                new InstantAction(()->follower.followPath(
                    follower.pathBuilder()
                        .addPath(new Path(new BezierPoint(follower.getPose())))
                        .setConstantHeadingInterpolation(
                            follower.getPose().getHeading()
                          + relative_sample.getVector().getTheta()
                        )
                        .build(),
                        true
                    )
                ),
                intake.readyGrabAction(
                        intake.linear_slide.trimIn(relative_sample.getVector().getMagnitude() + 2),
                        relative_sample.getHeading() - relative_sample.getVector().getTheta()
                )
            )
        );
    }
    public static Action fullTransferAction(Intake intake, Outtake outtake){
        return new SequentialAction(
                new ParallelAction(
                        intake.readyTransferAction(),
                        outtake.readyTransferAction()
                ),
                new InstantAction(outtake.claw::grab),
                new SleepAction(0.3),
                new InstantAction(intake.claw::open)
        );
    }
}
