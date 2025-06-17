package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.customactions.PathAction.pathAction;

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
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;

import org.firstinspires.ftc.teamcode.util.customactions.ForeverAction;
import org.firstinspires.ftc.teamcode.util.customactions.TriggerAction;

public class RobotActions {
    public static Action reachSample(Pose relative_sample, Intake intake, Follower follower){
        return new RaceAction(
            new ForeverAction(follower::update),
            new SequentialAction(
                pathAction( follower,
                    follower.pathBuilder()
                        .addPath(new Path(new BezierPoint(follower.getPose())))
                        .setConstantHeadingInterpolation(
                            follower.getPose().getHeading()
                          + relative_sample.getVector().getTheta()
                        )
                        .build()
                ),
                new InstantAction( () ->
                    intake.linear_slide.goTo(
                        intake.linear_slide.inToTicks(
                            intake.linear_slide.trimIn(relative_sample.getVector().getMagnitude())
                        )
                    )
                ),
                intake.readyGrabAction(
                        Intake.MinDistance,
                        relative_sample.getHeading() - relative_sample.getVector().getTheta()
                ),
                intake.linear_slide.loopUntilDone()
            )
        );
    }
    public static Action fullTransferAction(Intake intake, Outtake outtake){
        return new SequentialAction(
                new ParallelAction(
                        intake.readyTransferAction(),
                        new InstantAction(outtake::standby)
                ),
                new SleepAction(0.4),
                outtake.readyTransferAction(),
                new InstantAction(outtake.claw::grab),
                new SleepAction(0.3),
                new InstantAction(intake.claw::open)
        );
    }
}
