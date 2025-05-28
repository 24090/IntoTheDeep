package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;

import org.firstinspires.ftc.teamcode.util.customactions.ForeverAction;
import org.firstinspires.ftc.teamcode.util.customactions.TriggerAction;

public class RobotActions {
    public static Action reachSample(Pose relative_sample, Intake intake, Follower follower){
        Pose pose2 = follower.getPose();
        Vector vector = new Vector(new Point(relative_sample.getX(), relative_sample.getY() + 0.75));
        pose2.add(new Pose(0,0, vector.getTheta()));
        return new RaceAction( new SequentialAction(
            new InstantAction( () ->
                follower.turn(vector.getTheta(), true)
            ),
            new TriggerAction(()->(
                    (!follower.isBusy())
                    &&(follower.getVelocityMagnitude()<1)
                    &&(follower.getHeadingError()<0.02)
            )),
            new ParallelAction(
                intake.readyGrabAction(Intake.MinDistance, relative_sample.getHeading() - vector.getTheta()),
                new InstantAction( () -> intake.linear_slide.goTo( intake.linear_slide.inToTicks(
                        intake.linear_slide.trimIn(vector.getMagnitude())
                ))),
                intake.linear_slide.loopUntilDone()
            )

        ), new ForeverAction(follower::update));
    }
    public static Action fullTransferAction(Intake intake, Outtake outtake){
        return new SequentialAction(
                new ParallelAction(
                        intake.readyTransferAction(),
                        new InstantAction(outtake::standby),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.4),
                                        new InstantAction(intake.claw::open)
                                ),
                                new SequentialAction(
                                        new SleepAction(0.4),
                                        outtake.readyTransferAction()
                                )
                        ),
                        new SequentialAction(
                                new SleepAction(0.4),
                                new InstantAction(outtake.claw::grab)
                        )
        ));
    }
}
