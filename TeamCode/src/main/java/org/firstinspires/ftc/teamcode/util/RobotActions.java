package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;

public class RobotActions {
    public static Action reachSample(Pose2d relative_sample, Intake intake, Follower follower){
        return new SequentialAction(
            drive.actionBuilder(drive.pose).turnTo(
                    drive.pose.heading.times(
                            new Rotation2d(relative_sample.position.x, relative_sample.position.y)
                    )
            ).build(),
            new ParallelAction(
                intake.readyGrabAction(Intake.MinDistance, relative_sample.heading.toDouble()),
                new InstantAction( () -> intake.linear_slide.goTo( intake.linear_slide.inToTicks(
                        intake.linear_slide.trimIn(relative_sample.position.norm())
                ))),
                intake.linear_slide.loopUntilDone()
            )

        );
    }
}
