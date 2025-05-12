package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class RobotActions {
    public static Action reachSample(Pose2d relative_sample, Intake intake, MecanumDrive drive){
        return new SequentialAction(
            drive.actionBuilder(drive.pose).turnTo(
                    drive.pose.heading.times(
                            new Rotation2d(relative_sample.position.x, relative_sample.position.y)
                    )
            ).build(),
            new ParallelAction(
                intake.readyGrabAction(GameMap.MinIntakeDistance, relative_sample.heading.toDouble()),
                new InstantAction( () -> intake.linear_slide.goTo( intake.linear_slide.inToTicks(
                        intake.linear_slide.trimIn(relative_sample.position.norm())
                ))),
                intake.linear_slide.loopUntilDone()
            )

        );
    }
}
