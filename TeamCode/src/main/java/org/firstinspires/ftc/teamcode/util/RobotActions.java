package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class RobotActions {
    public static Action reachSample(Pose2d relative_sample, Intake intake, MecanumDrive drive){
        return new SequentialAction(
            intake.readyGrabAction(GameMap.MinIntakeDistance, relative_sample.heading.toDouble()),
            drive.actionBuilder(drive.pose).turn(
                    new Rotation2d(relative_sample.position.y, relative_sample.position.x).toDouble()
            ).build(),
            new InstantAction( () -> intake.linear_slide.goTo( intake.linear_slide.inToTicks(
                Math.min(
                    Math.max(
                        GameMap.MinIntakeDistance,
                        relative_sample.position.norm() - 4
                    ),
                    GameMap.MaxIntakeDistance
                )
            ))),
            intake.linear_slide.loopUntilDone()
        );
    }
}
