package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.*;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.GameMap;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.PoseStorer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoRed", group = "auto")
public class AutoRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        // HW stuff
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        // same as meepmeep
        final Pose2d start_pose = new Pose2d(GameMap.NetRedCorner.plus(new Vector2d(24.5 + GameMap.RobotWidth / 2, GameMap.RobotLength / 2)), PI / 2);
        final Pose2d score_pose = new Pose2d(GameMap.NetRedCorner.plus(new Vector2d(17.5, 17.5)), PI / 4);
        final Vector2d inner_spike_mark_position = GameMap.SpikeMarkNeutralLeftInner.minus(new Vector2d(0, GameMap.MaxIntakeDistance - 2));
        final Vector2d neutral_spike_mark_position = GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(0, GameMap.MaxIntakeDistance - 2));
        final Vector2d outer_spike_mark_position = GameMap.SpikeMarkNeutralLeftOuter.minus(
                Rotation2d.fromDouble(2*PI/3).vec().times(GameMap.MaxIntakeDistance - 2)
        );
        final Vector2d InnerDistance = GameMap.SpikeMarkNeutralLeftInner.minus(inner_spike_mark_position);
        final Vector2d CenterDistance = GameMap.SpikeMarkNeutralLeftCenter.minus(neutral_spike_mark_position);
        final Vector2d OuterDistance = GameMap.SpikeMarkNeutralLeftOuter.minus(outer_spike_mark_position);
// this line ≠ meepmeep
        MecanumDrive drive = new MecanumDrive(hardwareMap, start_pose);
        // this line ≠ meepmeep
        Action path = drive.actionBuilder(start_pose)
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .stopAndAdd(new ParallelAction(outtake.scoreAction(), intake.readyGrabAction(InnerDistance.norm(), InnerDistance.angleCast().toDouble())))
                .afterTime(0, outtake.slideWaitAction())
                .setTangent(0)
                .strafeToSplineHeading(inner_spike_mark_position, InnerDistance.angleCast().toDouble(), (pose2dDual, posePath, v) -> 20)
                .stopAndAdd(intake.pickUpAction())
                .afterTime(0, intake.fullTransferAction())
                .setTangent(0)
                .strafeToSplineHeading(score_pose.position.minus(new Vector2d(0.5, 0.5)), PI/4 + 0.1)
                .stopAndAdd(new ParallelAction(outtake.scoreAction(), intake.readyGrabAction(CenterDistance.norm() - 2, CenterDistance.angleCast().toDouble())))
                .afterTime(0, outtake.slideWaitAction())
                .setTangent(0)
                .strafeToSplineHeading(neutral_spike_mark_position, CenterDistance.angleCast().toDouble(), (pose2dDual, posePath, v) -> 10)
                .stopAndAdd(intake.pickUpAction())
                .afterTime(0, intake.fullTransferAction())
                .setTangent(0)
                .strafeToSplineHeading(score_pose.position.minus(new Vector2d(1.5, 1.5)), PI/4 + 0.1)
                .stopAndAdd(new ParallelAction(outtake.scoreAction(), intake.readyGrabAction(OuterDistance.norm() - 2, InnerDistance.angleCast().toDouble())))
                .afterTime(0, outtake.slideWaitAction())
                .setTangent(0)
                .strafeToSplineHeading(outer_spike_mark_position, OuterDistance.angleCast().toDouble(), (pose2dDual, posePath, v) -> 20)
                .stopAndAdd(intake.pickUpAction())
                .afterTime(0, intake.fullTransferAction())
                .setTangent(0)
                .strafeToSplineHeading(score_pose.position.minus(new Vector2d(1, 1)), PI/4 + 0.1)
                .stopAndAdd(outtake.fullScoreAction())
                .build();
        // these lines ≠ meepmeepe
        waitForStart();
        Actions.runBlocking(path);
        PoseStorer.pose = drive.pose;
    }
}
