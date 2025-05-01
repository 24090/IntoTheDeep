package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;

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
        final Pose2d score_pose = new Pose2d(GameMap.NetRedCorner.plus(new Vector2d(17, 17)), PI / 4);
        final Vector2d park_position = GameMap.ObservationRedCorner.plus(new Vector2d(-11.25, GameMap.RobotLength / 2 + 1));
        final Vector2d neutral_spike_mark_position = GameMap.SpikeMarkNeutralLeftInner.minus(new Vector2d(0, GameMap.MinIntakeDistance + 1));
        final Vector2d InnerDistance = GameMap.SpikeMarkNeutralLeftInner.minus(neutral_spike_mark_position);
        final Vector2d CenterDistance = GameMap.SpikeMarkNeutralLeftCenter.minus(neutral_spike_mark_position);
        final Vector2d OuterDistance = GameMap.SpikeMarkNeutralLeftOuter.minus(neutral_spike_mark_position);
        // this line ≠ meepmeep
        MecanumDrive drive = new MecanumDrive(hardwareMap, start_pose);
        // this line ≠ meepmeep
        Action path = drive.actionBuilder(start_pose)
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .stopAndAdd(new ParallelAction(outtake.scoreAction(), intake.readyGrabAction(InnerDistance.norm(), InnerDistance.angleCast().toDouble())))
                .afterTime(0, outtake.slide.loopUntilDone())
                .setTangent(0)
                .strafeToSplineHeading(neutral_spike_mark_position, InnerDistance.angleCast().toDouble(), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 20;
                    }})
                .stopAndAdd(new InstantAction(intake.claw::grab))
                .stopAndAdd(intake.fullTransferAction())
                .setTangent(0)
                .strafeToSplineHeading(score_pose.position.minus(new Vector2d(0.5, 0.5)), PI/4 + 0.1)
                .stopAndAdd(new ParallelAction(outtake.scoreAction(), intake.readyGrabAction(CenterDistance.norm() - 2, CenterDistance.angleCast().toDouble())))
                .afterTime(0, outtake.slide.loopUntilDone())
                .setTangent(0)
                .strafeToSplineHeading(neutral_spike_mark_position, CenterDistance.angleCast().toDouble(), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 10;
                    }})
                .stopAndAdd(new InstantAction(intake.claw::grab))
                .stopAndAdd(intake.fullTransferAction())
                .setTangent(0)
                .strafeToSplineHeading(score_pose.position.minus(new Vector2d(1.5, 1.5)), PI/4 + 0.1)
                .stopAndAdd(new ParallelAction(outtake.scoreAction(), intake.readyGrabAction(OuterDistance.norm() - 2, InnerDistance.angleCast().toDouble())))
                .afterTime(0, outtake.slide.loopUntilDone())
                .setTangent(0)
                .strafeToSplineHeading(neutral_spike_mark_position, OuterDistance.angleCast().toDouble(), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 20;
                    }}
                )
                .stopAndAdd(new InstantAction(intake.claw::grab))
                .stopAndAdd(intake.fullTransferAction())
                .setTangent(0)
                .strafeToSplineHeading(score_pose.position.minus(new Vector2d(1, 1)), PI/4 + 0.1)
                .stopAndAdd(outtake.fullScoreAction())
                .build();
        // these lines ≠ meepmeep
        waitForStart();
        Actions.runBlocking(path);
        PoseStorer.pose = drive.pose;
    }
}
