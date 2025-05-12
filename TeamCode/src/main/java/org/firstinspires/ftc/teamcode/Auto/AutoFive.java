package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.GameMap.RobotLength;
import static org.firstinspires.ftc.teamcode.util.GameMap.RobotWidth;
import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.BulkReads;
import org.firstinspires.ftc.teamcode.util.GameMap;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.PoseStorer;
import org.firstinspires.ftc.teamcode.util.RobotActions;
import org.firstinspires.ftc.teamcode.util.customactions.ForeverAction;
import org.firstinspires.ftc.teamcode.util.customactions.FutureAction;
import org.firstinspires.ftc.teamcode.util.customactions.TriggerAction;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Autonomous(name = "Five", group = "auto")
public class AutoFive extends LinearOpMode {
    MecanumDrive drive;
    Intake intake;
    Outtake outtake;
    Vision vision;
    @Override
    public void runOpMode() {
        // HW stuff
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        vision = new Vision(telemetry, hardwareMap);
        // same as meepmeep
        final Vector2d inner_sample = new Vector2d(-48.75, -27);
        final Vector2d corner = new Vector2d(-72, -72);
        final Pose2d start_pose = new Pose2d(corner.plus(new Vector2d(24 + RobotLength / 2, RobotWidth/ 2)), 0);
        final Pose2d score_pose = new Pose2d(corner.plus(new Vector2d(18.5, 17)), PI / 4);
        final Vector2d inner_spike_mark_position = inner_sample.minus(new Vector2d(0, Intake.MaxDistance));
        final Vector2d neutral_spike_mark_position = inner_sample.minus(new Vector2d(10, 0)).minus(new Vector2d(0, Intake.MaxDistance - 0.5));
        final Vector2d outer_spike_mark_position = inner_sample.minus(new Vector2d(20, 0)).plus(
                Rotation2d.fromDouble(-0.95).vec().times(Intake.MaxDistance - 0.5)
        );
        final Vector2d inner_distance = inner_sample.minus(inner_spike_mark_position);
        final Vector2d center_distance = inner_sample.minus(new Vector2d(10, 0)).minus(neutral_spike_mark_position);
        final Vector2d outer_distance = inner_sample.minus(new Vector2d(20, 0)).minus(outer_spike_mark_position);
        Sample sample = new Sample();
        // this line ≠ meepmeep
        drive = new MecanumDrive(hardwareMap, start_pose);
        // this line ≠ meepmeep
        Action path = drive.actionBuilder(start_pose)
                .setTangent(0)
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .stopAndAdd(new ParallelAction(outtake.scoreAction(), intake.readyGrabAction(inner_distance.norm(), -inner_distance.angleCast().toDouble())))
                .setTangent(0)
                .afterTime(0, outtake.slideWaitAction())
                .strafeToSplineHeading(inner_spike_mark_position, inner_distance.angleCast().toDouble(), (pose2dDual, posePath, v) -> 10)
                .stopAndAdd(intake.pickUpAction())
                .setTangent(0)
                .afterTime(0, intake.fullTransferAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading, (pose2dDual, posePath, v) -> 12)
                .stopAndAdd(new ParallelAction(outtake.scoreAction(), intake.readyGrabAction(center_distance.norm(), -center_distance.angleCast().toDouble())))
                .setTangent(0)
                .afterTime(0, outtake.slideWaitAction())
                .strafeToSplineHeading(neutral_spike_mark_position, center_distance.angleCast().toDouble(), (pose2dDual, posePath, v) -> 7)
                .stopAndAdd(intake.pickUpAction())
                .setTangent(0)
                .afterTime(0, intake.fullTransferAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading,  (pose2dDual, posePath, v) -> 12)
                .stopAndAdd(new ParallelAction(outtake.scoreAction(), intake.readyGrabAction(outer_distance.norm(), -outer_distance.angleCast().toDouble())))
                .setTangent(0)
                .afterTime(0, outtake.slideWaitAction())
                .strafeToSplineHeading(outer_spike_mark_position, outer_distance.angleCast().toDouble(), (pose2dDual, posePath, v) -> 8)
                .stopAndAdd(intake.pickUpAction())
                .setTangent(0)
                .afterTime(0, intake.fullTransferAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading, (pose2dDual, posePath, v) -> 12)
                .stopAndAdd(outtake.scoreAction())
                .setTangent(0)
                .afterTime(0, outtake.slideWaitAction())
                .strafeToSplineHeading(new Vector2d(-48, -12), 0, (pose2dDual, posePath, v) -> 50)
                .setTangent(0)
                .splineTo(new Vector2d(-30, -12), 0, (pose2dDual, posePath, v) -> 50)
                .stopAndAdd( new SequentialAction(
                        vision.findSample(sample),
                        new FutureAction( () ->
                                RobotActions.reachSample(sample.pose, intake, drive)
                        ),
                        intake.pickUpAction(),
                        intake.fullTransferAction()
                ))
                .strafeToSplineHeading(new Vector2d(-48, -12), 0, (pose2dDual, posePath, v) -> 50)
                .strafeToSplineHeading(score_pose.position, score_pose.heading, (pose2dDual, posePath, v) -> 40)
                .stopAndAdd(outtake.scoreAction())
                .setTangent(0)
                .afterTime(0, outtake.slideWaitAction())
                .build();
        // these lines ≠ meepmeep
        BulkReads bulkreads = new BulkReads(hardwareMap);
        waitForStart();
        bulkreads.setCachingMode(LynxModule.BulkCachingMode.MANUAL);
        Actions.runBlocking(
                new ParallelAction(
                        path,
                        new ForeverAction(bulkreads::readManual),
                        new Action() {
                            double last_time = -1;
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                if (last_time == -1){
                                    last_time = time;
                                } else {
                                    telemetryPacket.addLine("Loop Time (ms): " + (last_time - time) * 1000);
                                    last_time = time;
                                }
                                return true;
                            }
                        }));
        PoseStorer.pose = drive.pose;
    }
}
