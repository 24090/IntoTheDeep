package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.futureAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;
import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.pathAction;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.BulkReads;
import org.firstinspires.ftc.teamcode.util.GameMap;
import org.firstinspires.ftc.teamcode.util.PoseStorer;
import org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions;
import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.firstinspires.ftc.teamcode.vision.Vision;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "AutoRed", group = "auto")
public class AutoRed extends LinearOpMode {
    Follower follower;
    Intake intake;
    Outtake outtake;
    Vision vision;

    Action moveLineAction(Pose a, Pose b) {
        return RobotActions.moveLineAction(follower, a, b);
    }
    @Override
    public void runOpMode() {
        vision = new Vision(telemetry, hardwareMap);
        Sample sample = new Sample();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        final Pose start_pose = new Pose(GameMap.RobotWidth/2, 120.5 - GameMap.RobotLength / 2, -PI/2);
        follower.setStartingPose(start_pose);

        final Pose inner_sample = new Pose(48-1.75, 121.75, 0);
        final Pose score_pose = new Pose(19.25,144-19.25, -PI / 4);
        final Pose inner_grab_pose = new Pose(inner_sample.getX() - Intake.MaxDistance + 1.5, inner_sample.getY() + 0.75, 0);
        final Pose center_grab_pose = new Pose(inner_sample.getX() - Intake.MaxDistance + 1.5, inner_sample.getY() + 10, 0);
        final Pose outer_grab_pose = new Pose(inner_sample.getX(), inner_sample.getY() + 20.5, 0.7);
        final Pose submersible_pose = new Pose(72 - GameMap.RobotWidth/2, 100, -PI/2);
        final Vector outer_offset = new Vector(Intake.MaxDistance + 0.5, 0.7);
        outer_grab_pose.subtract(new Pose(outer_offset.getXComponent(), outer_offset.getYComponent(), 0));
        PathChain submersible_path = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Point(score_pose),
                    new Point(72, 124),
                    new Point(submersible_pose)
                ))
                .setLinearHeadingInterpolation(score_pose.getHeading(), submersible_pose.getHeading())
                .build();
        PathChain sub_to_score = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Point(submersible_pose),
                    new Point(72, 124),
                    new Point(score_pose)
                )).setLinearHeadingInterpolation(submersible_pose.getHeading(), score_pose.getHeading())
                .build();
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        // real init
        outtake.readyTransfer();
        outtake.claw.grab();
        // Path Init
        Action path = new SequentialAction(
            // Preload
            new ParallelAction(
                moveLineAction(start_pose, score_pose),
                outtake.readySampleAction()
            ),
            new ParallelAction(
                outtake.scoreAction(),
            // Inner
                intake.readyGrabAction(Intake.MaxDistance - 0.5, 0)
            ),
            new ParallelAction(
                outtake.slideWaitAction(),
                moveLineAction(score_pose, inner_grab_pose)
            ),
            intake.pickUpAction(),
            new ParallelAction(
                new SequentialAction(
                    RobotActions.fullTransferAction(intake, outtake),
                    outtake.readySampleAction()
                ),
                moveLineAction(inner_grab_pose, score_pose)
            ),
            new ParallelAction(
                outtake.scoreAction(),
            // Middle
                intake.readyGrabAction(Intake.MaxDistance - 0.5, 0)
            ),
            new ParallelAction(
                outtake.slideWaitAction(),
                moveLineAction(score_pose, center_grab_pose)
            ),
            intake.pickUpAction(),
            new ParallelAction(
                new SequentialAction(
                        RobotActions.fullTransferAction(intake, outtake),
                        outtake.readySampleAction()
                ),
                moveLineAction(center_grab_pose, score_pose)
            ),
            new ParallelAction(
                outtake.scoreAction(),
            // Outer
                intake.readyGrabAction(Intake.MaxDistance - 0.5, 0.7)
            ),
            new ParallelAction(
                outtake.slideWaitAction(),
                moveLineAction(score_pose, outer_grab_pose)
            ),
            intake.pickUpAction(),
            new ParallelAction(
                new SequentialAction(
                        RobotActions.fullTransferAction(intake, outtake),
                        outtake.readySampleAction()
                ),
                moveLineAction(outer_grab_pose, score_pose)
            ),
            outtake.scoreAction(),
            new ParallelAction(
                outtake.slideWaitAction(),
                // SUB 1
                pathAction(follower, submersible_path)
            ),
            new SequentialAction(
                vision.findSample(sample),
                futureAction( () ->
                    RobotActions.reachSample(sample.pose, intake, follower)
                ),
                new SleepAction(0.5),
                intake.pickUpAction()
            ),
            new ParallelAction(
                new SequentialAction(
                    RobotActions.fullTransferAction(intake, outtake),
                    outtake.readySampleAction()
                ),
                pathAction(follower, sub_to_score)
            ),
            outtake.scoreAction(),
            new ParallelAction(
                    outtake.slideWaitAction(),
            // SUB 2
                    pathAction(follower, submersible_path)
            ),
            new SequentialAction(
                vision.findSample(sample),
                futureAction( () ->
                    RobotActions.reachSample(sample.pose, intake, follower)
                ),
                new SleepAction(0.5),
                intake.pickUpAction()
            ),
            new ParallelAction(
                new SequentialAction(
                    RobotActions.fullTransferAction(intake, outtake),
                    outtake.readySampleAction()
                ),
                pathAction(follower, sub_to_score)
            ),
            outtake.fullScoreAction()

        );
        BulkReads bulkreads = new BulkReads(hardwareMap);
        waitForStart();
        bulkreads.setCachingMode(LynxModule.BulkCachingMode.MANUAL);
        runBlocking(
                new RaceAction(
                        path,
                        foreverAction(bulkreads::readManual),
                        foreverAction(follower::update),
                        triggerAction(()->!opModeIsActive())
                )
        );
        while (opModeIsActive());
        PoseStorer.pose = follower.getPose();
    }
}
