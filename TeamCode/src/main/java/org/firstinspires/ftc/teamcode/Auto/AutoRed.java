package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.customactions.PathAction.pathAction;
import static org.firstinspires.ftc.teamcode.util.customactions.RunBlocking.runBlocking;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.BulkReads;
import org.firstinspires.ftc.teamcode.util.GameMap;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.PoseStorer;
import org.firstinspires.ftc.teamcode.util.RobotActions;
import org.firstinspires.ftc.teamcode.util.customactions.ForeverAction;
import org.firstinspires.ftc.teamcode.util.customactions.FutureAction;
import org.firstinspires.ftc.teamcode.util.customactions.PathAction;
import org.firstinspires.ftc.teamcode.util.customactions.TriggerAction;
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

    Action move_line_action(Pose a, Pose b) {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(a), new Point(b)))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
        return pathAction(follower, path);
    }
    @Override
    public void runOpMode() {
        vision = new Vision(telemetry, hardwareMap);
        Sample sample = new Sample();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        final Pose start_pose = new Pose(GameMap.RobotWidth/2, 120 - GameMap.RobotLength / 2, -PI/2);
        follower.setStartingPose(start_pose);

        final Pose inner_sample = new Pose(48-2.75, 121.5, 0);
        final Pose score_pose = new Pose(18,144-18, -PI / 4);
        final Pose inner_grab_pose = new Pose(inner_sample.getX() - Intake.MaxDistance - 0.75, inner_sample.getY() + 0.75, 0);
        final Pose center_grab_pose = new Pose(inner_sample.getX() - Intake.MaxDistance - 0.5, inner_sample.getY() + 9.25, 0);
        final Pose outer_grab_pose = new Pose(inner_sample.getX(), inner_sample.getY() + 20, 0.7);
        final Pose submersible_pose = new Pose(72, 100, -PI/2);
        PathChain submersible_path = follower.pathBuilder()
                .addPath(
                    new BezierCurve(
                            new Point(score_pose),
                            new Point(72, 124),
                            new Point(submersible_pose)
                    )
                ).setLinearHeadingInterpolation(score_pose.getHeading(), submersible_pose.getHeading())
                .build();
        PathChain sub_to_score = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(submersible_pose),
                                new Point(72, 124),
                                new Point(score_pose)
                        )
                ).setLinearHeadingInterpolation(submersible_pose.getHeading(), score_pose.getHeading())
                .build();
        final Vector outer_offset = new Vector(Intake.MaxDistance - 0.5, 0.7);

        // HW stuff
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        // same as meepmeep
        outer_grab_pose.subtract(new Pose(outer_offset.getXComponent(), outer_offset.getYComponent(), 0));
        Action path = new SequentialAction(
            new ParallelAction(
                move_line_action(start_pose, score_pose),
                outtake.slideUpAction(),
                outtake.slideWaitAction()
            ),
            new ParallelAction(
                outtake.scoreAction(),
                intake.readyGrabAction(Intake.MaxDistance - 0.5, PI/2)
            ),
            new ParallelAction(
                outtake.slideWaitAction(),
                move_line_action(score_pose, inner_grab_pose)
            ),
            intake.pickUpAction(),
            new ParallelAction(
                new SequentialAction(
                    intake.fullTransferAction(),
                    outtake.slideUpAction(),
                    outtake.slideWaitAction()
                ),
                move_line_action(inner_grab_pose, score_pose)
            ),
            new ParallelAction(
                outtake.scoreAction(),
                intake.readyGrabAction(Intake.MaxDistance - 0.5, PI/2)
            ),
            new ParallelAction(
                outtake.slideWaitAction(),
                move_line_action(score_pose, center_grab_pose)
            ),
            intake.pickUpAction(),
            new ParallelAction(
                new SequentialAction(
                        intake.fullTransferAction(),
                        outtake.slideUpAction(),
                        outtake.slideWaitAction()
                ),
                move_line_action(center_grab_pose, score_pose)
            ),
            new ParallelAction(
                outtake.scoreAction(),
                intake.readyGrabAction(Intake.MaxDistance - 0.5, 0.7)
            ),
            new ParallelAction(
                outtake.slideWaitAction(),
                move_line_action(score_pose, outer_grab_pose)
            ),
            intake.pickUpAction(),
            new ParallelAction(
                new SequentialAction(
                        intake.fullTransferAction(),
                        outtake.slideUpAction(),
                        outtake.slideWaitAction()
                ),
                move_line_action(outer_grab_pose, score_pose)
            ),
            outtake.scoreAction(),
            new ParallelAction(
                    outtake.slideWaitAction(),
                    pathAction(follower, submersible_path)
            ),
            new SequentialAction(
                    vision.findSample(sample),
                    new FutureAction( () ->
                            RobotActions.reachSample(sample.pose, intake, follower)
                    ),
                    new SleepAction(0.5),
                    intake.pickUpAction()
            ),
            new ParallelAction(
                    new SequentialAction(
                            intake.fullTransferAction(),
                            outtake.slideUpAction(),
                            outtake.slideWaitAction()
                    ),
                    pathAction(follower, sub_to_score)
            ),
            outtake.fullScoreAction()

        );
        // these lines ≠ meepmeep
        BulkReads bulkreads = new BulkReads(hardwareMap);
        waitForStart();
        bulkreads.setCachingMode(LynxModule.BulkCachingMode.MANUAL);
        while (!opModeIsActive());
        runBlocking(
                new RaceAction(
                        path,
                        new ForeverAction(bulkreads::readManual),
                        new ForeverAction(follower::update),
                        new TriggerAction(()->!opModeIsActive())
                )
        );
        PoseStorer.pose = follower.getPose();
    }
}
