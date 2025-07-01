package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.futureAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;
import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.fullTransferAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.moveLineAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.pathAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.reachSample;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
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
import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Claw;
import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.firstinspires.ftc.teamcode.vision.SampleLocationPipeline;
import org.firstinspires.ftc.teamcode.vision.Vision;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "AutoRed", group = "auto")
public class AutoRed extends LinearOpMode {
    Follower follower;
    Intake intake;
    Outtake outtake;
    Vision vision;
    Sample sample = new Sample();
    boolean found_sample = false;
    Action getSampleAction(){
        return new SequentialAction(
                new RaceAction(
                        vision.findSample(sample, (sample) -> (
                            sample.getFirst() + follower.getPose().getX() > (49 + GameMap.RobotWidth/2) &&
                            sample.getFirst() + follower.getPose().getX() < (72 - GameMap.RobotWidth/2)
                        )
                        ),
                        foreverAction(follower::update)
                ),
                futureAction(() -> reachSample(sample.pose, intake, follower)),
                new RaceAction(
                        new SequentialAction(
                                intake.pickUpAction(),
                                fullTransferAction(intake, outtake)
                        ),
                        foreverAction(follower::update)
                ),
                new InstantAction(() ->
                        found_sample = intake.claw.getSensedColor() != Claw.ColorSensorOut.NONE
                )
        );
    }
    @Override
    public void runOpMode() {
        vision = new Vision(telemetry, hardwareMap);
        SampleLocationPipeline.AllowedColors.yellow = true;
        SampleLocationPipeline.AllowedColors.blue = true;
        SampleLocationPipeline.AllowedColors.red = false;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        final Pose start_pose = new Pose(GameMap.RobotWidth/2, 120.5 - GameMap.RobotLength / 2, -PI/2);
        follower.setStartingPose(start_pose);

        final Pose inner_sample = new Pose(48-1.75, 121.75, 0);
        final Pose score_pose = new Pose(18.75,144-18.75, -PI / 4);
        final Pose inner_grab_pose = new Pose(inner_sample.getX() - Intake.MaxDistance - 1, inner_sample.getY(), 0);
        final Pose center_grab_pose = new Pose(inner_sample.getX() - Intake.MaxDistance - 1, inner_sample.getY() + 10, 0);
        final Pose outer_grab_pose = new Pose(inner_sample.getX() - 1.5 , inner_sample.getY() + 20, 0.7);
        final Pose submersible_pose = new Pose(72 - GameMap.RobotWidth/2, 100, -PI/2);
        final Vector outer_offset = new Vector(Intake.MaxDistance + 0.5, 0.7);
        outer_grab_pose.subtract(new Pose(outer_offset.getXComponent(), outer_offset.getYComponent(), 0));
        final PathChain score_to_sub = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(score_pose),
                new Point(72, 124),
                new Point(submersible_pose)
            ))
            .setLinearHeadingInterpolation(score_pose.getHeading(), submersible_pose.getHeading())
            .build();
        final PathChain sub_to_score = follower.pathBuilder()
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
                moveLineAction(follower, start_pose, score_pose, () -> follower.atPose(score_pose, 3, 3, 0.17)),
                outtake.readySampleAction()
            ),
            outtake.openClawAction(),
            new ParallelAction(
                outtake.readyTransferAction(),
            // Inner
                intake.readyGrabAction(Intake.MaxDistance - 0.5, 0),
                moveLineAction(follower, score_pose, inner_grab_pose)
            ),
            intake.pickUpAction(),
            new ParallelAction(
                new SequentialAction(
                    RobotActions.fullTransferAction(intake, outtake),
                    outtake.readySampleAction()
                ),
                moveLineAction(follower, inner_grab_pose, score_pose)
            ),
            new ParallelAction(
                outtake.scoreAction(),
            // Middle
                intake.readyGrabAction(Intake.MaxDistance - 0.5, 0)
            ),
            new ParallelAction(
                outtake.slideWaitAction(),
                moveLineAction(follower, score_pose, center_grab_pose)
            ),
            intake.pickUpAction(),
            new ParallelAction(
                new SequentialAction(
                        RobotActions.fullTransferAction(intake, outtake),
                        outtake.readySampleAction()
                ),
                moveLineAction(follower, center_grab_pose, score_pose)
            ),
            new ParallelAction(
                outtake.scoreAction(),
            // Outer
                intake.readyGrabAction(Intake.MaxDistance - 0.5, 0.7)
            ),
            new ParallelAction(
                outtake.slideWaitAction(),
                moveLineAction(follower, score_pose, outer_grab_pose)
            ),
            intake.pickUpAction(),
            new ParallelAction(
                new SequentialAction(
                        RobotActions.fullTransferAction(intake, outtake),
                        outtake.readySampleAction()
                ),
                moveLineAction(follower, outer_grab_pose, score_pose)
            ),
            outtake.scoreAction(),
            new ParallelAction(
                outtake.slideWaitAction(),
                // SUB 1
                pathAction(follower, follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Point(score_pose),
                                new Point(96, 124),
                                new Point(submersible_pose)
                        ))
                        .setLinearHeadingInterpolation(score_pose.getHeading(), submersible_pose.getHeading())
                        .build()
                )
            ),
            new RaceAction(
                triggerAction(() -> found_sample),
                foreverAction(this::getSampleAction)
            ),
            new InstantAction(() -> found_sample = false),
            new ParallelAction(
                outtake.readySampleAction(),
                pathAction(follower, sub_to_score)
            ),
            outtake.scoreAction(),
            new ParallelAction(
                    outtake.slideWaitAction(),
            // SUB 2
                    pathAction(follower, score_to_sub)
            ),
            new RaceAction(
                triggerAction(() -> found_sample),
                foreverAction(this::getSampleAction)
            ),
            new InstantAction(() -> found_sample = false),
            new ParallelAction(
                    outtake.readySampleAction(),
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
