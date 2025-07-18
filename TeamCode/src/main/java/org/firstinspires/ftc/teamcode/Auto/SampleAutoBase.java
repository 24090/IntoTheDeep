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
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
public abstract class SampleAutoBase extends LinearOpMode {
    Follower follower;
    Intake intake;
    Outtake outtake;
    Vision vision;
    Sample sample = new Sample();
    boolean found_sample = false;

    boolean yellow;
    boolean red;
    boolean blue;
    boolean strict_boundary;
    abstract void setParams();


    boolean clawCheck() {
        Claw.ColorSensorOut color = intake.claw.getSensedColor();
        return (color == Claw.ColorSensorOut.BLUE && blue) || (color == Claw.ColorSensorOut.YELLOW && yellow) || (color == Claw.ColorSensorOut.RED && red);
    }
    Action getSampleAction(){
        return new SequentialAction(
            new RaceAction(
                vision.findSample(sample, (sample) -> (
                    sample.getFirst() + follower.getPose().getX() > (46.5 + GameMap.RobotWidth/2) &&
                    (sample.getFirst() + follower.getPose().getX() < (72 - GameMap.RobotWidth/2) || !strict_boundary) &&
                    follower.getPose().getY() - sample.getSecond() < 86
                )),
                foreverAction(() -> strict_boundary ?
                    new SequentialAction(
                        new SleepAction(2),
                        new InstantAction(intake.sweeper::moveOut),
                        new SleepAction(0.5),
                        new InstantAction(intake.sweeper::moveIn),
                        new SleepAction(2)
                    )
                    :
                    new SequentialAction(
                        new SleepAction(2),
                        new InstantAction(intake.sweeper::moveOut),
                        new SleepAction(0.5),
                        new InstantAction(intake.sweeper::moveIn),
                        new SleepAction(1.5),
                        moveLineAction(
                            follower,
                            follower.getPose(),
                            new Pose(
                                follower.getPose().getX() + GameMap.RobotWidth/2,
                                follower.getPose().getY(),
                                follower.getPose().getHeading())
                        ),
                        new SleepAction(2)
                    )
                ),
                foreverAction(follower::update)
            ),
            new InstantAction(intake.sweeper::moveIn),
        futureAction(() -> reachSample(sample.pose, intake, follower)),
                new RaceAction(
                    new SequentialAction(
                        intake.pickUpAction(),
                        futureAction(() -> new SleepAction(
                                Math.abs((sample.pose.getHeading()%(PI) + PI)%(PI) - PI/2) < PI/4? 0.6: 0.3
                        )),
                        new InstantAction(outtake.claw::open),
                        new RaceAction(
                            foreverAction(() -> {
                                telemetry.addData("intake", intake.slide.getPosition());
                                telemetry.addData("outtake", outtake.slide.getPosition());
                                telemetry.update();
                            }),
                            fullTransferAction(intake, outtake)
                        )
                    ),
                    foreverAction(follower::update)
                ),
                new InstantAction(() ->
                        found_sample = clawCheck()
                )
        );
    }
    @Override
    public void runOpMode() {
        setParams();
        vision = new Vision(telemetry, hardwareMap);
        SampleLocationPipeline.AllowedColors.yellow = yellow;
        SampleLocationPipeline.AllowedColors.blue = blue;
        SampleLocationPipeline.AllowedColors.red = red;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        final Pose start_pose = new Pose(GameMap.RobotWidth/2, 120.5 - GameMap.RobotLength / 2, -PI/2);
        follower.setStartingPose(start_pose);

        final Pose inner_sample = new Pose(48-2.75, 122.75, 0);
        final Pose score_pose = new Pose(18.75,144-18.75, -PI / 4);
        final Pose inner_grab_pose = new Pose(inner_sample.getX() - Intake.MaxDistance + 1.5, inner_sample.getY() + 0, 0);
        final Pose center_grab_pose = new Pose(inner_sample.getX() - Intake.MaxDistance, inner_sample.getY() + 10 - 0.5, 0);
        final Pose outer_grab_pose = new Pose(inner_sample.getX() , inner_sample.getY() + 20 + 1, 0.7);
        final Pose submersible_pose = new Pose(72 - GameMap.RobotWidth/2, 88 + GameMap.RobotLength/2, -PI/2);
        final Vector outer_offset = new Vector(Intake.MaxDistance, 0.7);
        outer_grab_pose.subtract(new Pose(outer_offset.getXComponent(), outer_offset.getYComponent(), 0));
        final PathChain score_to_sub = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(score_pose),
                new Point(submersible_pose.getX() - 15, score_pose.getY() + 30),
                new Point(submersible_pose)
            ))
            .setLinearHeadingInterpolation(score_pose.getHeading(), submersible_pose.getHeading())
            .build();
        final PathChain sub_to_score = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(submersible_pose),
                new Point(submersible_pose.getX(), score_pose.getY()),
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
                        moveLineAction(follower, start_pose, new Pose(score_pose.getX(), score_pose.getY(), score_pose.getHeading()), () -> follower.atPose(score_pose, 4, 4, 0.17)),
                        outtake.readySampleAction()
                ),
                outtake.openClawAction(),
                new ParallelAction(
                        outtake.readyTransferAction(),
                        // Inner
                        intake.readyGrabAction(Intake.MaxDistance - 0.5, 0),
                        new RaceAction(
                            moveLineAction(follower, score_pose, inner_grab_pose, () -> follower.atPose(inner_grab_pose, 1,1, 0.04) && follower.getVelocityMagnitude() < 2),
                            new SleepAction(4)
                        )
                ),
                intake.pickUpAction(),
                new InstantAction(outtake.claw::open),
                new ParallelAction(
                    new SequentialAction(
                        new RaceAction(
                            foreverAction(() -> {
                                telemetry.addData("intake", intake.slide.getPosition());
                                telemetry.addData("outtake", outtake.slide.getPosition());
                                telemetry.update();
                            }),
                            fullTransferAction(intake, outtake)
                        ),
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
                    new RaceAction(
                        moveLineAction(follower, score_pose, center_grab_pose,  () -> follower.atPose(center_grab_pose, 1.5,1, 0.04)),
                        new SleepAction(4)
                    )
                ),
                intake.pickUpAction(),
                new InstantAction(outtake.claw::open),
                new ParallelAction(
                    new SequentialAction(
                        new RaceAction(
                            foreverAction(() -> {
                                telemetry.addData("intake", intake.slide.getPosition());
                                telemetry.addData("outtake", outtake.slide.getPosition());
                                telemetry.update();
                            }),
                            fullTransferAction(intake, outtake)
                        ),
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
                    new RaceAction(
                            moveLineAction(follower, score_pose, outer_grab_pose,  () -> follower.atPose(outer_grab_pose, 1,1, 0.04)  && follower.getVelocityMagnitude() < 2),
                            new SleepAction(4)
                    )
                ),
                intake.pickUpAction(),
                new InstantAction(outtake.claw::open),
                new ParallelAction(
                    new SequentialAction(
                        new RaceAction(
                            foreverAction(() -> {
                                telemetry.addData("intake", intake.slide.getPosition());
                                telemetry.addData("outtake", outtake.slide.getPosition());
                                telemetry.update();
                            }),
                            fullTransferAction(intake, outtake)
                        ),
                        outtake.readySampleAction()
                    ),
                    moveLineAction(follower, outer_grab_pose, score_pose)
                ),
                outtake.scoreAction(),
                new RaceAction(
                        new ParallelAction(
                                outtake.slideWaitAction(),
                                // SUB 1
                                pathAction(follower, score_to_sub)
                        ),
                        foreverAction(intake.slide::movementLoop)
                ),
                new RaceAction(
                        triggerAction(() -> found_sample),
                        foreverAction(this::getSampleAction)
                ),
                new InstantAction(() -> found_sample = false),
                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(2),
                                outtake.readySampleAction()
                        ),
                        new RaceAction(
                            pathAction(follower, sub_to_score, () -> !follower.isBusy()),
                            foreverAction(()->{telemetry.addData("isBusy", follower.isBusy()); telemetry.update();})
                        )

                ),
                outtake.scoreAction(),
                new RaceAction(
                    new ParallelAction(
                        outtake.slideWaitAction(),
                        // SUB 2
                        pathAction(follower, score_to_sub)
                    ),
                    foreverAction(intake.slide::movementLoop)
                ),
                new RaceAction(
                        triggerAction(() -> found_sample),
                        foreverAction(this::getSampleAction)
                ),
                new InstantAction(() -> found_sample = false),
                new ParallelAction(
                    new SequentialAction(
                        new SleepAction(2),
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
