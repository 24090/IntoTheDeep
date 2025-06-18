package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;
import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.pathAction;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
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
@Autonomous(name = "SpecAuto", group = "auto")
public class SpecAuto extends LinearOpMode {
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
        final Pose start_pose = new Pose(GameMap.RobotLength/2, 72 - GameMap.RobotWidth/2, -PI);
        follower.setStartingPose(start_pose);

        final Pose inner_sample = new Pose(46.25, 23.25, 0);
        final Pose center_sample = new Pose(inner_sample.getX(), inner_sample.getY()-10, 0);
        final Pose outer_sample = new Pose(inner_sample.getX(), center_sample.getY()-10, 0);
        final Pose score_pose = new Pose(start_pose.getX() + 28,start_pose.getY(), start_pose.getHeading());
        final Pose inner_sample_sweep = new Pose(inner_sample.getX() - 13, inner_sample.getY() + 19.75,-PI/4);
        final Pose center_sample_sweep = new Pose(center_sample.getX() - 13, center_sample.getY() + 19.75,-PI/4);
        final Pose outer_sample_sweep = new Pose(outer_sample.getX() - 13, outer_sample.getY() + 19.75,-PI/4);
        final Pose grab_pose = new Pose(GameMap.RobotLength/2,36, 0);
        PathChain preload_to_sample_sweep = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(score_pose),
                                new Point(12,60),
                                new Point(inner_sample_sweep)
                        )
                ).setLinearHeadingInterpolation(score_pose.getHeading(),inner_sample_sweep.getHeading())
                .build();
        // HW stuff
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        // same as meepmeep
        Action path = new SequentialAction(
                moveLineAction(start_pose, score_pose),
                new InstantAction(outtake::readySpecimen),
                outtake.scoreSpecimenAction(),
                new ParallelAction(pathAction(follower, preload_to_sample_sweep), intake.readyGrabAction(Intake.MaxDistance,0)),
                new InstantAction(() -> {telemetry.addLine("Done"); telemetry.update();}),
                intake.pickUpAction(),
                moveLineAction(inner_sample_sweep, new Pose(inner_sample_sweep.getX()-3, inner_sample_sweep.getY()-3, inner_sample_sweep.getHeading() - PI/2)),
                new InstantAction(intake.claw::open),
                new ParallelAction(moveLineAction(new Pose(inner_sample_sweep.getX()-3, inner_sample_sweep.getY()-3, inner_sample_sweep.getHeading() - PI/2), center_sample_sweep), intake.readyGrabAction(Intake.MaxDistance,0)),
                intake.pickUpAction(),
                moveLineAction(center_sample_sweep, new Pose(center_sample_sweep.getX()-3, center_sample_sweep.getY()-3, center_sample_sweep.getHeading() - PI/2)),
                new InstantAction(intake.claw::open),
                new ParallelAction(moveLineAction(new Pose(center_sample_sweep.getX()-3, center_sample_sweep.getY()-3, center_sample_sweep.getHeading() - PI/2), outer_sample_sweep), intake.readyGrabAction(Intake.MaxDistance,0)),
                intake.pickUpAction(),
                moveLineAction(outer_sample_sweep, new Pose(outer_sample_sweep.getX()-3, outer_sample_sweep.getY()-3, outer_sample_sweep.getHeading() - PI/2)),
                new InstantAction(intake.claw::open),
                moveLineAction(new Pose(outer_sample_sweep.getX()-3, outer_sample_sweep.getY()-3, outer_sample_sweep.getHeading() - PI/2), grab_pose)

        );
        // these lines ≠ meepmeep
        BulkReads bulkreads = new BulkReads(hardwareMap);
        waitForStart();
        bulkreads.setCachingMode(LynxModule.BulkCachingMode.MANUAL);
        while (!opModeIsActive());
        runBlocking(
                new RaceAction(
                        path,
                        foreverAction(bulkreads::readManual),
                        foreverAction(follower::update),
                        triggerAction(()->!opModeIsActive())
                )
        );
        PoseStorer.pose = follower.getPose();
    }
}

