package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;
import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.moveLineAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.pathAction;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.util.BulkReads;
import org.firstinspires.ftc.teamcode.util.GameMap;
import org.firstinspires.ftc.teamcode.util.PoseStorer;
import org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions;
import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.firstinspires.ftc.teamcode.vision.Vision;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "SpecAuto", group = "auto")
public class SpecAuto extends LinearOpMode {
    Follower follower;
    Intake intake;
    Outtake outtake;

    @Override
    public void runOpMode() {
        Sample sample = new Sample();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        final Pose start_pose = new Pose(GameMap.RobotLength/2, 72 - GameMap.RobotWidth/2, -PI);
        follower.setStartingPose(start_pose);

        final Pose inner_sample = new Pose(46.25, 23.25 - 1.5, 0);
        final Pose center_sample = new Pose(inner_sample.getX(), inner_sample.getY()-10, 0);
        final Pose outer_sample = new Pose(inner_sample.getX(), center_sample.getY()-10, 0);
        final Pose score_pose = new Pose(start_pose.getX() + 40, start_pose.getY(),-PI);
        final PathChain sweep1 = follower.pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Point(37, 67, Point.CARTESIAN),
                        new Point(7, 0, Point.CARTESIAN),
                        new Point(66.000, 64.000, Point.CARTESIAN),
                        new Point(74.000, 21.000, Point.CARTESIAN),
                        new Point(54.000, 20.000, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(-PI)
                .addPath(
                    new BezierLine(
                        new Point(48.000, 20.000, Point.CARTESIAN),
                        new Point(20, 20, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(-PI)
                .build();

        final PathChain sweep2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(10.000, 22.000, Point.CARTESIAN),
                                new Point(81.000, 59.000, Point.CARTESIAN),
                                new Point(56.000, 6.000, Point.CARTESIAN),
                                new Point(57.000, 8.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Point(48, 8, Point.CARTESIAN),
                                new Point(20, 8, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        final PathChain sweep3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(80.000, 43.000, Point.CARTESIAN),
                                new Point(62.000, 4.000, Point.CARTESIAN),
                                new Point(55.000, 6.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Point(48, 6, Point.CARTESIAN),
                                new Point(20, 6, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        final Pose grab_pose = new Pose(GameMap.RobotLength/2,36, 0);
        // HW stuff
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        Action path = new SequentialAction(
            new ParallelAction(
                outtake.readySpecimenAction(),
                new SequentialAction(
                    new InstantAction(() -> follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(start_pose, score_pose))
                        .setLinearHeadingInterpolation(start_pose.getHeading(), score_pose.getHeading())
                        .build(),
                        false
                    )),
                    triggerAction(()->(follower.getCurrentTValue()>0.3)&&(follower.getVelocityMagnitude()<2))
                )
            ),
            new ParallelAction(
                outtake.readyTransferAction(),
                new SequentialAction(
                        pathAction(follower, sweep1, 2, 0.04),
                        pathAction(follower, sweep2, 2, 0.04),
                        pathAction(follower, sweep3, 2, 0.04)
                )
            )
        );
        BulkReads bulkreads = new BulkReads(hardwareMap);
        outtake.readyTransfer();
        outtake.claw.grab();
        waitForStart();
        bulkreads.setCachingMode(LynxModule.BulkCachingMode.MANUAL);
        while (!opModeIsActive());
        runBlocking(new RaceAction(
            path,
            foreverAction(bulkreads::readManual),
            foreverAction(follower::update),
            triggerAction(()->!opModeIsActive())
        ));
        PoseStorer.pose = follower.getPose();
    }
}

