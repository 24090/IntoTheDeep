package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.util.customactions.RunBlocking.runBlocking;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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
import org.firstinspires.ftc.teamcode.util.customactions.ForeverAction;
import org.firstinspires.ftc.teamcode.util.customactions.PathAction;
import org.firstinspires.ftc.teamcode.util.customactions.TriggerAction;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "AutoRed", group = "auto")
public class AutoRed extends LinearOpMode {
    Follower follower;
    Intake intake;
    Outtake outtake;

    Action move_line_action(Pose a, Pose b) {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(a), new Point(b)))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
        return new SequentialAction(
                new InstantAction(()->follower.followPath(path, true)),
                new TriggerAction(()->(!follower.isBusy())&&(follower.getVelocityMagnitude()<1)&&(follower.getHeadingError()<0.02))
        );
    }
    @Override
    public void runOpMode() {
        final Pose inner_sample = new Pose(48-2.75, 121.5, 0);
        final Pose start_pose = new Pose(GameMap.RobotWidth/2, 120 - GameMap.RobotLength / 2, -PI/2);
        final Pose score_pose = new Pose(18,144-18, -PI / 4);
        final Pose inner_grab_pose = new Pose(inner_sample.getX() - Intake.MaxDistance - 0.5, inner_sample.getY() + 0.75, 0);
        final Pose center_grab_pose = new Pose(inner_sample.getX() - Intake.MaxDistance - 0.5, inner_sample.getY() + 9.25, 0);
        final Pose outer_grab_pose = new Pose(inner_sample.getX(), inner_sample.getY() + 20, 0.9);
        final Vector outer_offset = new Vector(Intake.MaxDistance - 0.5, 0.9);

        // HW stuff
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        // same as meepmeep
        outer_grab_pose.subtract(new Pose(outer_offset.getXComponent(), outer_offset.getYComponent(), 0));
        // this line ≠ meepmeep
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(start_pose);
        // this line ≠ meepmeep
        Action path = new SequentialAction(
            move_line_action(start_pose, score_pose),
            new ParallelAction(
                outtake.scoreAction(),
                intake.readyGrabAction(Intake.MaxDistance - 0.5, -PI/2)
            ),
            new ParallelAction(
                outtake.slideWaitAction(),
                move_line_action(score_pose, inner_grab_pose)
            ),
            intake.pickUpAction(),
            new ParallelAction(
                intake.fullTransferAction(),
                move_line_action(inner_grab_pose, score_pose)
            ),
            new ParallelAction(
                outtake.scoreAction(),
                intake.readyGrabAction(Intake.MaxDistance - 0.5, -PI/2)
            ),
            new ParallelAction(
                outtake.slideWaitAction(),
                move_line_action(score_pose, center_grab_pose)
            ),
            intake.pickUpAction(),
            new ParallelAction(
                intake.fullTransferAction(),
                move_line_action(center_grab_pose, score_pose)
            ),
            new ParallelAction(
                outtake.scoreAction()
            ),
            new ParallelAction(
                outtake.slideWaitAction(),
                move_line_action(score_pose, outer_grab_pose),
                intake.readyGrabAction(Intake.MaxDistance - 0.5, 0.9)
            ),
            intake.pickUpAction(),
            new ParallelAction(
                intake.fullTransferAction()
            ),
            move_line_action(start_pose, score_pose),
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
