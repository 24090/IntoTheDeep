package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
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
import org.firstinspires.ftc.teamcode.vision.Vision;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

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
        return new PathAction(path, follower);
    }
    @Override
    public void runOpMode() {
        // HW stuff
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        // same as meepmeep
        final Pose inner_sample = new Pose(23, 45, 0);
        final Pose start_pose = new Pose(24 + GameMap.RobotLength / 2, GameMap.RobotWidth/ 2, 0);
        final Pose score_pose = new Pose(17,17, PI / 4);
        final Pose inner_grab_pose = new Pose(inner_sample.getX(), inner_sample.getY() - Intake.MaxDistance - 0.5, PI/2);
        final Pose center_grab_pose = new Pose(inner_sample.getX() - 10, inner_sample.getY() - Intake.MaxDistance - 0.5, PI/2);
        final Pose outer_grab_pose = new Pose(inner_sample.getX() - 20, inner_sample.getY(), 0.95);
        final Vector outer_offset = new Vector(Intake.MaxDistance - 0.5, 0.95);
        outer_grab_pose.subtract(new Pose(outer_offset.getXComponent(), outer_offset.getYComponent(), -outer_offset.getTheta()));
        // this line ≠ meepmeep
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
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
                outtake.scoreAction(),
                intake.readyGrabAction(Intake.MaxDistance - 0.5, -PI/2)
            ),
            new ParallelAction(
                outtake.slideWaitAction(),
                move_line_action(score_pose, score_pose)
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
        double last_time = -1;
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            bulkreads.readManual();
            path.run(packet);
        }
        PoseStorer.pose = follower.pose;
    }
}
