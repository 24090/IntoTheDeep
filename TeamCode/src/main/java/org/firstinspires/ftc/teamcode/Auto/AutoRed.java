package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.*;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.GameMap;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.MechanismActions;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.PoseStorer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoRed", group = "auto")
public class AutoRed extends LinearOpMode {

    @Override
    public void runOpMode() {

        // HW stuff
        Intake intake = new Intake(
                hardwareMap.get(Servo.class, "intake_servo_a1"),
                hardwareMap.get(Servo.class, "intake_servo_a2"),
                hardwareMap.get(Servo.class, "intake_servo_b"),
                hardwareMap.get(DcMotor.class, "intake_motor")
        );
        Outtake outtake;
        outtake = new Outtake(
                hardwareMap.get(Servo.class, "outtake_servo"),
                hardwareMap.get(DcMotor.class, "outtake_slide_motor")
        );
        MechanismActions ma = new MechanismActions(intake, outtake, this);
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
                .stopAndAdd(new ParallelAction(ma.ScoreAction(), ma.ReadyGrabAction(InnerDistance.norm())))
                .setTangent(0)
                .strafeToSplineHeading(neutral_spike_mark_position, InnerDistance.angleCast().toDouble(), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 20;
                    }})
                .stopAndAdd(ma.GrabSpinAction())
                .setTangent(0)
                .strafeTo(neutral_spike_mark_position.plus(InnerDistance.div(InnerDistance.norm()).angleCast().times(new Vector2d(7, 0))))
                .stopAndAdd(ma.FullTransferAction())
                .setTangent(0)
                .strafeToSplineHeading(score_pose.position.minus(new Vector2d(0.5, 0.5)), PI/4 + 0.1)
                .stopAndAdd(new ParallelAction(ma.ScoreAction(), ma.ReadyGrabAction(CenterDistance.norm() - 2)))
                .setTangent(0)
                .strafeToSplineHeading(neutral_spike_mark_position, CenterDistance.angleCast().toDouble(), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 10;
                    }})
                .stopAndAdd(ma.GrabSpinAction())
                .setTangent(0)
                .strafeTo(neutral_spike_mark_position.plus(CenterDistance.div(CenterDistance.norm()).angleCast().times(new Vector2d(7, 0))))
                .stopAndAdd(ma.FullTransferAction())
                .setTangent(0)
                .strafeToSplineHeading(score_pose.position.minus(new Vector2d(1.5, 1.5)), PI/4 + 0.1)
                .stopAndAdd(new ParallelAction(ma.ScoreAction(), ma.ReadyGrabAction(OuterDistance.norm() - 2)))
                .setTangent(0)
                .strafeToSplineHeading(neutral_spike_mark_position, OuterDistance.angleCast().toDouble(), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 20;
                    }}
                )
                .stopAndAdd(ma.GrabSpinAction())
                .setTangent(0)
                .strafeTo(neutral_spike_mark_position.plus(OuterDistance.div(OuterDistance.norm()).angleCast().times(new Vector2d(7, 0))))
                .stopAndAdd(ma.FullTransferAction())
                .setTangent(0)
                .strafeToSplineHeading(score_pose.position.minus(new Vector2d(1, 1)), PI/4 + 0.1)
                .stopAndAdd(ma.FullScoreAction())
                .build();
        // these lines ≠ meepmeep
        waitForStart();
        Actions.runBlocking(path);
        PoseStorer.pose = drive.pose;
    }
}
