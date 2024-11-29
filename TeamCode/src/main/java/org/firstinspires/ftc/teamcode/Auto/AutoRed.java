package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.*;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.GameMap;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.MechanismActions;
import org.firstinspires.ftc.teamcode.util.Outtake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoRed", group = "auto")
public class AutoRed extends LinearOpMode {

    @Override
    public void runOpMode(){
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

        Vector2d score_position =
                GameMap.NetRedCorner.plus(new Vector2d(9.5,9.5)).plus(
                        Rotation2d.fromDouble(PI/4).times(new Vector2d(GameMap.OuttakeDistance, 0))
                );
        Pose2d start_pose = new Pose2d(GameMap.NetRedCorner.plus(new Vector2d(24 + GameMap.RobotWidth/2,GameMap.RobotLength/2)), PI/2);
        Pose2d score_pose = new Pose2d(score_position, PI/4);
        Vector2d park_position = GameMap.ObservationRedCorner.plus(new Vector2d(-11.25, GameMap.RobotLength/2 + 1));
        Vector2d neutral_spike_mark_position = GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(0, GameMap.MaxIntakeDistance - 4));
        Vector2d red_spike_mark_position = GameMap.SpikeMarkRedCenter.minus(new Vector2d(0, GameMap.MaxIntakeDistance - 4));
        double rotation = GameMap.SpikeMarkNeutralLeftInner.minus(neutral_spike_mark_position).angleCast().toDouble() - PI/2;
        MecanumDrive drive = new MecanumDrive(hardwareMap, start_pose);
        Action path = drive.actionBuilder(start_pose)
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .stopAndAdd(ma.FullScoreAction())
                //SCORE #1
                // red net zone to LN spike marks * 3
                .strafeToSplineHeading(neutral_spike_mark_position, PI )
                .stopAndAdd(ma.FullGrabAction())
                .stopAndAdd(ma.FullTransferAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .stopAndAdd(ma.FullScoreAction())
                .strafeToSplineHeading(neutral_spike_mark_position.minus(new Vector2d(10,0)), PI)
                .stopAndAdd(ma.FullGrabAction())
                .stopAndAdd(ma.FullTransferAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .stopAndAdd(ma.FullScoreAction())
                .strafeToSplineHeading(neutral_spike_mark_position.minus(new Vector2d(20,0)), PI)
                .stopAndAdd(ma.FullGrabAction())
                .stopAndAdd(ma.FullTransferAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .strafeToSplineHeading(park_position, PI/2)
                .build();
        waitForStart();
        Actions.runBlocking(new SequentialAction(path));

    }
}
