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
        Vector2d score_position = GameMap.NetRedCorner.minus(new Vector2d(GameMap.OuttakeDistance,0).times(PI/4));
        Pase2d start_pose = new Pose2d(GameMap.NetRedCorner.plus(new Vector2d()), PI/2);
        Pose2d score_pose = new Pose2d(score_position, 5.0/4.0*PI);
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
        MechanismActions ma = new MechanismActions(intake, outtake);
        MecanumDrive drive = new MecanumDrive(hardwareMap, score_pose);
        Action path = drive.actionBuilder(score_pose)
                // red net zone to LN spike marks * 3
                .strafeToSplineHeading(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(GameMap.MaxIntakeDistance, 0)), PI/2)
                .stopAndAdd(ma.FullGrabAction())
                .stopAndAdd(ma.FullTransferAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .stopAndAdd(ma.FullScoreAction())
                .strafeToSplineHeading(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(GameMap.MaxIntakeDistance, 0)), PI/2)
                .stopAndAdd(ma.FullGrabAction())
                .stopAndAdd(ma.FullTransferAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .stopAndAdd(ma.FullScoreAction())
                .strafeToSplineHeading(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(GameMap.MaxIntakeDistance, 0)), PI/2)
                .stopAndAdd(ma.FullGrabAction())
                .stopAndAdd(ma.FullTransferAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .stopAndAdd(ma.FullScoreAction())
                // Red net zone to Red Center spike marks * 3
                .strafeToSplineHeading(GameMap.SpikeMarkRedCenter.minus(new Vector2d(GameMap.MaxIntakeDistance, 0)), PI/2)
                .stopAndAdd(ma.FullGrabAction())
                .stopAndAdd(ma.FullTransferAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .stopAndAdd(ma.FullScoreAction())
                .strafeToSplineHeading(GameMap.SpikeMarkRedCenter.minus(new Vector2d(GameMap.MaxIntakeDistance, 0)), PI/2)
                .stopAndAdd(ma.FullGrabAction())
                .stopAndAdd(ma.FullTransferAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .stopAndAdd(ma.FullScoreAction())
                .strafeToSplineHeading(GameMap.SpikeMarkRedCenter.minus(new Vector2d(GameMap.MaxIntakeDistance, 0)), PI/2)
                .stopAndAdd(ma.FullGrabAction())
                .stopAndAdd(ma.FullTransferAction())
                .strafeToSplineHeading(score_pose.position, score_pose.heading)
                .stopAndAdd(ma.FullScoreAction())
                .build();
        waitForStart();
        Actions.runBlocking(new SequentialAction(path));

    }
}
