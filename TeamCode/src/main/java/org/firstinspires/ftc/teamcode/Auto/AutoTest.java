package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.*;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;

@Autonomous(name = "AutoTest", group = "auto")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-37.35, -71.30, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Action diagonal;
        diagonal = drive.actionBuilder(drive.pose) // RRPathGen is still on RR 0.5, make sure to update this stuff when pasting
                .splineTo(new Vector2d(-37.00, -12.28), Math.toRadians(90.00))
                .splineTo(new Vector2d(12.28, -11.58), Math.toRadians(0.81))
                .splineTo(new Vector2d(47.80, -37.87), Math.toRadians(180))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(diagonal));

    }
}
