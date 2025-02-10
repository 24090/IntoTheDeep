package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.*;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;

@Autonomous(name = "AutoTest", group = "auto")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        Pose2d startPose = new Pose2d(13.15, -71.48, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action mainPath;

        mainPath = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(-0.09, -35.96), Math.toRadians(90))
                .waitSeconds(5)
                .splineTo(new Vector2d(-53.19, -53.54), Math.toRadians(235))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(mainPath));

    }
}
