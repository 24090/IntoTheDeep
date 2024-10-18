package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.*;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.GameMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;

@Autonomous(name = "AutoRed", group = "auto")
public class AutoRed extends LinearOpMode {

    @Override
    public void runOpMode(){
        Pose2d startPose = new Pose2d(GameMap.NetRedCorner.times(0.666), -3*PI/4);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


        Action path = drive.actionBuilder(startPose)
                // red net zone to LN spike marks * 3
                .strafeToSplineHeading(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(-2, 11)), PI/2)
                .waitSeconds(1.0)
                .strafeToSplineHeading(startPose.position, startPose.heading)
                .waitSeconds(1.0)
                .strafeToSplineHeading(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(-2, 11)), PI/2)
                .waitSeconds(1.0)
                .strafeToSplineHeading(startPose.position, startPose.heading)
                .waitSeconds(1.0)
                .strafeToSplineHeading(GameMap.SpikeMarkNeutralLeftCenter.minus(new Vector2d(-2, 11)), PI/2)
                .waitSeconds(1.0)
                .strafeToSplineHeading(startPose.position, startPose.heading)
                .waitSeconds(1.0)
                // Red net zone to Red Center spike marks * 3
                .strafeToSplineHeading(GameMap.SpikeMarkRedCenter.minus(new Vector2d(1, 11)), PI/2)
                .waitSeconds(1.0)
                .strafeToSplineHeading(startPose.position, startPose.heading)
                .waitSeconds(1.0)
                .strafeToSplineHeading(GameMap.SpikeMarkRedCenter.minus(new Vector2d(1, 11)), PI/2)
                .waitSeconds(1.0)
                .strafeToSplineHeading(startPose.position, startPose.heading)
                .waitSeconds(1.0)
                .strafeToSplineHeading(GameMap.SpikeMarkRedCenter.minus(new Vector2d(1, 11)), PI/2)
                .waitSeconds(1.0)
                .strafeToSplineHeading(startPose.position, startPose.heading)
                .build();
        waitForStart();
        Actions.runBlocking(new SequentialAction(path));

    }
}
