package org.firstinspires.ftc.teamcode.controlled;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name = "PushBot")
public class PushBot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));
        }
}}
