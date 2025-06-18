package org.firstinspires.ftc.teamcode.controlled;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "PushBot")
public class PushBot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.startTeleopDrive();
        waitForStart();
        while (opModeIsActive()) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            follower.update();
        }
}}
