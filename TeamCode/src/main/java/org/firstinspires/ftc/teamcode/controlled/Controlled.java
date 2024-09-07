package org.firstinspires.ftc.teamcode.controlled;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * TODO:
 *  give pose to next OpMode/take pose from previous
 *  assisted aim if necessary
 *  dpad mode if necessary
 */
@TeleOp(name = "Controller")
public class Controlled extends LinearOpMode{
    public void runOpMode(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        waitForStart();
        while (opModeIsActive()){
            // y and x ARE correct here, as "forward" is represented by the X coordinate for roadrunner and the Y coordinate on the controller
            Vector2d linearVelocity = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            double angularVelocity = -gamepad1.right_stick_x;
            drive.setDrivePowers(new PoseVelocity2d(linearVelocity, angularVelocity));
        }
    }
}
