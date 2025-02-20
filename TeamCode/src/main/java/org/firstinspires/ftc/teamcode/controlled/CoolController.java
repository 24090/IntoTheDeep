package org.firstinspires.ftc.teamcode.controlled;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.PoseStorer;

/*
cs0 - Intake Claw
cs1 - Intake Claw Orientation
cs2 - Intake Wrist
cs3 - Intake Arm
cs4 - Outtake Arm Right (Looking From The Back)
cs5 - Outtake Arm Left (Looking From The Back)

cm0 - Intake Slide Motor Right (Looking From The Back)
cm1 -
cm2 - Outtake Slide Motor Right (Looking From The Back)
cm3 - Outtake Slide Motor Left (Looking From The Back)

es0 - Outtake Claw
es1 - cs3
es2 -
es3 -
es4 -
es5 -

em0 -  Front Left
em1 - Front Right
em2 - Back Right
em3 - Back Left
 */

@TeleOp(group="controlled", name="CoolController")
public class CoolController extends LinearOpMode {
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorer.pose);

        Intake intake = new Intake(
                hardwareMap.get(Servo.class, "cs0"),
                hardwareMap.get(Servo.class, "cs1"),
                hardwareMap.get(Servo.class, "cs2"),
                hardwareMap.get(Servo.class, "es1"),
                hardwareMap.get(DcMotor.class, "cm1")
        );

        Outtake outtake = new Outtake(
                hardwareMap.get(Servo.class, "es0"),
                hardwareMap.get(Servo.class, "cs4"),
                hardwareMap.get(Servo.class, "cs5"),
                hardwareMap.get(DcMotor.class, "cm2"),
                hardwareMap.get(DcMotor.class, "cm3")
        );

        Thread t = new Thread(() -> {
            while (opModeIsActive()){
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_y, gamepad1.left_stick_x), gamepad1.right_stick_x));
                drive.updatePoseEstimate();
            }
        });

        waitForStart();
        t.start();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                if (gamepad1.left_trigger > 0) {
                    intake.longIntake();
                } else {
                    intake.shortIntake();
                }
                outtake.close();
            }
            if(gamepad1.right_bumper) {
                outtake.open();
                intake.close();
            }
            if (gamepad1.a) {
                outtake.grab();
                intake.grab();
                while (gamepad1.a) {}
            }
            if (gamepad1.b) {
                outtake.score();
            }
            if (gamepad1.dpad_right) {
                intake.moveWristRight();
            }
            if (gamepad1.dpad_left) {
                intake.moveWristLeft();
            }

            telemetry.addData("outtake state", outtake.outtake_state);
            telemetry.addData("intake state", intake.intake_state);
            telemetry.update();
        }
    }
}
