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

em0 - Front Left Drivetrain Motor (Looking From The Back)
em1 - Front Right Drivetrain Motor (Looking From The Back)
em2 - Back Right Drivetrain Motor (Looking From The Back)
em3 - Back Left Drivetrain Motor (Looking From The Back)
 */

@TeleOp(group="controlled", name="CoolController")
public class CoolController extends LinearOpMode {
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorer.pose);

        Intake intake = new Intake(hardwareMap);

        Outtake outtake = new Outtake(hardwareMap);

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
                sleep(2000);
                outtake.close();
            }
            if(gamepad1.right_bumper && gamepad1.right_trigger == 0) {
                outtake.open();
                sleep(1000);
                intake.close();
                while (gamepad1.right_bumper) {}
            } else if (gamepad1.right_bumper && gamepad1.right_trigger > 0) {
                intake.transferPos();
                outtake.transferPos();
                outtake.activateClaw();
                sleep(100);
                intake.activateClaw();
                sleep(300);
                intake.close();
                sleep(300);
                outtake.open();
                while (gamepad1.right_bumper) {}
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
