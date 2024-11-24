package org.firstinspires.ftc.teamcode.controlled;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.util.ColorDistance;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Mechanisms;
import org.firstinspires.ftc.teamcode.util.Outtake;

/**
 * TODO:
 *  give pose to next OpMode/take pose from previous
 *  assisted aim if necessary
 *  dpad mode if necessary
 */
@TeleOp(name = "Controller")
public class Controlled extends LinearOpMode{
    Thread transfer_thread;
    public void runOpMode(){
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
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
        waitForStart();
        while (opModeIsActive()){
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y/ (gamepad1.left_stick_button ? 3 : 1), -gamepad1.left_stick_x/ (gamepad1.left_stick_button ? 3 : 1)), -gamepad1.right_stick_x/ (gamepad1.right_stick_button ? 3 : 1)));
            if (gamepad1.left_bumper){
                intake.stop();
                intake.moveDown();
                intake.linear_slide.extendToAsync(gamepad1.left_trigger * 1100, 50);
            } else if (gamepad1.right_bumper){
                if (transfer_thread == null || !transfer_thread.isAlive()){
                    transfer_thread = new Thread(()->{
                        outtake.close();
                        outtake.down();
                        intake.transferSample();
                    });
                    transfer_thread.run();
                }
            }
            if (gamepad1.dpad_up){
                intake.release();
            } else if (gamepad1.dpad_down){
                    intake.grab();
                } else {
                    intake.stop();
                }
            }
            if (gamepad1.y){
                outtake.up();
            } else if (gamepad1.a){
                outtake.down();
                sleep(500);
                outtake.close();
            } else if (gamepad1.b){
                outtake.open();
            } else {
                outtake.linear_slide.stop();
            }
            telemetry.addData("pos", outtake.linear_slide.getPos());
            telemetry.update();
        }
    }

