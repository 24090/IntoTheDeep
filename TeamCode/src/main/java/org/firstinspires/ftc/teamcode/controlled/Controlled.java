package org.firstinspires.ftc.teamcode.controlled;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.GameMap;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.PoseStorer;
import org.firstinspires.ftc.teamcode.util.customactions.ForeverAction;

/**
 * TODO:
 *  give pose to next OpMode/take pose from previous
 *  assisted aim if necessary
 *  dpad mode if necessary
 */
@TeleOp(name = "Controller")
public class Controlled extends LinearOpMode{
    FtcDashboard dash = FtcDashboard.getInstance();
    public void runOpMode(){
        double last_time = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorer.pose);
        Intake intake;
        intake = new Intake(hardwareMap);
        Outtake outtake;
        intake.claw.toReadyGrabPos();
        outtake = new Outtake(hardwareMap);

        InstantAction movement = new InstantAction(() -> {
                if (intake.linear_slide.getPosition() > 100) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x / 3));
                } else {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), gamepad1.right_stick_x));
                }

                drive.updatePoseEstimate();
            }
        );
        
        waitForStart();
        while (opModeIsActive()){
            double turret_angle = 0;
            outtake.backgroundIter();
            intake.linear_slide.movementLoop();
            movement.getF().run();
            if (gamepad1.left_bumper){
                intake.readyGrab(
                        gamepad1.left_trigger * (Intake.MaxDistance - Intake.MinDistance) + Intake.MinDistance,
                        0 // TODO: Claw rotation
                );
            } else if (gamepad1.right_bumper){
                Actions.runBlocking(
                        new RaceAction(
                            new ForeverAction(movement),
                            new ForeverAction(outtake::backgroundIter),
                            new SequentialAction(
                                intake.fullTransferAction(),
                                new InstantAction(intake.claw::toReadyGrabPos)
                            )
                        )
                );
            }
            if (gamepad1.dpad_up){
                Actions.runBlocking(
                        new RaceAction(
                                new ForeverAction(movement),
                                new ForeverAction(outtake::backgroundIter),
                                intake.pickUpAction()
                        )
                );
            } else if (gamepad1.dpad_down){
                intake.claw.open();
            }
            if (gamepad1.dpad_left){
                turret_angle -= (time - last_time);
                intake.claw.rotate(turret_angle);
            } else if (gamepad1.dpad_right){
                turret_angle += (time - last_time);
                intake.claw.rotate(turret_angle);
            }
            if (gamepad1.y){
                outtake.slide.up();
            } else if (gamepad1.a){
                outtake.down();
            } else if (gamepad1.b){
                outtake.open();
            } else if (gamepad1.x){
                outtake.close();
            }
            last_time = time;
        }
    }
}
