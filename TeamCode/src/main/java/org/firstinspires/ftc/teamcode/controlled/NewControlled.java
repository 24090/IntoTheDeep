package org.firstinspires.ftc.teamcode.controlled;

import static org.firstinspires.ftc.teamcode.util.customactions.RunBlocking.runBlocking;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.RobotActions;
import org.firstinspires.ftc.teamcode.util.customactions.ForeverAction;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "New Controlled")
public class NewControlled extends LinearOpMode {
    FtcDashboard dash = FtcDashboard.getInstance();
    double last_time = 0;
    public void runOpMode(){
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        Intake intake;
        intake = new Intake(hardwareMap);
        Outtake outtake;
        intake.claw.toReadyGrabPos();
        outtake = new Outtake(hardwareMap);

        InstantAction intake_update = new InstantAction(() -> {
            /*if (Math.abs(gamepad2.left_stick_y) > 0.5) {
                intake.linear_slide.goTo(
                        intake.linear_slide.trimTicks(intake.linear_slide.getPosition() - 100 * Math.signum(gamepad2.left_stick_y)),
                        50
                );
                intake.linear_slide.setMotorPower(-gamepad1.left_stick_y * 2 + Math.signum(gamepad2.left_stick_y));
            }*/ if (gamepad2.dpad_up) {
                intake.linear_slide.goTo(
                        intake.linear_slide.trimTicks(intake.linear_slide.getPosition() + 200 * (last_time - time)),
                        50
                );
                intake.linear_slide.setMotorPower(0.1);
            } else if (gamepad2.dpad_down) {
                intake.linear_slide.goTo(
                        intake.linear_slide.trimTicks(intake.linear_slide.getPosition() - 200 * (last_time - time)),
                        50
                );
                intake.linear_slide.setMotorPower(-0.1);
            } else {
                if (gamepad2.left_stick_button) {
                    intake.linear_slide.moveOut();
                }
                intake.linear_slide.movementLoop();
            }
        });

        InstantAction movement = new InstantAction(() -> {
            if (intake.linear_slide.getPosition() > 200 || gamepad1.right_stick_button) {
                follower.setTeleOpMovementVectors(-gamepad1.right_stick_y/3, -gamepad1.left_stick_x/3, -gamepad1.right_stick_x/3);
            } else {
                follower.setTeleOpMovementVectors(-gamepad1.right_stick_y, -gamepad1.right_stick_x, gamepad1.left_stick_x);
            }
            follower.update();
        }
        );
        boolean old_a = false;
        boolean old_b = false;
        follower.startTeleopDrive();
        waitForStart();
        while (opModeIsActive()){
            double turret_angle = 0;
            outtake.backgroundIter();
            telemetry.addData("loop time after outtake", (time - last_time) * 1000);
            intake_update.getF().run();
            telemetry.addData("loop time after intake+outtake", (time - last_time) * 1000);
            movement.getF().run();
            telemetry.addData("loop time after intake+outtake+movement", (time - last_time) * 1000);
            if (gamepad1.left_bumper){
                runBlocking(
                    new RaceAction(
                        new ForeverAction(movement),
                        new ForeverAction(outtake::backgroundIter),
                        new SequentialAction(
                            RobotActions.fullTransferAction(intake, outtake),
                            new InstantAction(intake.claw::toReadyGrabPos)
                        )
                    )
                );
            }
            if (gamepad1.right_bumper) {
                runBlocking(
                    new RaceAction(
                        new ForeverAction(movement),
                        new ForeverAction(outtake::backgroundIter),
                        intake.pickUpAction()
                    )
                );
            }
            if (gamepad1.x) {
                outtake.readySample();
            } else if (gamepad1.y) {
                outtake.readyTransfer();
            }
            if (gamepad1.left_trigger > 0.4) {
                outtake.readyTransfer();
            } else if (gamepad1.right_trigger > 0.4) {
                outtake.readySample();
            }
            if (gamepad1.a && !old_a){
                outtake.claw.toggleGrab();
            }
            if (gamepad1.b && !old_b){
                intake.claw.toggleGrab();
            }
            if (gamepad1.dpad_left) {
                turret_angle += (time-last_time);
                intake.claw.rotate(turret_angle);
            } else if (gamepad1.dpad_right){
                turret_angle -= (time-last_time);
                intake.claw.rotate(turret_angle);
            }
            old_a = gamepad1.a;
            old_b = gamepad1.b;
            telemetry.addData("loop time (ms))", (time - last_time) * 1000);
            telemetry.addData("turret angle (deg)", turret_angle/PI * 180);
            last_time = time;
            telemetry.update();
        }
    }
}
