package org.firstinspires.ftc.teamcode.controlled;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;
import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions;
import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;

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
        intake.claw.toReadyGrabPos(0);
        outtake = new Outtake(hardwareMap);
        InstantAction intake_update = new InstantAction(() -> {
            if (Math.abs(gamepad1.left_stick_y) > 0.5) {
                intake.slide.goTo(
                        intake.slide.trimTicks(intake.slide.getPosition() - 100 * (int) Math.signum(gamepad1.left_stick_y)),
                        50
                );
                intake.slide.setMotorPower(-gamepad1.left_stick_y/4);
            } else {
                if (gamepad1.left_stick_button) {
                    intake.slide.moveOut();
                }
                intake.slide.movementLoop();
            }
        });
        InstantAction movement = new InstantAction(() -> {
            if (intake.slide.getPosition() > 200 || gamepad1.right_stick_button) {
                follower.setTeleOpMovementVectors(-gamepad1.right_stick_y/3, -gamepad1.right_stick_x/3, -gamepad1.left_stick_x/3);
            } else {
                follower.setTeleOpMovementVectors(-gamepad1.right_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
            }
            follower.update();
        });
        boolean old_a = false;
        boolean old_b = false;
        follower.startTeleopDrive();
        waitForStart();
        outtake.readyTransfer();
        intake.readyGrab(0);
        while (opModeIsActive()){
            outtake.backgroundIter();
            telemetry.addData("loop time after outtake", (time - last_time) * 1000);
            intake_update.getF().run();
            telemetry.addData("loop time after intake+outtake", (time - last_time) * 1000);
            movement.getF().run();
            telemetry.addData("loop time after intake+outtake+movement", (time - last_time) * 1000);
            if (gamepad1.left_bumper){
                runBlocking(
                    new RaceAction(
                        foreverAction(movement),
                        foreverAction(outtake::backgroundIter),
                        new SequentialAction(
                            RobotActions.fullTransferAction(intake, outtake),
                            new InstantAction(() -> intake.claw.toReadyGrabPos())
                        )
                    )
                );
            }
            if (gamepad1.right_bumper) {
                runBlocking(
                    new RaceAction(
                        foreverAction(movement),
                        foreverAction(outtake::backgroundIter),
                        intake.pickUpAction()
                    )
                );
            }
            if (gamepad1.x) {
                outtake.readySample();
            } else if (gamepad1.y) {
                outtake.readyTransfer();
            }
            if (gamepad1.a && !old_a){
                outtake.claw.toggleGrab();
            }
            if (gamepad1.b && !old_b){
                intake.claw.toggleGrab();
            }
            if (gamepad2.left_stick_button) {
                intake.claw.turret_angle = atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            }
            intake.claw.wrist_ready();
            old_a = gamepad1.a;
            old_b = gamepad1.b;
            telemetry.addData("loop time (ms))", (time - last_time) * 1000);
            telemetry.addData("turret angle (deg)", intake.claw.turret_angle/PI * 180);
            last_time = time;
            telemetry.update();
        }
    }
}
