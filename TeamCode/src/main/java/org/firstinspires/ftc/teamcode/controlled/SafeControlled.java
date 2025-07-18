package org.firstinspires.ftc.teamcode.controlled;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;
import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions;
import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "Safe Controlled")
public class SafeControlled extends LinearOpMode {
    FtcDashboard dash = FtcDashboard.getInstance();
    double last_time = 0;
    public void runOpMode(){
        // Initialize
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        Intake intake;
        intake = new Intake(hardwareMap);
        Outtake outtake;
        intake.claw.toReadyGrabPos();
        outtake = new Outtake(hardwareMap);

        InstantAction intake_update = new InstantAction(() -> {
            if (Math.abs(gamepad1.left_stick_y) > 0.5) {
                intake.slide.goTo(
                        intake.slide.trimTicks(intake.slide.getPosition() - 100 * (int) Math.signum(gamepad1.left_stick_y)),
                        50
                );
                intake.slide.setMotorPower(-gamepad1.left_stick_y * 2 + Math.signum(gamepad1.left_stick_y));
            } else if (gamepad1.dpad_up) {
                intake.slide.goTo(
                        intake.slide.trimTicks((int) (intake.slide.getPosition() + 200 * (last_time - time))),
                        50
                );
                intake.slide.setMotorPower(0.1);
            } else if (gamepad1.dpad_down) {
                intake.slide.goTo(
                        intake.slide.trimTicks((int) (intake.slide.getPosition() - 200 * (last_time - time))),
                        50
                );
                intake.slide.setMotorPower(-0.1);
            } else {
                if (gamepad1.left_stick_button) {
                    intake.slide.moveOut();
                }
                intake.slide.movementLoop();
            }
        });

        Runnable movement = () -> {
            Vector vel = new Vector(new Point(-gamepad1.right_stick_y, -gamepad1.right_stick_x));
            vel.rotateVector(follower.getPose().getHeading());
            if (follower.getPose().getX() > 11) {
                vel.setOrthogonalComponents(min(vel.getXComponent(), -0.1), vel.getYComponent());
            }
            if (follower.getPose().getY() > 11) {
                vel.setOrthogonalComponents(vel.getXComponent(), min(vel.getYComponent(), -0.1));
            }
            if (follower.getPose().getX() < -11) {
                vel.setOrthogonalComponents(max(vel.getXComponent(), 0.1), vel.getYComponent());
            }
            if (follower.getPose().getY() < -11) {
                vel.setOrthogonalComponents(vel.getXComponent(), max(vel.getYComponent(), 0.1));
            }
            vel.rotateVector(-follower.getPose().getHeading());
            follower.setTeleOpMovementVectors(vel.getXComponent(), vel.getYComponent(), -gamepad1.left_stick_x);
            follower.update();
        };
        boolean old_a = false;
        boolean old_b = false;
        follower.startTeleopDrive();
        waitForStart();
        while (opModeIsActive()){
            outtake.backgroundIter();
            telemetry.addData("loop time after outtake", (time - last_time) * 1000);
            intake_update.getF().run();
            telemetry.addData("loop time after intake+outtake", (time - last_time) * 1000);
            movement.run();
            telemetry.addData("loop time after intake+outtake+movement", (time - last_time) * 1000);
            if (gamepad1.left_bumper){
                runBlocking(
                        new RaceAction(
                                foreverAction(movement),
                                foreverAction(outtake::backgroundIter),
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
                                foreverAction(movement),
                                foreverAction(outtake::backgroundIter),
                                intake.pickUpAction()
                        )
                );

            }
            if (gamepad1.x) {
                outtake.readySampleAction();
            } else if (gamepad1.y) {
                outtake.readyTransfer();
            }
            if (gamepad1.left_trigger > 0.4) {
                outtake.readyTransfer();
            } else if (gamepad1.right_trigger > 0.4) {
                outtake.readySampleAction();
            }
            if (gamepad1.a && !old_a){
                outtake.claw.toggleGrab();
            }
            if (gamepad1.b && !old_b){
                intake.claw.toggleGrab();
            }
            if (gamepad1.dpad_left) {
                intake.claw.turret_angle += (time-last_time);
                intake.claw.wrist_ready();
            } else if (gamepad1.dpad_right){
                intake.claw.turret_angle -= (time-last_time);
                intake.claw.wrist_ready();
            }
            old_a = gamepad1.a;
            old_b = gamepad1.b;
            telemetry.addData("loop time (ms))", (time - last_time) * 1000);
            telemetry.addData("turret angle (deg)", intake.claw.turret_angle/PI * 180);
            last_time = time;
            telemetry.update();
        }
    }
}
