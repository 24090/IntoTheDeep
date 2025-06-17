package org.firstinspires.ftc.teamcode.controlled;

import static org.firstinspires.ftc.teamcode.util.RobotActions.fullTransferAction;
import static org.firstinspires.ftc.teamcode.util.customactions.RunBlocking.runBlocking;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.RaceAction;  
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Intake;
import org.firstinspires.ftc.teamcode.util.Outtake;
import org.firstinspires.ftc.teamcode.util.customactions.ForeverAction;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * TODO:
 *  give pose to next OpMode/take pose from previous
 *  assisted aim if necessary
 *  dpad mode if necessary
 */
@TeleOp(name = "Controller")
public class Controlled extends LinearOpMode{
    FtcDashboard dash = FtcDashboard.getInstance();
    Follower follower;
    Intake intake;
    Outtake outtake;
    public enum State{IN_GRAB, NORMAL, SAMPLESCORING, SPECIMENSCORING}
    State state = State.NORMAL;
    public void runOpMode(){
        double last_time = 0;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        boolean old_right_bumper = false;
        intake = new Intake(hardwareMap);
        intake.claw.toReadyGrabPos();
        outtake = new Outtake(hardwareMap);
        InstantAction movement = new InstantAction(() -> {
                follower.setTeleOpMovementVectors(
                            -gamepad1.left_stick_y - gamepad2.left_stick_y/5,
                            -gamepad1.left_stick_x - gamepad2.left_stick_x/5,
                            -gamepad1.right_stick_x - gamepad2.right_stick_y/5
                );
                follower.update();
            }
        );
        InstantAction linearSlide = new InstantAction(()->{
            if gamepad2
        });
        follower.startTeleopDrive();
        waitForStart();
        while (opModeIsActive()){
            double turret_angle = 0;
            outtake.backgroundIter();
            intake.linear_slide.movementLoop();
            movement.getF().run();
            if (gamepad1.right_bumper && !old_right_bumper){
                old_right_bumper = true;
                if (state == State.NORMAL){
                    runBlocking(outtake.readySpecimenAction());
                    state = State.SPECIMENSCORING;
                }
                else if (state == State.SPECIMENSCORING){
                    runBlocking(outtake.scoreSpecimen());
                    state = State.NORMAL;
                }
            } else {
                old_right_bumper = false;
            }
            if (gamepad1.y){
                outtake.readySample();
                state = State.SAMPLESCORING;
            } else if (gamepad1.a){
                outtake.standby();
                state = State.NORMAL;
            } else if (gamepad1.b){
                outtake.claw.open();
            }


            if (gamepad2.x){
                runBlocking(fullTransferAction(intake, outtake));
                state = State.NORMAL;
            }
            if (gamepad2.dpad_down){
                intake.claw.open();
            }
            if (gamepad2.left_bumper){
                intake.readyGrab(
                        gamepad2.left_trigger * (Intake.MaxDistance - Intake.MinDistance) + Intake.MinDistance,
                        0 // TODO: Claw rotation
                );
                state = State.IN_GRAB;
            }
            if (gamepad2.dpad_up){
                runBlocking(
                        new RaceAction(
                                new ForeverAction(movement),
                                new ForeverAction(outtake::backgroundIter),
                                intake.pickUpAction()
                        )
                );
            }
            if (gamepad2.dpad_left){
                turret_angle -= (time - last_time);
                intake.claw.rotate(turret_angle);
            } else if (gamepad2.dpad_right){
                turret_angle += (time - last_time);
                intake.claw.rotate(turret_angle);
            }
            last_time = time;
        }
    }
}
