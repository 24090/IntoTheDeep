package org.firstinspires.ftc.teamcode.controlled;

import static org.firstinspires.ftc.teamcode.util.customactions.RunBlocking.runBlocking;

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

/**
 * TODO:
 *  give pose to next OpMode/take pose from previous
 *  assisted aim if necessary
 *  dpad mode if necessary
 */
@TeleOp(name = "Controller")
public class Controlled extends LinearOpMode{
    FtcDashboard dash = FtcDashboard.getInstance();
    public enum State{IN_GRAB, NORMAL, SCORING}
    State state = State.NORMAL;
    public void runOpMode(){
        double last_time = 0;
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        RobotActions robotActions;
        robotActions = new RobotActions();
        Intake intake;
        intake = new Intake(hardwareMap);
        Outtake outtake;
        intake.claw.toReadyGrabPos();
        outtake = new Outtake(hardwareMap);
        InstantAction movement = new InstantAction(() -> {
                if (state != State.IN_GRAB) {
                    follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                } else {
                    follower.setTeleOpMovementVectors(0, -gamepad1.left_stick_x/3, 0);
                }
                follower.update();
            }
        );
        follower.startTeleopDrive();
        waitForStart();
        while (opModeIsActive()){
            double turret_angle = 0;
            outtake.backgroundIter();
            intake.linear_slide.movementLoop();
            movement.getF().run();
            if (gamepad1.right_bumper){
                runBlocking(
                        new RaceAction(
                            new ForeverAction(movement),
                            new ForeverAction(outtake::backgroundIter),
                            RobotActions.fullTransferAction(intake, outtake)
                        )
                );
            }
            if (gamepad1.y){
                outtake.readySample();
                state = State.SCORING;
            } else if (gamepad1.a){
                outtake.standby();
                state = State.NORMAL;
            } else if (gamepad1.b){
                outtake.claw.open();
            }


            if (gamepad2.x){
                runBlocking(robotActions.fullTransferAction(intake, outtake));
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
