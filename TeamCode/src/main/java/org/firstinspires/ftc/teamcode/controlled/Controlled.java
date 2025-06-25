package org.firstinspires.ftc.teamcode.controlled;

import static org.firstinspires.ftc.teamcode.util.CustomActions.foreverAction;
import static org.firstinspires.ftc.teamcode.util.CustomActions.runBlocking;
import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.firmFullTransferAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.fullTransferAction;
import static org.firstinspires.ftc.teamcode.util.mechanisms.RobotActions.moveLineAction;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.PoseStorer;
import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;
import org.firstinspires.ftc.teamcode.util.mechanisms.outtake.Outtake;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

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
        follower.setStartingPose(PoseStorer.pose);
        boolean old_right_bumper = false;
        intake = new Intake(hardwareMap);
        intake.claw.toReadyGrabPos(0);
        outtake = new Outtake(hardwareMap);
        outtake.readyTransfer();
        InstantAction movement = new InstantAction(() -> {
                follower.setTeleOpMovementVectors(
                            -gamepad1.left_stick_y - gamepad2.left_stick_y/7,
                            -gamepad1.left_stick_x - gamepad2.left_stick_x/7,
                            -gamepad1.right_stick_x - gamepad2.right_stick_x/7
                );
                follower.update();
            }
        );
        follower.startTeleopDrive();
        waitForStart();
        while (opModeIsActive()){
            outtake.backgroundIter();
            intake.slide.movementLoop();
            movement.getF().run();
            if (gamepad1.dpad_down){
                follower.breakFollowing();
                runBlocking(
                    new RaceAction(
                        triggerAction(() -> !gamepad1.dpad_down),
                        new ParallelAction(
                            moveLineAction(
                                follower,
                                follower.getPose(),
                                new Pose(19.25,144-19.25, -PI / 4),
                                2, 0.04
                            ),
                            outtake.readySampleAction()
                        )
                    )
                );
                follower.breakFollowing();
                follower.startTeleopDrive();
            }
            if (gamepad1.right_bumper && !old_right_bumper){
                old_right_bumper = true;
                if (state == State.NORMAL){
                    runBlocking(
                        new RaceAction(
                            foreverAction(movement),
                            outtake.readySpecimenAction()
                        )
                    );
                    state = State.SPECIMENSCORING;
                }
                else if (state == State.SPECIMENSCORING){
                    runBlocking(
                            new RaceAction(
                                    foreverAction(movement),
                                    outtake.scoreSpecimenAction()
                            )
                    );
                    state = State.NORMAL;
                }
            } else {
                old_right_bumper = false;
            }
            if (gamepad1.y){
                outtake.readySample();
                state = State.SAMPLESCORING;
            } else if (gamepad1.a){
                outtake.readyTransfer();
                state = State.NORMAL;
            } else if (gamepad1.b){
                outtake.claw.open();
            }
            if (gamepad2.right_bumper){
                runBlocking(
                    new RaceAction(
                        foreverAction(movement),
                        fullTransferAction(intake, outtake)
                    )
                );
                state = State.NORMAL;
            }
            if (gamepad2.right_stick_button){
                intake.claw.turret_angle = 0;
                intake.readyGrab(
                        Intake.MaxDistance
                );
                state = State.IN_GRAB;
            }
            if (gamepad2.dpad_up){
                runBlocking(
                        new RaceAction(
                                foreverAction(movement),
                                firmFullTransferAction(intake, outtake)
                        )
                );
                state = State.NORMAL;
            }
            intake.slide.goTo(intake.slide.trimTicks(
                    intake.slide.target_pos +
                    (int) ((gamepad2.dpad_up? 1 : (gamepad2.dpad_down? -1 :0)) * 70 * (time - last_time))
            ));
            if (gamepad2.a){
                runBlocking(
                        new RaceAction(
                                foreverAction(movement),
                                foreverAction(outtake::backgroundIter),
                                intake.pickUpAction()
                        )
                );
            }
            if (gamepad2.y){
                intake.claw.open();
            }
            if (gamepad2.b){
                intake.claw.turret_angle = 0;
                intake.claw.toReadyGrabPos(intake.claw.turret_angle);
                intake.claw.wrist_ready();
            } else if (gamepad2.x){
                intake.claw.turret_angle = PI/2;
                intake.claw.toReadyGrabPos(intake.claw.turret_angle);
                intake.claw.wrist_ready();
            }
            last_time = time;
        }
    }
}
