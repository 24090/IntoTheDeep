package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

public class MechanismActions {
    Intake intake;
    Outtake outtake;
    public MechanismActions(Intake intake, Outtake outtake){
        this.intake = intake;
        this.outtake = outtake;
    }
    class OuttakeSlideUpAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return outtake.linear_slide.extendToIter(4500,50);
        }
    }
    class OuttakeSlideDownAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return outtake.linear_slide.extendToIter(0, 50);
        }
    }
    class OpenGateAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return Math.abs(outtake.open()) < 0.01;
        }
    }
    class CloseGateAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return Math.abs(outtake.close()) < 0.01;
        }
    }
    public Action ReadyGrabAction(double to){
        return new ParallelAction(new HSlideToAction(to), new IntakeDownAction());
    }
    class IntakeReleaseAction implements Action{
        double start_time = -1;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (start_time == -1){
                start_time = System.currentTimeMillis();
            }
            intake.release();
            return (System.currentTimeMillis() - start_time) < 1000;
        }
    }

    class HSlideToAction implements Action{
        double to;
        public HSlideToAction(double to){
            super();
            this.to = to;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return intake.linear_slide.extendToIter(to, 50);
        }
    }
    class HSlideInAction implements Action{
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return intake.linear_slide.extendToIter(0, 50);
        }
    }
    class IntakeDownAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.intake_servo_a1.setPosition(0.75);
            intake.intake_servo_a2.setPosition(1 - 0.75);
            return intake.moveDown() < 0.01;
        }
    }
    class IntakeUpAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.intake_servo_a1.setPosition(0.75);
            intake.intake_servo_a2.setPosition(1 - 0.75);
            return intake.moveUp() < 0.01;
        }
    }
    class IntakeGrabAction implements Action{
        double start_time = -1;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (start_time == -1){
                start_time = System.currentTimeMillis();
            }
            intake.grab();
            return (System.currentTimeMillis() - start_time) < 1000;
        }
    }
    class IntakeHoldAction implements Action{
        double start_time;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.hold();
            return true;
        }
    }
    public Action ReadyTransferAction(){
        return new ParallelAction(new OuttakeSlideDownAction(), new CloseGateAction(), new HSlideInAction(), new IntakeUpAction(), new IntakeHoldAction());
    }
    public Action FullTransferAction(){
        return new SequentialAction(ReadyTransferAction(), new IntakeReleaseAction());
    }
    public Action FullGrabAction(){
        return new SequentialAction(ReadyGrabAction(1000), new IntakeGrabAction());
    }
    public Action FullScoreAction(){
        return new SequentialAction(new OuttakeSlideUpAction(), new OpenGateAction(), new CloseGateAction(), new OuttakeSlideDownAction());
    }
}
