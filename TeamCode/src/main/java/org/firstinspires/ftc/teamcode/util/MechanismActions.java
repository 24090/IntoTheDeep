package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MechanismActions {
    Intake intake;
    Outtake outtake;
    LinearOpMode opMode;
    public MechanismActions(Intake intake, Outtake outtake, LinearOpMode opMode){
        this.intake = intake;
        this.outtake = outtake;
        this.opMode = opMode;
    }
    public class OuttakeSlideUpAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outtake.up();
            return !outtake.linear_slide.within_error;
        }
    }
    public class OuttakeSlideDownAction implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outtake.down();
            return !outtake.linear_slide.within_error;
        }
    }
    public Action OpenGateAction(){
        return new ParallelAction(new SleepAction(1), new InstantAction(() -> outtake.open()));
    }
    public Action CloseGateAction(){
        return new InstantAction(() -> outtake.close());
    }
    public Action ScoreAction(){
        return new SequentialAction( new OuttakeSlideUpAction(), OpenGateAction(), new InstantAction(() -> outtake.linear_slide.moveDown()));
    }
    public Action FullScoreAction(){
        return new SequentialAction( new OuttakeSlideUpAction(), OpenGateAction(), OuttakeSlideDownAction());
    }
    public Action OuttakeSlideUpAction(){
        return new OuttakeSlideUpAction();
    }
    public Action OuttakeSlideDownAction(){
        return new ParallelAction(new OuttakeSlideDownAction(), new SequentialAction(new SleepAction(0.3), CloseGateAction()));
    }
}
