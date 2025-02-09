package org.firstinspires.ftc.teamcode.NewRobot.Util.IntegratedMechs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NewRobot.Util.LinearSlides.NewIntakeSlide;
import org.firstinspires.ftc.teamcode.NewRobot.Util.LinearSlides.NewOuttakeSlide;
import org.firstinspires.ftc.teamcode.OldRobot.Util.Intake;
import org.firstinspires.ftc.teamcode.OldRobot.Util.Outtake;

public class NewMechActions {
    NewIntakeSlide intake;
    public NewMechActions(NewIntakeSlide intake){
        this.intake = intake;
    }
    public class HSlideToAction implements Action{
        double to;
        public HSlideToAction(double to){
            super();
            this.to = to;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return !(intake.extendToIter(to, 50));
        }
    }
    public Action HslideTo(double pos){
        return new SequentialAction(new HSlideToAction(pos));
    }
}
