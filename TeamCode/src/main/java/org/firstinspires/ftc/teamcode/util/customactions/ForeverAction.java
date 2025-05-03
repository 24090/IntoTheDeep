package org.firstinspires.ftc.teamcode.util.customactions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;

public class ForeverAction implements Action {
    Action action;
    public ForeverAction(InstantAction action){
        this.action = action;
    }
    public ForeverAction(InstantFunction function){
        this.action = new InstantAction(function);
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        action.run(telemetryPacket);
        return true;
    }
}
