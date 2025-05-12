package org.firstinspires.ftc.teamcode.util.customactions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;

public class FutureAction implements Action {
    Supplier<Action> action_supplier;
    Action action = null;
    public FutureAction(Supplier<Action> action_supplier) {
        this.action_supplier = action_supplier;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (action == null) {
           action = action_supplier.get();
        }
        return action.run(telemetryPacket);
    }
}
