package org.firstinspires.ftc.teamcode.util.customactions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;

public class TriggerAction implements Action {
    Supplier<Boolean> condition;

    /**
     * Creates a new action that ends when condition is true
     * @param condition
     */
    public TriggerAction(Supplier<Boolean> condition) {
        this.condition = condition;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return !condition.get();
    }
}
