package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;

import java.util.function.Supplier;

public class CustomActions {
    public static void runBlocking(Action action) {
        FtcDashboard dash = FtcDashboard.getInstance();
        Canvas previewCanvas = new Canvas();
        action.preview(previewCanvas);

        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());

            running = action.run(packet);

            dash.sendTelemetryPacket(packet);
        }
    }

    public static Action foreverAction(Supplier<Action> action_supplier){
        return new Action() {
            Action current = action_supplier.get();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!current.run(telemetryPacket)) {
                    current = action_supplier.get();
                }
                return true;
            }
        };
    }

    public static Action foreverAction(Runnable f){
        return foreverAction(() -> new InstantAction(f::run));
    }

    public static Action futureAction(Supplier<Action> supplier){
        return new Action(){
            Action action = null;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (action == null) {
                    action = supplier.get();
                }
                return action.run(telemetryPacket);
            }
        };
    }

    public static Action triggerAction(Supplier<Boolean> condition){
        return telemetryPacket -> !condition.get();
    }
}
