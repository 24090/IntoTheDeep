package org.firstinspires.ftc.teamcode.testing;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Claw;

@TeleOp(group = "testing", name = "ColorSensorTesting")
public class ColorSensorTesting extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        waitForStart();
        while (opModeIsActive()){
            int color_int =  claw.color_sensor.getNormalizedColors().toColor();
            float[] hsv = new float[3];
            Color.colorToHSV(color_int, hsv);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("h", hsv[0]);
            packet.put("s", hsv[1]);
            packet.put("v", hsv[2]);
            packet.put("color", claw.getSensedColor());
            dashboard.sendTelemetryPacket(packet);
        }

    }
}
