package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(group = "testing", name = "Set Any Servo")
public class SetAnyServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        String[] servo_names = {
                "claw_servo", "wrist_servo_left", "wrist_servo_right", "elbow_servo_left", "elbow_servo_right", "outtake_servo"
        };
        int servo_number = 0;
        double pos = 0;
        double last_time = 0;
        boolean just_clicked = false;
        waitForStart();
        while(opModeIsActive()){
            telemetry.addLine("A - Turn on servo");
            telemetry.addLine("B - Turn off servo");
            telemetry.addLine("X - Next servo");
            telemetry.addLine("Y - Previous servo");
            telemetry.addLine("DPAD UP - increment position");
            telemetry.addLine("DPAD DOWN - increment position");
            if (gamepad1.a) {
                hardwareMap.get(ServoImplEx.class, servo_names[servo_number]).setPwmRange(new PwmControl.PwmRange(500, 2500));
                hardwareMap.get(Servo.class, servo_names[servo_number]).getController().pwmEnable();
            } else if (gamepad1.b) {
                hardwareMap.get(Servo.class, servo_names[servo_number]).getController().pwmDisable();
            } else if (gamepad1.x && !just_clicked) {
                servo_number = (servo_number + 1)%servo_names.length;
                just_clicked = true;
            } else if (gamepad1.y && !just_clicked) {
                servo_number = (servo_number - 1) % servo_names.length;
                just_clicked = true;
            } else if (!gamepad1.x && !gamepad1.y && just_clicked) {
                just_clicked = false;
            } else if (gamepad1.dpad_up) {
                // 5 second from 0 -> 1
                pos = Math.min(pos + (time - last_time) / 5 , 1);
                hardwareMap.get(Servo.class, servo_names[servo_number]).setPosition(pos);
            } else if (gamepad1.dpad_down) {
                // 5 second from 1 -> 0
                pos = Math.max(pos - (time - last_time) / 5 , 0);
                hardwareMap.get(Servo.class, servo_names[servo_number]).setPosition(pos);
            }
            telemetry.addData("Current Servo", servo_names[servo_number]);
            telemetry.addData("pos", pos);
            last_time = time;
            telemetry.update();
        }
    }
}
