package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(group = "testing", name = "Move Any Servo")
public class MoveAnyMotor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        String[] motor_names = {
                "intake_motor"
        };
        int motor_number = 0;
        double pos = 0;
        double last_time = 0;
        boolean just_clicked = false;
        waitForStart();
        while(opModeIsActive()){
            telemetry.addLine("A - Turn on motor");
            telemetry.addLine("B - Turn off motor");
            telemetry.addLine("X - Next motor");
            telemetry.addLine("Y - Previous motor");
            telemetry.addLine("DPAD UP - increment power");
            telemetry.addLine("DPAD DOWN - increment power");
            if (gamepad1.a) {
                hardwareMap.get(DcMotor.class, motor_names[motor_number]).setPower(pos);
            } else if (gamepad1.b) {
                hardwareMap.get(DcMotor.class, motor_names[motor_number]).setPower(0);
            } else if (gamepad1.x && !just_clicked) {
                motor_number = (motor_number + 1)%motor_names.length;
                just_clicked = true;
            } else if (gamepad1.y && !just_clicked) {
                motor_number = (motor_number - 1) % motor_names.length;
                just_clicked = true;
            } else if (!gamepad1.x && !gamepad1.y && just_clicked) {
                just_clicked = false;
            } else if (gamepad1.dpad_up) {
                // 5 second from 0 -> 1
                pos = Math.min(pos + (time - last_time) / 5 , 1);
                hardwareMap.get(DcMotor.class, motor_names[motor_number]).setPower(pos);
            } else if (gamepad1.dpad_down) {
                // 5 second from 1 -> 0
                pos = Math.max(pos - (time - last_time) / 5 , -1);
                hardwareMap.get(DcMotor.class, motor_names[motor_number]).setPower(pos);
            }
            telemetry.addData("Current Motor", motor_names[motor_number]);
            telemetry.addData("pos", pos);
            last_time = time;
            telemetry.update();
        }
    }
}
