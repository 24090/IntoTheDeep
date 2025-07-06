package org.firstinspires.ftc.teamcode.util.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Sweeper {
    public static double in_pos = 0.38;
    public static double out_pos = 0.55;

    ServoImplEx servo;
    boolean sweeper_is_out = false;

    public Sweeper(HardwareMap hwmap) {
        this.servo = hwmap.get(ServoImplEx.class, "sweeper_servo");
            servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void moveIn(){
        sweeper_is_out = false;
        servo.setPosition(in_pos);
    }

    public void moveOut(){
        sweeper_is_out = true;
        servo.setPosition(out_pos);
    }

    public void toggle(){
        if (sweeper_is_out) {
            moveIn();
        }
        else {
            moveOut();
        }
    }
}
