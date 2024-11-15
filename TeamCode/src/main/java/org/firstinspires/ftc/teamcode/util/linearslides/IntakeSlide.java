package org.firstinspires.ftc.teamcode.util.linearslides;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSlide extends LinearSlide {
    /**
     * Class for using Intake Slide
     */
    public IntakeSlide(DcMotor motor) {
        super(motor, 1100, 0, 0.15);
    }
    public Thread moveIn(){
        return extendToAsync(0, 50);
    }
    public Thread fullOut(){
        return extendToAsync(1000, 50);
    }
}
