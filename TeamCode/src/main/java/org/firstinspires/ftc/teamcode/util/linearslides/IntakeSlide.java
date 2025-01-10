package org.firstinspires.ftc.teamcode.util.linearslides;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.GameMap;

public class IntakeSlide extends LinearSlide {
    /**
     * Class for using Intake Slide
     */
    public IntakeSlide(DcMotor motor) {
        super(motor, 1100, -50, 0.15);
    }
    public Thread moveIn(){
        return extendToAsync(0, 50);
    }
    public Thread fullOut(){
        return extendToAsync(1000, 50);
    }
    public double inToTicks(double extension_from_center_in){
        return (extension_from_center_in - GameMap.MinIntakeDistance)/(GameMap.MaxIntakeDistance-GameMap.MinIntakeDistance) * (1100 - -50) + -50;
    }
}
