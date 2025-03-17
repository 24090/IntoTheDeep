package org.firstinspires.ftc.teamcode.util.linearslides;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.GameMap;

public class IntakeSlide extends LinearSlide {
    /**
     * Class for using Intake Slide
     */
    public IntakeSlide(DcMotor motor) {
        super(motor, 1100, 0, 0);
    }
    public void moveIn(){
        goTo(0, 50);
    }
    public void moveOut(){
        goTo(1000, 50);
    }

    public double powerFunction(){
        double distance = target_pos - getPosition();
        return Math.signum(distance) * 0.3 + distance * 6;
    }
    /**
     * Converts
     * @param extension_from_center_in extension distance from the center of the robot, measured in inches
     * @return number of ticks to extend linear slide to
     */
    public double inToTicks(double extension_from_center_in){
        return (extension_from_center_in - GameMap.MinIntakeDistance)/(GameMap.MaxIntakeDistance-GameMap.MinIntakeDistance) * (1100);
    }
}
