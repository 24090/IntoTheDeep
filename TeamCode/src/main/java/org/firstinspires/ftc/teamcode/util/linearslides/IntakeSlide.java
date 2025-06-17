package org.firstinspires.ftc.teamcode.util.linearslides;

import static java.lang.Double.max;
import static java.lang.Double.min;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.GameMap;
import org.firstinspires.ftc.teamcode.util.Intake;

public class IntakeSlide extends LinearSlide {
    public static final double MAX_EXTEND = 320;
    public static final double MIN_EXTEND = 0;
    /**
     * Class for using Intake Slide
     * @param hwmap the hardware map, used to find the motor
     */
    public IntakeSlide(HardwareMap hwmap) {
        super(hwmap.get(DcMotor.class, "intake_motor"), MIN_EXTEND, MAX_EXTEND, 0, 10);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void moveIn(){
        goTo(0);
    }
    public void moveOut(){
        goTo(319);
    }

    public double powerFunction(){
        double distance = target_pos - getPosition();
        return Math.signum(distance) * 0.4 + distance/600;
    }
    /**
     * Converts
     * @param extension_from_center_in extension distance from the center of the robot, measured in inches
     * @return number of ticks to extend linear slide to
     */
    public double inToTicks(double extension_from_center_in){
        return (extension_from_center_in - Intake.MinDistance)/(Intake.MaxDistance-Intake.MinDistance) * (MAX_EXTEND - MIN_EXTEND) + MIN_EXTEND;
    }

    public double trimIn(double in){
        return max(min(in, Intake.MaxDistance), Intake.MinDistance);
    }
}
