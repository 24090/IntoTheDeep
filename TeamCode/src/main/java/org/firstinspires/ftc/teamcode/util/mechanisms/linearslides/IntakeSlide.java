package org.firstinspires.ftc.teamcode.util.mechanisms.linearslides;

import static java.lang.Double.max;
import static java.lang.Double.min;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.mechanisms.intake.Intake;

public class IntakeSlide extends LinearSlide {
    public static final int MAX_EXTEND = 300;
    public static final int MIN_EXTEND = 0;
    /**
     * Class for using Intake Slide
     * @param hwmap the hardware map, used to find the motor
     */
    public IntakeSlide(HardwareMap hwmap) {
        super(hwmap.get(DcMotor.class, "intake_motor"), MIN_EXTEND, MAX_EXTEND, 0, 20);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void moveIn(){
        goTo(0, 20);
    }
    public void goTo(int pos){
        goTo(pos, 20);
    }
    public void moveOut(){
        goTo(300);
    }

    public double powerFunction(){
        double distance = target_pos - getPosition();
        return distance/200.0;
    }

    public void stop(){
        if (target_pos > 290) {
            setMotorPower(0.1 + 0.1 * (target_pos - getPosition())/20);
        } else {
            setMotorPower(0.1 * (target_pos - getPosition())/20);
        }
    }
    /**
     * Converts
     * @param extension_from_center_in extension distance from the center of the robot, measured in inches
     * @return number of ticks to extend linear slide to
     */
    public int inToTicks(double extension_from_center_in){
        return (int) ((extension_from_center_in - Intake.MinDistance)/(Intake.MaxDistance-Intake.MinDistance) * (MAX_EXTEND - MIN_EXTEND) + MIN_EXTEND);
    }

    public double trimIn(double in){
        return max(min(in, Intake.MaxDistance), Intake.MinDistance);
    }

}
