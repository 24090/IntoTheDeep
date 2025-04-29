package org.firstinspires.ftc.teamcode.util.linearslides;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.GameMap;

import java.util.function.Supplier;

public class IntakeSlide extends LinearSlide {
    public static final double MAX_EXTEND = 1200;
    public static final double MIN_EXTEND = 0;
    /**
     * Class for using Intake Slide
     * @param hwmap the hardware map, used to find the motor
     * @param thread_ender end the loop when the function returns true
     */
    public IntakeSlide(HardwareMap hwmap) {
        super(hwmap.get(DcMotor.class, "intake_motor"), MIN_EXTEND, MAX_EXTEND, 0, 50);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void moveIn(){
        goTo(0);
    }
    public void moveOut(){
        goTo(1000);
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
        return (extension_from_center_in - GameMap.MinIntakeDistance)/(GameMap.MaxIntakeDistance-GameMap.MinIntakeDistance) * (MAX_EXTEND - MIN_EXTEND) + MIN_EXTEND;
    }
}
