package org.firstinspires.ftc.teamcode.util.linearslides;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearSlide {
    public DcMotor motor;
    double min_extend;
    double max_extend;
    double minimum_power;
    double inherent_power;
    Thread current_action = null;

    /**
     * Class for using linear slides\
     * @param motor the motor attached to the slide
     * @param max_extend the furthest value the slide can extend to
     * @param min_extend the lowest value the slide extends to
     * @param minimum_power the minimum power required to move
     */
    public LinearSlide(DcMotor motor, double max_extend, double min_extend, double minimum_power) {
        this.motor = motor;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.min_extend = min_extend;
        this.max_extend = max_extend;
        this.minimum_power = minimum_power;
    }

    public void stop(){
        motor.setPower(0);
    }

    /**
     * Sets the slide speed to move towards a certain point.
     * Should be used while iterating in a while loop. Make sure to power off motor at end.
     * @param pos desired position, in ticks
     * @param max_error max acceptable distance from position, in ticks
     * @return whether or not the  position is within the error
     */
    public boolean extendToIter(double pos, double max_error){
        if ((pos < min_extend) || (pos > max_extend)){
            throw new Error("Requested slide extension out of bounds.");
        }
        double distance = pos - motor.getCurrentPosition();
        motor.setPower(6 * (distance / (max_extend - min_extend)) + (minimum_power * Math.signum(distance)));
        return Math.abs(distance) < max_error;
    }

    public void extendToBreaking(double pos, double max_error){
        while (!extendToIter(pos, max_error)){}
        stop();
    }

    public Thread extendToAsync(double pos, double max_error){
        if (current_action != null && current_action.isAlive()){
            current_action.interrupt();
            current_action = null;
        }
        Thread thread = new Thread(() -> extendToBreaking(pos, max_error));
        thread.start();
        current_action = thread;
        return thread;
    }
}
