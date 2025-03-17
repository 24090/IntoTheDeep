package org.firstinspires.ftc.teamcode.util.linearslides;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class LinearSlide {
    public DcMotor motor;
    private final double min_extend;
    private final double max_extend;
    private double zero_pos;
    public double max_error;
    public double target_pos;
    public Boolean within_error;
    Thread movement_thread;

    /**
     * Class for using linear slides
     * @param motor the motor attached to the slide
     * @param max_extend the furthest value the slide can extend to
     * @param min_extend the lowest value the slide extends to
     * @param target_pos the initial target position
     */
    public LinearSlide(DcMotor motor, double max_extend, double min_extend, double target_pos) {
        this.motor = motor;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.min_extend = min_extend;
        this.max_extend = max_extend;
        this.target_pos = target_pos;
        this.max_error = 50;
        movement_thread = new Thread(this::movementLoop);
    }

    /**
     * Stops any movement. Only use this function while the movement thread is paused.
     */
    public void stop(){
        motor.setPower(0);
    }
    /**
     * Starts the movement thread. To actually stop the motor, make sure to call stop() after this.
     */
    public void stopThread(){
        movement_thread.interrupt();
    }

    /**
     * Starts the movement thread. You don't need to use this function on initialization
     */
    public void startThread(){
        movement_thread.start();
    }

    /**
     * Determines the power given to the motor at a given moment. The function may do this in a variety of ways
     * @return The power given to the motor
     */
    abstract double powerFunction();

    /**
     * This method controls the linear slide throughout the program.
     */
    private void movementLoop(){
        while (true) {
            assert target_pos < max_extend: "requested extension too high";
            assert target_pos > min_extend: "requested extension too low";
            within_error = (Math.abs(target_pos - zero_pos) < max_error);
            if (within_error) {
                this.stop();
            } else {
                motor.setPower(powerFunction());
            }
        }

    }

    /**
     * Sets the slide to be zero at this position
     */
    public void setZero(){
        zero_pos = motor.getCurrentPosition();
    }
    /**
     * Gets the position of the slide, adjusted based on runtime-set zero position
     * @return the position of the slide
     */
    public double getPosition(){
        return (motor.getCurrentPosition() - zero_pos);
    }
    /**
     * Sets the slide to run towards a certain position asynchronously
     * @param pos the position to run to, in ticks
     * @param err the acceptable error, in ticks.
     */
    public void goTo(double pos, double err){
        target_pos = pos;
        max_error = err;
    }
    /**
     * Waits for movement. This is breaking, so avoid using this when possible
     */
    public void waitForMovement(){
        while (!within_error){}
    }
}
