package org.firstinspires.ftc.teamcode.util.mechanisms.linearslides;

import static org.firstinspires.ftc.teamcode.util.CustomActions.triggerAction;
import static java.lang.Integer.max;
import static java.lang.Integer.min;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class LinearSlide {
    protected DcMotor motor;

    private final int min_extend;
    private final int max_extend;
    private double zero_pos;
    public double max_error;

    public int target_pos;
    public Boolean within_error = false;

    /**
     * Class for using linear slides
     * @param motor the motor attached to the slide
     * @param max_extend the furthest value the slide can extend to
     * @param min_extend the lowest value the slide extends to
     * @param max_error the initial allowed distance (in ticks) from the target position
     * @param target_pos target_pos the initial encoder position to target
     */
    public LinearSlide(DcMotor motor, int min_extend, int max_extend, int target_pos, double max_error) {
        assert(max_extend > min_extend);
        this.motor = motor;
        this.min_extend = min_extend;
        this.max_extend = max_extend;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.target_pos = target_pos;
        this.max_error = max_error;
    }

    public int trimTicks(int ticks){
        return max(min(ticks, max_extend), min_extend);
    }

    /**
     * Stops any movement. Only use this function while the movement thread is paused.
     */
    public void stop(){
        motor.setPower(0);
    }

    /**
     * Determines the power given to the motor at a given moment. The function may do this in a variety of ways
     * @return The power given to the motor
     */
    abstract double powerFunction();

    /**
     * This method controls the linear slide throughout the program.
     */
    public void movementLoop(){
            assert(target_pos <= max_extend);
            assert(target_pos >= min_extend);

            within_error = (Math.abs(target_pos - getPosition()) < max_error);
            if (within_error) {
                this.stop();
            } else {
                motor.setPower(powerFunction());
            }
    }

    public void setMotorPower(double power){
        motor.setPower(power);
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
    public int getPosition(){
        return (int) (motor.getCurrentPosition() - zero_pos);
    }

    /**
     * Sets the slide to run towards a certain position asynchronously
     * @param pos the position to run to, in ticks
     * @param err the acceptable error, in ticks.
     */
    public void goTo(int pos, double err){
        target_pos = pos;
        motor.setTargetPosition(pos);
        max_error = err;
    }

    public void goTo(int pos){
        target_pos = pos;
    }

    /**
     * Waits for movement. This is breaking, so avoid using this when possible
     */
    public void waitForMovement(){while (!within_error);}

    public Action loopUntilDone() {
        return telemetryPacket -> {
            movementLoop();
            return !within_error;
        };
    }
}
