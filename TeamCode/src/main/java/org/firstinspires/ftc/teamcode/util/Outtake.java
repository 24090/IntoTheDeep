package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.linearslides.OuttakeSlide;

public class Outtake {
    Servo outtake_servo;
    public OuttakeSlide linear_slide;
    public enum state {
        READY_TRANSFER,
        READY_RELEASE,
        RELEASE
    }
    state target_state = state.READY_TRANSFER;
    state current_state = state.READY_TRANSFER;
    public Outtake(Servo outtake_servo, DcMotor motor) {
        this.outtake_servo = outtake_servo;
        this.linear_slide = new OuttakeSlide(motor);
    }
    public boolean stateMachine(){
        switch (target_state) {
            case READY_TRANSFER:
                close();
                if (linear_slide.extendToIter(0, 50) && (outtake_servo.getPosition() <= 0)) {
                    linear_slide.motor.setPower(0);
                    current_state = state.READY_TRANSFER;
                }
                break;
            case READY_RELEASE:
                if (current_state != state.READY_TRANSFER){break;}
                if  (linear_slide.extendToIter(4650, 50)) {
                    linear_slide.stop();
                    current_state = state.READY_RELEASE;
                }
            case RELEASE:
                if ((current_state != state.READY_RELEASE)){break;}
                open();
                // slide should already be up from READY_RELEASE
                if (outtake_servo.getPosition() >= 180) {
                    current_state = state.RELEASE;
                    // may need to add in a wait here
                    target_state = state.READY_TRANSFER;
                    return true;
                }
        }
        return false;
    }
    public void open(){
        outtake_servo.setPosition(0);
    }
    public void close(){
        outtake_servo.setPosition(180);
    }
    public Thread up(){
        return linear_slide.moveUp();
    }
    public Thread down(){
        return linear_slide.moveDown();
    }
    public void readyTransfer(){
        close();
        down();
    }
    public void scoreProcess() throws InterruptedException {
        up();
        open();
        Thread.sleep(1000);
        readyTransfer();
    }
}
