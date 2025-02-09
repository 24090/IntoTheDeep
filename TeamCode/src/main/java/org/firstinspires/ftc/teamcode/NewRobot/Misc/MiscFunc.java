package org.firstinspires.ftc.teamcode.NewRobot.Misc;

public class MiscFunc {
    public MiscFunc(){
    }
    public double clamp(double value, double min, double max) {
        if (value > max){
            value = max;
        }
        if (value < min){
            value = min;
        }
        return value;
    }
}
