package org.firstinspires.ftc.teamcode.NewRobot.Util.LinearSlides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NewIntakeSlide extends NewLinearSlide {
    public HardwareMap hardwareMap;
    public String motorName;
    double maxExtension;
    double minExtension;
    public NewIntakeSlide(HardwareMap hardwareMap, String motorName) {
        super(hardwareMap, motorName, 0, -1450, 0.2);
        this.hardwareMap = hardwareMap;
        this.motorName = motorName;
    }
    public Thread moveIn(){
        return extendToAsync(0, 50);
    }
    public Thread fullOut(){
        return extendToAsync(-1400, 50);
    }
    public Thread slideTo(double pos){
        return extendToAsync(pos, 50);
    }
}
