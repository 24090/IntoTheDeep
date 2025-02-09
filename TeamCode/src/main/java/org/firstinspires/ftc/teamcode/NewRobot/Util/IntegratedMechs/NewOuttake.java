package org.firstinspires.ftc.teamcode.NewRobot.Util.IntegratedMechs;

import org.firstinspires.ftc.teamcode.NewRobot.Util.LinearSlides.NewIntakeSlide;
import org.firstinspires.ftc.teamcode.NewRobot.Util.LinearSlides.NewOuttakeSlide;

public class NewOuttake {
    NewOuttakeSlide outtakeSlide1;
    NewOuttakeSlide outtakeSlide2;
    public NewOuttake(NewOuttakeSlide outtakeSlide1, NewOuttakeSlide outtakeSlide2){
        this.outtakeSlide1 =outtakeSlide1;
        this.outtakeSlide2 =outtakeSlide2;
    }
    public void outtakeSlideTo(double pos){
        outtakeSlide1.slideTo(-pos);
        outtakeSlide2.slideTo(pos);
    }
}
