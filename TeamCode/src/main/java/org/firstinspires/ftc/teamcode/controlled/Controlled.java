package org.firstinspires.ftc.teamcode.controlled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import java.util.List;
@TeleOp
public class Controlled extends LinearOpMode{
    public void runOpMode(){

        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap,StandardTrackingWheelLocalizer.getWheelPositions() , );
    }

}
