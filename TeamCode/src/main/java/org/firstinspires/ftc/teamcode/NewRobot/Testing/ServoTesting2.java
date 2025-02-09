package org.firstinspires.ftc.teamcode.NewRobot.Testing;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NewRobot.Misc.MiscFunc;
import org.firstinspires.ftc.teamcode.NewRobot.Util.IntegratedMechs.NewMechActions;
import org.firstinspires.ftc.teamcode.NewRobot.Util.LinearSlides.NewIntakeSlide;
import org.firstinspires.ftc.teamcode.NewRobot.Util.Servo.Servos;
import org.firstinspires.ftc.teamcode.NewRobot.Misc.Toggle;

@TeleOp
public class ServoTesting2 extends LinearOpMode {
    public double fromDegrees1(double input){
        return (input/90) + 0.5;
    }
    public double fromDegrees2(double input){
        return (input/165);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        NewIntakeSlide linear2 = new NewIntakeSlide(hardwareMap, "linear2");

        NewMechActions mech = new NewMechActions(linear2);
        MiscFunc miscFunc = new MiscFunc();
        Toggle toggle1 = new Toggle(Toggle.initToggleState.STARTTRUE);
        Toggle toggle2 = new Toggle(Toggle.initToggleState.STARTFALSE);
        Toggle toggle3 = new Toggle(Toggle.initToggleState.STARTFALSE);
        Servos servo1 = new Servos("servo1", hardwareMap, 0.41, 0.41, 0.63, 0, 0);
        Servos servo2 = new Servos("servo2", hardwareMap, this.fromDegrees1(0), 0.24, 0.88, -90, 90);
        Servos servo3 = new Servos("servo3", hardwareMap, this.fromDegrees1(2), 0.13, 0.76, -90, 90);
        Servos servo4 = new Servos("servo4", hardwareMap, this.fromDegrees2(13.5), 0, 0.6, 0, 165);
        Servos servo5 = new Servos("servo5", hardwareMap, 0.5, 0.36, 0.6, 0, 0);
        Servos servo6 = new Servos("servo6", hardwareMap, 0, 0, 1, -90, 90);
        Servos servo7 = new Servos("servo7", hardwareMap, 0, 0, 1, -90, 90);
        servo7.reverse();
        double j = 0;
        double k = 2;
        double l = 13.5;
        double o = -90;
        int i = 0;
        waitForStart();
        while (opModeIsActive()){
            k = miscFunc.clamp(gamepad1.y ? k + 0.3 : gamepad1.a ? k - 0.3 : k, -90, 90);
            servo3.setPosInDegrees(k);
            j = miscFunc.clamp(gamepad1.dpad_left ? j + 0.3 : gamepad1.dpad_right ? j - 0.3 : j, -90, 90);
            servo2.setPosInDegrees(j);
            l = miscFunc.clamp(gamepad1.dpad_up ? l + 0.3 : gamepad1.dpad_down ? l - 0.3 : l, 0, 165);
            servo4.setPosInDegrees(l);
            o = miscFunc.clamp(gamepad1.left_stick_y > 0 ? o + 0.3 : gamepad1.left_stick_y < 0 ? o - 0.3 : o, -90, 90);
            servo6.setPosInDegrees(o);
            servo7.setPosInDegrees(o);
            if (gamepad1.b){
                servo1.setPosition(toggle1.poke() ? 1 : 0);
                while (gamepad1.b && opModeIsActive()){
                }
            }
            if (gamepad1.x){
                servo5.setPosition(toggle2.poke() ? 1 : 0);
                while (gamepad1.x && opModeIsActive()){
                }
            }
            if (gamepad1.left_bumper && i == 0){
                k=90;
                l=165;
                j=0;
                servo3.setPosInDegrees(k);
                servo2.setPosInDegrees(j);
                servo4.setPosInDegrees(l);
                Actions.runBlocking(mech.HslideTo(-1200));
                i=1;
            }
            if (gamepad1.right_bumper && i == 1){
                k=-45;
                l=20;
                j=0;
                servo3.setPosInDegrees(k);
                servo2.setPosInDegrees(j);
                servo4.setPosInDegrees(l);
                Actions.runBlocking(mech.HslideTo(-100));
                i=0;
            }
            telemetry.addData("servo1pos", servo1.getPosition());
            telemetry.addData("servo2pos", servo2.getPosition());
            telemetry.addData("servo3pos", servo3.getPosition());
            telemetry.addData("servo4pos", servo4.getPosition());
            telemetry.addData("servo5pos", servo5.getPosition());
            telemetry.addData("servo6pos", servo6.getPosition());
            telemetry.addData("servo7pos", servo7.getPosition());
            telemetry.addData("pos", linear2.getPosition());
            telemetry.update();
        }

    }
}
