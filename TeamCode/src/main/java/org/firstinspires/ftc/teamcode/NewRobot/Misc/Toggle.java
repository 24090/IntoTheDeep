package org.firstinspires.ftc.teamcode.NewRobot.Misc;

public class Toggle {
    boolean toggle;
    public enum initToggleState{STARTTRUE, STARTFALSE}
    public Toggle(initToggleState inittogglestate){
        toggle = inittogglestate == initToggleState.STARTTRUE;
    }
    public boolean poke(){
        toggle = !toggle;
        return toggle;
    }
}
