package org.firstinspires.ftc.teamcode.UltimateGoalComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;

public class WobbleArm extends RobotComponent {
    public DcMotor wobbleArm;

    public Servo Clasp;


    public final double POWER = .2;
     public final double REVERSE_POWER = .4;

    boolean buttonIsHeld  = false;
    boolean otherButtonIsHeld = false;

    boolean positionIn = true;
    boolean positionUp = true;

    wobbleGoalStates[] WGPostions = {wobbleGoalStates.WHEELS_OFF_CLASP_OUT, wobbleGoalStates.WHEELS_ON_CLASP_OUT,
            wobbleGoalStates.WHEELS_ON_CLASP_IN, wobbleGoalStates.WHEELS_OFF_CLASP_IN};
    int counter = 0;
    wobbleGoalStates WGPosition = WGPostions[counter];


    public WobbleArm(robotBase BASE) {
        super(BASE);
        init();
    }
    void init(){
        wobbleArm = base.getMapper().mapMotor("wobbleArm");
        Clasp = base.getMapper().mapServo("clasp");
    }
    public enum POSITION {OPEN_POSITION, CLOSE_POSITION}

    public void runToPosition(POSITION target){
        switch(target){
            case OPEN_POSITION:
                wobbleArm.setPower(-POWER);
              //  base.getOpMode().sleep(1000);
            case CLOSE_POSITION:
                wobbleArm.setPower(POWER);
        }
    }
    public enum Position { OPEN_POSITION, CLOSED_POSITION, INIT_POSITION };
    public void setClaspPosition ( Position targetPositon) {
        switch (targetPositon) {
            case OPEN_POSITION:
                Clasp.setPosition(.6);
                break;

            case INIT_POSITION:
            case CLOSED_POSITION:
                Clasp.setPosition(1);
                break;
        }
    }
    public enum wobbleGoalStates {
        WHEELS_ON_CLASP_OUT, WHEELS_OFF_CLASP_OUT, WHEELS_ON_CLASP_IN, WHEELS_OFF_CLASP_IN
    }
    public void moveClaspInTeleop(boolean button){

        if(button && !buttonIsHeld){
            buttonIsHeld = true;
            if(!positionIn){
                Clasp.setPosition(1);
            }
            else {
                Clasp.setPosition(0.6);
            }
            positionIn = !positionIn;


        }
        if(!button){
            buttonIsHeld = false;
        }
    }
//    public void moveRCClaspInTeleop(boolean otherButton){
//
//        if(otherButton && !otherButtonIsHeld){
//            otherButtonIsHeld = true;
//            if(!positionUp){
//              rcClasp.setPosition(1);
//            }
//            else {
//              rcClasp.setPosition(0.5);
//            }
//            positionUp = !positionUp;
//        }
//        if(!otherButton){
//            otherButtonIsHeld = false;
//        }
//    }
//
    @Override
    public void stop() {
        wobbleArm.setPower(0);
    }
}
