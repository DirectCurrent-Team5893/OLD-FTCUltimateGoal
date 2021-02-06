package org.firstinspires.ftc.teamcode.UltimateGoalComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;

public class WobbleArm extends RobotComponent {
    public DcMotor wobbleArm;

    public Servo lowerClasp;
    public Servo upperClasp;


    public final double POWER = .2;
     public final double REVERSE_POWER = .4;

    boolean buttonIsHeld  = false;

    boolean positionIn = true;


    public WobbleArm(robotBase BASE) {
        super(BASE);
        init();
    }
    void init(){
        wobbleArm = base.getMapper().mapMotor("wobbleArm");
        upperClasp = base.getMapper().mapServo("clasp");
        lowerClasp = base.getMapper().mapServo("lowerClasp");
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
                upperClasp.setPosition(.65);
                break;

            case INIT_POSITION:
            case CLOSED_POSITION:
                upperClasp.setPosition(.86);
                break;
        }
    }
    public void moveClaspInTeleop(boolean button){

        if(button && !buttonIsHeld){
            buttonIsHeld = true;
            if(!positionIn){
                upperClasp.setPosition(0);
                lowerClasp.setPosition(1);
            }
            else {
                upperClasp.setPosition(1);
                lowerClasp.setPosition(0);


            }
            positionIn = !positionIn;
        }
        if(!button){
            buttonIsHeld = false;
        }
    }

    @Override
    public void stop() {
        wobbleArm.setPower(0);
    }
}
