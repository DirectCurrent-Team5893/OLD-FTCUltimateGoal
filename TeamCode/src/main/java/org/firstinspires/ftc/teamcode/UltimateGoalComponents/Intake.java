package org.firstinspires.ftc.teamcode.UltimateGoalComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;

public class Intake extends RobotComponent {

   public DcMotor intakeLeft;
   public DcMotor intakeRight;

    public Servo intakeArm;

    protected final double UP_POSITION = .40;
    protected final double COLLECT_POSITION = 0.09;

    public boolean buttonIsHeld = false;
    public Intake(robotBase BASE) {
        super(BASE);

        intakeLeft = base.getMapper().mapMotor("intakeLeft");
        intakeRight = base.getMapper().mapMotor("intakeRight");
        intakeArm = base.getMapper().mapServo("servoArm");


    }

    public enum sucMode {
        ACTIVE, NOT_ACTIVE
    }

    public void setArmMode(sucMode suckmode){
        switch (suckmode){
            case ACTIVE:
                intakeArm.setPosition(COLLECT_POSITION);
                break;
            case NOT_ACTIVE:
                intakeArm.setPosition(UP_POSITION);
                break;
        }
    }
    public void moveArmInTeleop (boolean button){
        if(button && !buttonIsHeld) {
            if (intakeArm.getPosition() == UP_POSITION) {
                this.intakeArm.setPosition(COLLECT_POSITION);
            } else {
                this.intakeArm.setPosition(UP_POSITION);
            }
            buttonIsHeld = true;
        }
        if(!button) buttonIsHeld = false;
    }
    public void suck(double strength, boolean isSpitting){
        if(!isSpitting) {
            intakeLeft.setPower(strength);
            intakeRight.setPower(-strength);
        }
    }
    public void spit(double power, boolean isSucking){
        if(!isSucking) {
            intakeRight.setPower(power);
            intakeLeft.setPower(-power);
        }
    }
    @Override
    public void stop() {
        intakeRight.setPower(0);
        intakeLeft.setPower(0);
    }
}
