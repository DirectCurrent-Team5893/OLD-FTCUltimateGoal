package org.firstinspires.ftc.teamcode.UltimateGoalComponents;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;

public class WobbleArm extends RobotComponent {
    public DcMotor wobbleArm;

    double POWER = .5;

    public WobbleArm(robotBase BASE) {
        super(BASE);
        init();
    }
    void init(){
        wobbleArm = base().getMapper().mapMotor("wobbleArm");
    }
    public enum POSITION {OPEN_POSITION, CLOSE_POSITION}

    public void runToPosition(POSITION target){
        switch(target){
            case OPEN_POSITION:
                wobbleArm.setPower(POWER);

            case CLOSE_POSITION:
                wobbleArm.setPower(-POWER);
        }
    }

    @Override
    public void stop() {

    }
}
