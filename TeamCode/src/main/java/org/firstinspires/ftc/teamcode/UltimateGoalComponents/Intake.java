package org.firstinspires.ftc.teamcode.UltimateGoalComponents;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;

public class Intake extends RobotComponent {

    DcMotor flyWheelShooter;

    public Intake(robotBase BASE) {
        super(BASE);

        flyWheelShooter = base.getMapper().mapMotor("flyWheel");
    }

    public void suck(double power, boolean isSpitting){
        if(!isSpitting)
        flyWheelShooter.setPower(power);
    }
    public void spit(double power, boolean isSucking){
        if(!isSucking)
        flyWheelShooter.setPower(-power);
    }
    @Override
    public void stop() {
        flyWheelShooter.setPower(0);
    }
}
