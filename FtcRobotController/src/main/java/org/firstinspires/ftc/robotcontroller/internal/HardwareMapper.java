package org.firstinspires.ftc.robotcontroller.internal;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareMapper {
    private robotBase base;

    public HardwareMapper(robotBase Robotbase){
        this.base = Robotbase;
    }

    public DcMotor mapMotor(final String NAME, final DcMotorSimple.Direction Direction, final DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        DcMotor temporaryMotor;
        temporaryMotor = base.getHardwaremap().dcMotor.get(NAME);
        temporaryMotor.setDirection(Direction);
        temporaryMotor.setZeroPowerBehavior(zeroPowerBehavior);

        return temporaryMotor;
    }
    public DcMotor mapMotor(final String NAME, final DcMotorSimple.Direction DIRECTION){
        DcMotor tempMotor;
        tempMotor = base.getHardwaremap().get(DcMotor.class, NAME);
        tempMotor.setDirection(DIRECTION);

        return tempMotor;
    }
    public DcMotor mapMotor(final String NAME){
        DcMotor temporaryMotor;
        temporaryMotor = base.getHardwaremap().get(DcMotor.class, NAME);

        return temporaryMotor;
    }
    public Servo mapServo(final String NAME){
        Servo temporaryServo;
        temporaryServo = base.getHardwaremap().servo.get(NAME);

        return temporaryServo;
    }
    public ModernRoboticsI2cGyro mapMRGyro(final String NAME){
        ModernRoboticsI2cGyro tempGyro;

        tempGyro = base.getHardwaremap().get(ModernRoboticsI2cGyro.class, NAME);

        return tempGyro;
    }
    public ModernRoboticsI2cRangeSensor mapMRRange(final String NAME){
        ModernRoboticsI2cRangeSensor tempRange;

        tempRange = base.getHardwaremap().get(ModernRoboticsI2cRangeSensor.class, NAME);
        tempRange.initialize();

        try {
            tempRange = base.getHardwaremap().get(ModernRoboticsI2cRangeSensor.class, NAME);
        }
        catch(Exception e){
        }
        return tempRange;
    }


}
