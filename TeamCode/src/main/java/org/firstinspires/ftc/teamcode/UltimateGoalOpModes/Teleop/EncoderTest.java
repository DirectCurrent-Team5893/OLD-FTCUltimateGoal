package org.firstinspires.ftc.teamcode.UltimateGoalOpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FullBase;

@TeleOp(name = "Encoder Test", group = "Linear Opmode")
public class EncoderTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    FullBase Base ;


    int speedArray[] = {3570, 3910, 4165, 4335, 4675, 5015, 5100};
    int arrayCounter = 3;
    int arrayMax = 6;
    int speed = speedArray[arrayCounter];
    double targetSpeedDecimal = 0;

    boolean dpadUpHeld = false;
    boolean dpadDownHeld = false;
    boolean dpadRightHeld = false;
    boolean dpadLeftHeld  = false;
    boolean gamepad1XHeld = false;
    boolean gamepad1YHeld = false;

    boolean flickerPositon =true;
    boolean firstTime = true;
    boolean slowMode = false;
    double initTime;
    int initPos;

    @Override
    public void runOpMode() {
        Base = new FullBase(telemetry,this, hardwareMap, true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Base.init();
        telemetry.addLine("done with init");
        waitForStart();
        while (opModeIsActive()){
            String Telemetry = String.format("Front Right: %d\n Front Left: %d\n Back Right: %d\n Back Left: %d\n Gyro: %d", Base.drivetrain.frontRight.getCurrentPosition(),
                    Base.drivetrain.frontLeft.getCurrentPosition(),Base.drivetrain.backRight.getCurrentPosition(),Base.drivetrain.backLeft.getCurrentPosition(), Base.drivetrain.gyroSensor.getHeading());
            telemetry.addLine(Telemetry);
            telemetry.update();

        }
    }
}
