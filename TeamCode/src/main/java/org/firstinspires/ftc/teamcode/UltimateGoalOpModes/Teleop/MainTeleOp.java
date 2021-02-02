package org.firstinspires.ftc.teamcode.UltimateGoalOpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FullBase;

@TeleOp(name = "Main TeleOp", group = "Linear Opmode")
public class MainTeleOp extends LinearOpMode {

    private ElapsedTime     runtime = new ElapsedTime();
    FullBase Base ;
    boolean flickerPositon =true;
    int speed = 8000;
    boolean dpadUpHeld = false;
    boolean dpadDownHeld = false;
    @Override
    public void runOpMode() {
        Base = new FullBase(telemetry,this, hardwareMap, true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Base.init();
        telemetry.addLine("done with init");
        waitForStart();
        while (opModeIsActive()){
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            Base.drivetrain.drive(forward,right,turn);

            Base.hopper.moveHopperInTeleop(gamepad2.a);

            flickerPositon = Base.hopper.moveFlicker(gamepad2.b, flickerPositon, runtime);
            if(Base.hopper.hopperMover.getPosition() != 0.86)
            {
            Base.intake.suck(gamepad1.right_trigger,Math.abs(gamepad1.left_trigger)>.1);
            }
            Base.intake.spit(gamepad1.left_trigger,Math.abs(gamepad1.right_trigger)>.1);
            if(gamepad2.right_bumper) Base.shooter.getToTargetSpeed(speed);
            //Base.shooter.shoot(gamepad1.x);
            if(gamepad2.dpad_up && !dpadUpHeld){ speed = 6100; dpadUpHeld = true;}
            if(!gamepad2.dpad_up) dpadUpHeld = false;
            if(gamepad2.dpad_down && !dpadDownHeld){ speed = 5900; dpadDownHeld = true;}
            if(!gamepad2.dpad_down) dpadDownHeld = false;
            if(gamepad1.dpad_right){Base.wobbleArm.wobbleArm.setPower(.2);}
            else if(gamepad1.dpad_left){Base.wobbleArm.wobbleArm.setPower(-.2);}
            else {Base.wobbleArm.wobbleArm.setPower(0);}
            Base.wobbleArm.moveClaspInTeleop(gamepad1.left_bumper);


            Base.getTelemetry().addLine("Speed: " + speed);
            Base.getTelemetry().addData("Shooter Encoders:",Base.shooter.ShooterWheel.getCurrentPosition());
            Base.getTelemetry().update();
            Base.shooter.stop(gamepad2.left_bumper);
        }
    }
}
