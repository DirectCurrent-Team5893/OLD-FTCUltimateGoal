package org.firstinspires.ftc.teamcode.UltimateGoalOpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FullBase;

@TeleOp(name = "Main TeleOp", group = "Linear Opmode")
public class MainTeleOp extends LinearOpMode {

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
            //drivetrain
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            Base.drivetrain.drive(forward, right, turn, slowMode);

            //hopper
            Base.hopper.moveHopperInTeleop(gamepad2.a);

            flickerPositon = Base.hopper.moveFlicker(gamepad2.b, flickerPositon, runtime);
            if(Base.hopper.hopperMover.getPosition() != 0.86)
            {
            Base.intake.suck(gamepad1.right_trigger ,Math.abs(gamepad1.left_trigger)>.1);
            }
            Base.intake.spit(gamepad1.left_trigger * .8,Math.abs(gamepad1.right_trigger)>.1);
            //Turns on shooter
            if(gamepad2.right_bumper) targetSpeedDecimal = Base.shooter.getToTargetSpeed(speed);


            // Iterates through preset speeds
            if(gamepad2.dpad_up && !dpadUpHeld){ if(arrayCounter < arrayMax){speed = speedArray[++arrayCounter];} dpadUpHeld = true;}
            if(!gamepad2.dpad_up) dpadUpHeld = false;

            if(gamepad2.dpad_down && !dpadDownHeld){ if(arrayCounter > 0) {speed= speedArray[--arrayCounter];} dpadDownHeld = true;}
            if(!gamepad2.dpad_down) dpadDownHeld = false;

            //Increments through slight increments 10
            if(gamepad2.dpad_right && !dpadRightHeld){ speed += 10; dpadRightHeld = true;}
            if(!gamepad2.dpad_right) dpadRightHeld = false;

            if(gamepad2.dpad_left && !dpadLeftHeld){ speed -= 10; dpadLeftHeld = true;}
            if(!gamepad2.dpad_left) dpadLeftHeld = false;

            Base.intake.moveArmInTeleop(gamepad1.x);

//            if(gamepad2.dpad_down && !dpadDownHeld){ if(arrayCounter > 0) {speed= speedArray[--arrayCounter];} dpadDownHeld = true;}
//            if(!gamepad2.dpad_down) dpadDownHeld = false;
//
//            if(gamepad2.dpad_down && !dpadDownHeld){ if(arrayCounter > 0) {speed= speedArray[--arrayCounter];} dpadDownHeld = true;}
//            if(!gamepad2.dpad_down) dpadDownHeld = false;

//            if(gamepad1.x && !gamepad1XHeld)/*{Base.shootPowerShots();  */		throw new RuntimeException("You Broke IT");
//                gamepad1XHeld = true;}
            if(!gamepad1.x) gamepad1XHeld = false;
            if(gamepad1.y && !gamepad1YHeld){ slowMode = !slowMode;  gamepad1YHeld = true;}
            if(!gamepad1.y) gamepad1YHeld = false;

           if(gamepad1.dpad_left){Base.wobbleArm.wobbleArm.setPower(1);}
            else if(gamepad1.dpad_right){Base.wobbleArm.wobbleArm.setPower(-.5);}
            else if(gamepad2.left_stick_y < -0.1){Base.wobbleArm.wobbleArm.setPower(1);}
            else if(gamepad2.left_stick_y > 0.1){Base.wobbleArm.wobbleArm.setPower(-.5);}
            else {Base.wobbleArm.wobbleArm.setPower(0);}
            Base.wobbleArm.moveClaspInTeleop(gamepad1.left_bumper);
//            Base.wobbleArm.moveRCClaspInTeleop(gamepad1.x);

            if(firstTime || Base.getCurrentRPM(initTime, this.time, initPos, Base.shooter.ShooterWheel.getCurrentPosition()))
            {
                initTime = this.time;
                initPos = Base.shooter.ShooterWheel.getCurrentPosition();
                firstTime = false;

            }
            Base.getTelemetry().addLine("Speed: " + speed);
            Base.getTelemetry().addLine("Target Speed: " + targetSpeedDecimal);
            Base.getTelemetry().addData("Shooter Encoders:",Base.shooter.ShooterWheel.getCurrentPosition());
            Base.getTelemetry().addData("Angle: ", Base.drivetrain.gyroSensor.getIntegratedZValue());
//            Base.getTelemetry().addData("Distance:", Base.drivetrain.distance(DistanceUnit.INCH));
//            Base.getTelemetry().addData("Edited Distance: ", Base.drivetrain.customDistanceInInches());
            Base.getTelemetry().update();
            Base.shooter.stop(gamepad2.left_bumper);
        }
    }
}
