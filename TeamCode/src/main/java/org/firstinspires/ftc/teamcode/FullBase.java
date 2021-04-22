package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UltimateGoalComponents.*;

public class FullBase extends robotBase {
    public Drivetrain drivetrain;
    public Hopper hopper;
    public Shooter shooter;

    public WobbleArm wobbleArm;

    public Intake intake;

    private RobotComponent[] components = new RobotComponent[5];

    public double rpm = 0;

    public FullBase(Telemetry telemetry, LinearOpMode opMode, HardwareMap hardwaremap, boolean debugging) {
        super(telemetry, opMode, hardwaremap,debugging);
    }

    @Override
    public void init() {
        //create drivetrain
        telemetry.addLine("Drivetrain about to init");
        drivetrain = new Drivetrain(this);
        telemetry.addLine("drive inited");
        components[0] = drivetrain;

        //create hopper
        hopper = new Hopper(this);
        hopper.setHopperPosition(Hopper.Position.INIT_POSITION);
       hopper.setFlickerPosition(Hopper.flickerPosition.INIT_POSITION);
        components[1] = hopper;

        //create shooter
        shooter = new Shooter(this);
        components[2] = shooter;

        //create wobbleArm
        wobbleArm = new WobbleArm(this);
//        wobbleArm.setClaspPosition(WobbleArm.Position.INIT_POSITION);
        wobbleArm.Clasp.setPosition(.6);
        components[3] = wobbleArm;


        //create intake
        intake = new Intake (this);
        intake.setArmMode(Intake.sucMode.NOT_ACTIVE);
        components[4] = intake;
    }
    public void shootPowerShots(Drivetrain.NUM_OF_RINGS numOfRings){
        switch (numOfRings){
            case ONE:
                this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED, -1);
                this.opMode.sleep(600);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
                this.opMode.sleep(600);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
                this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,-2);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
                this.opMode.sleep(600);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
                this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,-3);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
                this.opMode.sleep(600);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
            case FOUR:
                this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED, -2);
                this.opMode.sleep(600);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
                this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,-3);
                this.opMode.sleep(600);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
                this.shooter.getToTargetSpeed(3700);
                this.opMode.sleep(800);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
                this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,-3);
                this.opMode.sleep(600);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
                this.opMode.sleep(600);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
            case ZERO:
                this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED, -3);
                this.opMode.sleep(600);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
                this.opMode.sleep(600);
                this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,-5);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
                this.opMode.sleep(2000);
                this.shooter.getToTargetSpeed(3800);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
                this.opMode.sleep(600);
                this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,-7);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
                this.opMode.sleep(2000);
                this.shooter.getToTargetSpeed(3750);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
                this.opMode.sleep(600);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        }
    }

    public void shootShots(){

//        final int TIME_TO_GO_OUT = 250;
//        final int TIME_TO_GET_SPEED = 600;
//        this.debugTelemetery("Get to target : Shooter");
//        this.shooter.getToTargetSpeed(5900);
//        this.debugTelemetery("Shoot One");
//        this.opMode.sleep(TIME_TO_GET_SPEED);
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
//        this.opMode.sleep(TIME_TO_GO_OUT);
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
//        this.debugTelemetery("Shoot Two");
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
//        this.opMode.sleep(TIME_TO_GO_OUT);
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
//        this.debugTelemetery("Turn 3");
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
//        this.opMode.sleep(TIME_TO_GO_OUT);
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
//        this.debugTelemetery("Turn around back to wall");
//        this.opMode.sleep(200);
//        this.shooter.stop();
        this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED, -1.5);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.opMode.sleep(600);
        this.shooter.getToTargetSpeed(3500);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.shooter.getToTargetSpeed(3500);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.shooter.getToTargetSpeed(3500);


    }
    public void shootShotsInCaseFour(){

//        final int TIME_TO_GO_OUT = 250;
//        final int TIME_TO_GET_SPEED = 600;
//        this.debugTelemetery("Get to target : Shooter");
//        this.shooter.getToTargetSpeed(5900);
//        this.debugTelemetery("Shoot One");
//        this.opMode.sleep(TIME_TO_GET_SPEED);
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
//        this.opMode.sleep(TIME_TO_GO_OUT);
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
//        this.debugTelemetery("Shoot Two");
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
//        this.opMode.sleep(TIME_TO_GO_OUT);
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
//        this.debugTelemetery("Turn 3");
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
//        this.opMode.sleep(TIME_TO_GO_OUT);
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
//        this.debugTelemetery("Turn around back to wall");
//        this.opMode.sleep(200);
//        this.shooter.stop();
        this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED, 3);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.shooter.getToTargetSpeed(3800);
        this.opMode.sleep(800);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.opMode.sleep(800);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.hopperMover.setPosition(.63);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
//        this.intake.intakeArm.setPosition(.17);
    }
    public void driveToThirdBox(){
        double distToThree = 80;
        double avoidingRings = 21;
        this.drivetrain.encoderDriveWOWG(1,0,50,50,0,0);
        this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,0);
        this.drivetrain.gyroDrive(1,60,60,60, 60,0,0);
        this.drivetrain.gyroTurn(1,30);
        intake.setArmMode(Intake.sucMode.NOT_ACTIVE);
        this.opMode.sleep(100);
        this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,0);
        //checkEncoders();
    }


    public void getUpToSpeed(double targetSpeed){
        do {
            int initPos = this.shooter.ShooterWheel.getCurrentPosition();
            this.opMode.sleep(10);
            int finalPos = this.shooter.ShooterWheel.getCurrentPosition();

            int differenceInPos = finalPos - initPos;
            double revolutions = differenceInPos / 28;
            double minutes = 0.00016666666666666666666666;
            rpm = revolutions / minutes;
            this.telemetry.addLine("Current RPM: " + rpm);
        }
        while(targetSpeed == rpm);

    }
    public void shootShotsinCaseZero(){

//        final int TIME_TO_GO_OUT = 250;
//        final int TIME_TO_GET_SPEED = 600;
//        this.debugTelemetery("Get to target : Shooter");
//        this.shooter.getToTargetSpeed(5900);
//        this.debugTelemetery("Shoot One");
//        this.opMode.sleep(TIME_TO_GET_SPEED);
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
//        this.opMode.sleep(TIME_TO_GO_OUT);
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
//        this.debugTelemetery("Shoot Two");
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
//        this.opMode.sleep(TIME_TO_GO_OUT);
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
//        this.debugTelemetery("Turn 3");
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
//        this.opMode.sleep(TIME_TO_GO_OUT);
//        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
//        this.debugTelemetery("Turn around back to wall");
//        this.opMode.sleep(200);
//        this.shooter.stop();
        this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,-1);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,0);
//        this.shooter.getToTargetSpeed(3875);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.shooter.getToTargetSpeed(3825);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
    }

    public void lineUpWithWG() {
        this.drivetrain.gyroDrive(1,12,12,12,12,0,0);
        this.drivetrain.gyroTurn(.6,0); 
        this.drivetrain.encoderDriveWOWG(.8,0,-45,-45,0,0);
        this.drivetrain.gyroTurn(.6,0);
        this.drivetrain.gyroDrive(1,-22,-22,-22,-22,0,0);
        this.drivetrain.encoderDriveWOWG(this.drivetrain.DRIVE_SPEED,-17,17, 17, -17, 0);
        this.intake.setArmMode(Intake.sucMode.ACTIVE);
        this.intake.intakeRight.setPower(-.2);
        this.opMode.sleep(500);
        this.hopper.setHopperPosition(Hopper.Position.INIT_POSITION);
        this.drivetrain.gyroDrive(1, 40, 40,40,40,0,0);
        this.drivetrain.encoderDriveWOWG(1,0,80,80,0,0);
        this.drivetrain.gyroTurn(1,30);
        this.intake.setArmMode(Intake.sucMode.NOT_ACTIVE);
        this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,0);
        this.drivetrain.gyroDrive(1,-22,-22,-22,-22,0,0);

    }

    public enum wobbleNumber{
        FIRST, SECOND
    }
    public enum numOfRings{
        ZERO, ONE, FOUR
    }
    public void depositWobble(numOfRings numOfRings,wobbleNumber numOfWobble){
        final double STRAFE_TO_DEPOSIT;
        final double DIST_TO_LINE;
        switch(numOfWobble){
            case FIRST:
                switch(numOfRings){
                    case ZERO:
                        STRAFE_TO_DEPOSIT = 15.0;
                        this.debugTelemetery("Line Up with box", true);
//                        this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED,-STRAFE_TO_DEPOSIT,STRAFE_TO_DEPOSIT,STRAFE_TO_DEPOSIT,-STRAFE_TO_DEPOSIT,0);
                        this.drivetrain.encoderDriveWO(this.drivetrain.DRIVE_SPEED,0,42,42,0,0);
                        this.drivetrain.gyroTurn(this.drivetrain.TURN_SPEED,0);
                        this.debugTelemetery("Drop Wobble Goal",true);
                        //this.wobbleArm.runToPosition(WobbleArm.POSITION.OPEN_POSITION);
//                        this.wobbleArm.wobbleArm.setPower(-this.wobbleArm.REVERSE_POWER);
//                        this.opMode.sleep(1000);
//                        this.wobbleArm.Clasp.setPosition(0);
                        //
                        this.intake.setArmMode(Intake.sucMode.NOT_ACTIVE);
                        this.intake.intakeRight.setPower(0);
                        this.debugTelemetery("Get back to origin", true);
                        this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,-4,-4,-4,-4,0, 0 );
                        final double STRAFE_TO_SHOOT = 16.0;
                        this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED,STRAFE_TO_SHOOT,-STRAFE_TO_SHOOT,-STRAFE_TO_SHOOT,STRAFE_TO_SHOOT,0);
                        this.drivetrain.gyroTurn(this.drivetrain.TURN_SPEED,-3);
                        break;
                    case ONE:
                        STRAFE_TO_DEPOSIT = 10.0;
                        DIST_TO_LINE = 17;
                        this.debugTelemetery("Line Up with box", true);
//                        this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED,STRAFE_TO_DEPOSIT,-STRAFE_TO_DEPOSIT,-STRAFE_TO_DEPOSIT,STRAFE_TO_DEPOSIT,0);
                        this.debugTelemetery("Drop Wobble Goal",true);
                        //this.wobbleArm.runToPosition(WobbleArm.POSITION.OPEN_POSITION);
//                        this.wobbleArm.wobbleArm.setPower(-this.wobbleArm.REVERSE_POWER);
//                        this.opMode.sleep(1000);
//                        this.wobbleArm.Clasp.setPosition(0);
//fix
                        this.intake.setArmMode(Intake.sucMode.NOT_ACTIVE);
                        this.intake.intakeRight.setPower(0);
                        this.debugTelemetery("Get back to origin", true);
                        this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,-DIST_TO_LINE,-DIST_TO_LINE,-DIST_TO_LINE,-DIST_TO_LINE, 0, 0);
//                        this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED, -2, 2, 2, -2, 0);

                        break;
//                        final double STRAFE_TO_WOBBLE = 12.0;
//                        final double LINE_UP_TO_SHOOT = -18.0;
//                        this.wobbleArm.runToPosition(WobbleArm.POSITION.OPEN_POSITION);
//                        this.wobbleArm.setClaspPosition(WobbleArm.Position.OPEN_POSITION);
//                        this.drivetrain.gyroDrive(Drivetrain.DRIVE_SPEED, LINE_UP_TO_SHOOT, LINE_UP_TO_SHOOT, LINE_UP_TO_SHOOT, LINE_UP_TO_SHOOT, this.drivetrain.STRAIGHT,0);
//                        this.drivetrain.encoderDrive(Drivetrain.DRIVE_SPEED,STRAFE_TO_WOBBLE,-STRAFE_TO_WOBBLE,-STRAFE_TO_WOBBLE,STRAFE_TO_WOBBLE,0);
//                        this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,this.drivetrain.BACKWARD)
                    case FOUR:
                        STRAFE_TO_DEPOSIT = 39.0;
                        DIST_TO_LINE = 15;
                        this.debugTelemetery("Line Up with box", true);
                        this.debugTelemetery("Drop Wobble Goal",true);
//                        this.wobbleArm.runToPosition(WobbleArm.POSITION.OPEN_POSITION);
//                        this.wobbleArm.wobbleArm.setPower(-this.wobbleArm.REVERSE_POWER);
//                        this.opMode.sleep(1000);
//                        this.wobbleArm.Clasp.setPosition(0);
//FIX
                        this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,-2,-2,-2,-2,0, 0 );
                        this.debugTelemetery("Get back to origin", true);
                        this.debugTelemetery("strafe back");

                        this.drivetrain.encoderDriveWOWG(this.drivetrain.DRIVE_SPEED,0,-STRAFE_TO_DEPOSIT,-STRAFE_TO_DEPOSIT,0,0);
                        this.debugTelemetery("Drive back to shoot");
                        this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,-DIST_TO_LINE,-DIST_TO_LINE,-DIST_TO_LINE,-DIST_TO_LINE,this.drivetrain.STRAIGHT,0);
                        this.drivetrain.gyroTurn(this.drivetrain.TURN_SPEED, 0);
                        this.intake.intakeArm.setPosition(.4);
                        this.intake.setArmMode(Intake.sucMode.NOT_ACTIVE);
                        break;

                }
//            case SECOND:
//                final double DRIVE_BACK_TO_SECOND = 36.3;
//                final double STRAFE_TO_WOBBLE_TWO = 5.44;
//
//                switch(numOfRings){
//                    case ZERO:
//                        final double DRIVE_TO_FIRST_BOX = 48;
//                        final double STRAFE_TO_FIRST_BOX = 17.68;
//
//                        this.drivetrain.encoderDrive(Drivetrain.DRIVE_SPEED,-STRAFE_TO_WOBBLE_TWO,STRAFE_TO_WOBBLE_TWO,STRAFE_TO_WOBBLE_TWO,-STRAFE_TO_WOBBLE_TWO,0);
//                        this.drivetrain.gyroDrive(Drivetrain.DRIVE_SPEED,DRIVE_BACK_TO_SECOND,DRIVE_BACK_TO_SECOND,DRIVE_BACK_TO_SECOND,DRIVE_BACK_TO_SECOND,this.drivetrain.STRAIGHT,0);
//                        this.wobbleArm.setClaspPosition(WobbleArm.Position.CLOSED_POSITION);
//                        this.opMode.sleep(100);
//                        this.wobbleArm.runToPosition(WobbleArm.POSITION.CLOSE_POSITION);
//                        this.drivetrain.gyroTurn(TURN_SPEED, 0);
//                        this.drivetrain.encoderDrive(Drivetrain.DRIVE_SPEED,-STRAFE_TO_FIRST_BOX,STRAFE_TO_FIRST_BOX,STRAFE_TO_FIRST_BOX,-STRAFE_TO_FIRST_BOX,0);
//                        this.drivetrain.gyroDrive(Drivetrain.DRIVE_SPEED,DRIVE_TO_FIRST_BOX,DRIVE_TO_FIRST_BOX,DRIVE_TO_FIRST_BOX,DRIVE_TO_FIRST_BOX,this.drivetrain.STRAIGHT,0);
//                        this.wobbleArm.runToPosition(WobbleArm.POSITION.OPEN_POSITION);
//                        this.wobbleArm.setClaspPosition(WobbleArm.Position.OPEN_POSITION);
//                        this.opMode.sleep(100);
//                        break;
//                    case ONE:
//                        final double DRIVE_TO_SECOND_BOX = 0;
//                        final double STRAFE_TO_SECOND_BOX = 0;
//                        this.drivetrain.encoderDrive(Drivetrain.DRIVE_SPEED,-STRAFE_TO_WOBBLE_TWO,STRAFE_TO_WOBBLE_TWO,STRAFE_TO_WOBBLE_TWO,-STRAFE_TO_WOBBLE_TWO,0);
//                        this.drivetrain.gyroDrive(Drivetrain.DRIVE_SPEED,DRIVE_BACK_TO_SECOND,DRIVE_BACK_TO_SECOND,DRIVE_BACK_TO_SECOND,DRIVE_BACK_TO_SECOND,this.drivetrain.STRAIGHT,0);
//                        this.wobbleArm.setClaspPosition(WobbleArm.Position.CLOSED_POSITION);
//                        this.opMode.sleep(100);
//                        this.wobbleArm.runToPosition(WobbleArm.POSITION.CLOSE_POSITION);
//                        this.drivetrain.gyroTurn(TURN_SPEED, 0);
//                        this.drivetrain.encoderDrive(Drivetrain.DRIVE_SPEED,-STRAFE_TO_SECOND_BOX,STRAFE_TO_SECOND_BOX,STRAFE_TO_SECOND_BOX,-STRAFE_TO_SECOND_BOX,0);
//                        this.drivetrain.gyroDrive(Drivetrain.DRIVE_SPEED,DRIVE_TO_SECOND_BOX,DRIVE_TO_SECOND_BOX,DRIVE_TO_SECOND_BOX,DRIVE_TO_SECOND_BOX,this.drivetrain.STRAIGHT,0);
//                        this.wobbleArm.runToPosition(WobbleArm.POSITION.OPEN_POSITION);
//                        this.wobbleArm.setClaspPosition(WobbleArm.Position.OPEN_POSITION);
//                        this.opMode.sleep(100);
//                        break;
//                    case FOUR:
//                        final double DRIVE_TO_THIRD_BOX = 0;
//                        final double STRAFE_TO_THIRD_BOX = 10;
//                        this.drivetrain.encoderDrive(Drivetrain.DRIVE_SPEED,-STRAFE_TO_WOBBLE_TWO,STRAFE_TO_WOBBLE_TWO,STRAFE_TO_WOBBLE_TWO,-STRAFE_TO_WOBBLE_TWO,0);
//                        this.drivetrain.gyroDrive(Drivetrain.DRIVE_SPEED,DRIVE_BACK_TO_SECOND,DRIVE_BACK_TO_SECOND,DRIVE_BACK_TO_SECOND,DRIVE_BACK_TO_SECOND,this.drivetrain.STRAIGHT,0);
//                        this.wobbleArm.setClaspPosition(WobbleArm.Position.CLOSED_POSITION);
//                        this.opMode.sleep(100);
//                        this.wobbleArm.runToPosition(WobbleArm.POSITION.CLOSE_POSITION);
//                        this.drivetrain.gyroTurn(TURN_SPEED, 0);
//                        this.drivetrain.encoderDrive(Drivetrain.DRIVE_SPEED,-STRAFE_TO_THIRD_BOX,STRAFE_TO_THIRD_BOX,STRAFE_TO_THIRD_BOX,-STRAFE_TO_THIRD_BOX,0);
//                        this.drivetrain.gyroDrive(Drivetrain.DRIVE_SPEED,DRIVE_TO_THIRD_BOX,DRIVE_TO_THIRD_BOX,DRIVE_TO_THIRD_BOX,DRIVE_TO_THIRD_BOX,this.drivetrain.STRAIGHT,0);
//                        this.wobbleArm.runToPosition(WobbleArm.POSITION.OPEN_POSITION);
//                        this.wobbleArm.setClaspPosition(WobbleArm.Position.OPEN_POSITION);
//                        this.opMode.sleep(100);
//                        break;
//
//                }
        }
    }
    public boolean getCurrentRPM(double initTime, double currentTime, int initPos, int currentPos ){
        double differenceInTime = currentTime - initTime;
        if(differenceInTime > 1){
         int differenceInPos = currentPos - initPos;
         double revolutions = differenceInPos / 28;
         double minutes = differenceInTime / 60;
         rpm = revolutions / minutes;
        }
        this.telemetry.addLine("Current RPM: " + rpm);
        if(differenceInTime > 1) return true;
        return false;
    }

//    public void raiseWobbleArm(){
//        this.wobbleArm.wobbleArm.setPower(this.wobbleArm.REVERSE_POWER);
//        this.wobbleArm.Clasp.setPosition(0);
//        //FIx
//        this.opMode.sleep(1000);
//        this.wobbleArm.wobbleArm.setPower(0);
//
//
//    }
    public void hitBackWall(Drivetrain.NUM_OF_RINGS numOfRings){
        switch(numOfRings){
            case ZERO:
                this.drivetrain.gyroDrive(Drivetrain.DRIVE_SPEED,-4,-4,-4,-4,0,0);
               // this.drivetrain.gyroTurn(this.drivetrain.TURN_SPEED,180);
                this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED,-15,15, 15, -15, 0);
                this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,-33,-33,-33, -33,0,0);
                this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,0);
                this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED, 29,-29,-29,29,0);
                this.drivetrain.gyroDrive(.2,2,2,2,2,0,0);
//                this.wobbleArm.rcClasp.setPosition(1);
                this.intake.intakeArm.setPosition(.12);
                this.hopper.hopperMover.setPosition(.86);
                this.opMode.sleep(500);
//                this.drivetrain.gyroDrive(Drivetrain.DRIVE_SPEED,-5,-5,-5,-5,180,0);
                this.drivetrain.gyroDrive(0.8, 5, 5,5,5,0,0);
                this.drivetrain.encoderDriveWOWG(this.drivetrain.TURN_SPEED,0,68,68,0,0);
                this.drivetrain.gyroTurn(.3,0);
                //this.encoderDrive(DRIVE_SPEED,  0,60,0,60,0);

            break;
            case ONE:
//                this.drivetrain.encoderDriveWO(Drivetrain.DRIVE_SPEED,-37,0,0,-37,0);
////                this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,180);
//                this.drivetrain.gyroDrive(Drivetrain.DRIVE_SPEED,-40,-40,-40,-40,0,0);
//                this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED, 28.5,-28.5,-28.5,28.5,0);
//                this.drivetrain.gyroDrive(.2,5,5,5,5,0,0);
////                this.wobbleArm.rcClasp.setPosition(1);
//                this.intake.setArmMode(Intake.sucMode.ACTIVE);
//                this.intake.intakeRight.setPower(.2);
//                this.opMode.sleep(500);
//                this.drivetrain.encoderDriveWO(this.drivetrain.TURN_SPEED,25,0,0,25,0);
//                this.drivetrain.gyroDrive(Drivetrain.DRIVE_SPEED,41,41,41,41,0,0);
////                this.drivetrain.encoderDriveWO(this.drivetrain.DRIVE_SPEED,0,-85,-85,0,0);
////                this.drivetrain.encoderDriveWO(this.drivetrain.TURN_SPEED,-45,0,0,-45,0);
////                this.drivetrain.gyroDrive(Drivetrain.TURN_SPEED,-10,-10,-10,-10,180,0);
//
////                this.wobbleArm.rcClasp.setPosition(0);
                this.drivetrain.gyroTurn(1.75,0);
                this.drivetrain.gyroDrive(1, -12, -12,-12,-12,0,0);
                this.drivetrain.gyroTurn(.75,0);
                this.drivetrain.encoderDriveWOWG(.8,27.5,-27.5,-27.5,27.5,0);
                this.drivetrain.gyroDrive(1, 4,4,4,4,0,0);
                this.intake.setArmMode(Intake.sucMode.ACTIVE);
                this.intake.intakeRight.setPower(-.2);
                this.opMode.sleep(500);
                this.hopper.setHopperPosition(Hopper.Position.INIT_POSITION);
                this.drivetrain.encoderDriveWOWG(Drivetrain.DRIVE_SPEED,0,28,28,0,0);
                this.drivetrain.gyroDrive(0.60, 44, 44,44,44,0,0);
                this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,0);
                break;
            case FOUR:
//                this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,180);
                this.drivetrain.gyroTurn(.75,0);
                this.drivetrain.gyroDrive(1, -12, -12,-12,-12,0,0);
                this.drivetrain.gyroTurn(.75,0);
                this.drivetrain.encoderDriveWOWG(.8,23,-23,-23,23,0);
                this.drivetrain.gyroDrive(1, 3,3,3,3,0,0);
                this.intake.intakeArm.setPosition(.12);
                this.intake.intakeRight.setPower(-.2);
                this.opMode.sleep(500);
                this.hopper.setHopperPosition(Hopper.Position.INIT_POSITION);
                this.drivetrain.gyroDrive(1, 40, 40,40,40,0,0);
                this.drivetrain.encoderDriveWOWG(1,0,62,62,0,0);
                break;


        }
        this.intake.setArmMode(Intake.sucMode.NOT_ACTIVE);
        this.intake.intakeRight.setPower(0);
        this.opMode.sleep(250);
    }
    public void ParkWith2WG(Drivetrain.NUM_OF_RINGS numOfRings){
        final double STRAFE_TO_LINE;
        final double DRIVE_TO_LINE;
        final double DRIVE_TO_SHOOT;
        switch (numOfRings){
            case FOUR:
                STRAFE_TO_LINE = 6.0;
                DRIVE_TO_LINE = -10.0;
//                this.encoderDrive(DRIVE_SPEED,STRAFE_TO_LINE,STRAFE_TO_LINE,STRAFE_TO_LINE, STRAFE_TO_LINE,0);
                DRIVE_TO_SHOOT = -34.0;
//                this.encoderDrive(DRIVE_SPEED,STRAFE_TO_LINE,STRAFE_TO_LINE,STRAFE_TO_LINE, STRAFE_TO_LINE,0);
                this.drivetrain.gyroDrive(1,DRIVE_TO_SHOOT,DRIVE_TO_SHOOT,DRIVE_TO_SHOOT,DRIVE_TO_SHOOT,10,0);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
                this.opMode.sleep(100);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
                this.opMode.sleep(100);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
                this.opMode.sleep(100);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
                this.drivetrain.gyroDrive(1,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,0,0);
                break;
            case ONE:
                STRAFE_TO_LINE = 6.0;
                DRIVE_TO_LINE = 12.0;
                DRIVE_TO_SHOOT = -18.0;
//                this.encoderDrive(DRIVE_SPEED,STRAFE_TO_LINE,STRAFE_TO_LINE,STRAFE_TO_LINE, STRAFE_TO_LINE,0);
                this.drivetrain.gyroDrive(1,DRIVE_TO_SHOOT,DRIVE_TO_SHOOT,DRIVE_TO_SHOOT,DRIVE_TO_SHOOT,5,0);
                this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
                this.opMode.sleep(500);
                this.drivetrain.gyroDrive(1,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,0,0);


                break;

            case ZERO:
//                this.wobbleArm.rcClasp.setPosition(0.5);
                STRAFE_TO_LINE = 6.0;
                DRIVE_TO_LINE = 6.0;
//                this.encoderDrive(DRIVE_SPEED,STRAFE_TO_LINE,STRAFE_TO_LINE,STRAFE_TO_LINE, STRAFE_TO_LINE,0);
//                this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,DRIVE_TO_LINE,-DRIVE_TO_LINE,-DRIVE_TO_LINE, -DRIVE_TO_LINE,0,0);
                this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED,20,-20,-20,20,0);
                this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,15,15,15,15,0,0);
                break;
        }
    }
    public  void collectRing(numOfRings rings){
    switch(rings){
        case ONE:
            this.intake.intakeLeft.setPower(.7);
            this.intake.intakeRight.setPower(-1);
            this.drivetrain.gyroTurn(.75,180);
            this.drivetrain.encoderDriveWOWG(1,0,5,5,0,180);
            this.drivetrain.gyroDrive(1,20, 20,20,20,180,0);
            this.drivetrain.encoderDriveWOWG(Drivetrain.DRIVE_SPEED,25,0,0,25,180);
            this.intake.intakeRight.setPower(0);
            break;
        case FOUR:

//            this.drivetrain.gyroTurn(.75,180);
//            this.drivetrain.encoderDriveWOWG(1,0,5,5,0,180);
            this.intake.intakeLeft.setPower(.7);
            this.intake.intakeRight.setPower(-.8);
            this.drivetrain.gyroTurn(.8,180);
            this.drivetrain.gyroDrive(.8,15, 15,15,15,180,0);
            this.drivetrain.encoderDriveWOWG(Drivetrain.DRIVE_SPEED,25,0,0,25,180);
            break;
    }
    }

    /**
     * @param timeInMs
     * */
    public void wait(double timeInMs){

        while(this.opMode.time < (timeInMs/1000));
    }
    @Override
    public void stop() {
        for( int i = 0; i<=1; i++){
            components[i].stop();
        }
    }
}
