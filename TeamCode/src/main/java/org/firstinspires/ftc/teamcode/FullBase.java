package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UltimateGoalComponents.Drivetrain;
import org.firstinspires.ftc.teamcode.UltimateGoalComponents.Hopper;
import org.firstinspires.ftc.teamcode.UltimateGoalComponents.Intake;
import org.firstinspires.ftc.teamcode.UltimateGoalComponents.Shooter;
import org.firstinspires.ftc.teamcode.UltimateGoalComponents.WobbleArm;

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
        wobbleArm.upperClasp.setPosition(1);
        wobbleArm.lowerClasp.setPosition(0);
        wobbleArm.rcClasp.setPosition(.5);
        components[3] = wobbleArm;


        //create intake
        intake = new Intake (this);
        components[4] = intake;
    }
    public void shootPowerShots(){
        this.shooter.getToTargetSpeed(3800);
        this.drivetrain.gyroDrive(Drivetrain.DRIVE_SPEED,-2,-2,-2,-2,0,0);
        this.drivetrain.gyroTurn(this.drivetrain.TURN_SPEED,0);
        this.drivetrain.resetEncoders();
        this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,10,-10,-10,10,0,0);
        this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,0,0,0,0,55.9,0,0);
        this.drivetrain.gyroTurn(this.drivetrain.TURN_SPEED, -1);
        this.getUpToSpeed(3800);
        this.shooter.getToTargetSpeed(3800);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.opMode.sleep(600);
        this.shooter.getToTargetSpeed(3530);
        this.getUpToSpeed(3530);
//        this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED,12,-12,-12,12,0);
        this.drivetrain.gyroTurn(this.drivetrain.TURN_SPEED, -4);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.opMode.sleep(600);
        this.getUpToSpeed(3530);
//        this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED,12,-12,-12,12,0);
        this.drivetrain.gyroTurn(this.drivetrain.TURN_SPEED, -8);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.opMode.sleep(600);
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
        this.shooter.getToTargetSpeed(4165);
        this.opMode.sleep(3000);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.opMode.sleep(600);

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
        this.shooter.getToTargetSpeed(4165);
        this.opMode.sleep(2900);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.IN_POSITION);
        this.opMode.sleep(600);
        this.hopper.setFlickerPosition(Hopper.flickerPosition.OUT_POSITION);
        this.opMode.sleep(600);

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
                        STRAFE_TO_DEPOSIT = 22.0;
                        this.debugTelemetery("Line Up with box", true);
                        this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED,-STRAFE_TO_DEPOSIT,STRAFE_TO_DEPOSIT,STRAFE_TO_DEPOSIT,-STRAFE_TO_DEPOSIT,0);
                        this.drivetrain.gyroTurn(this.drivetrain.TURN_SPEED,0);
                        this.debugTelemetery("Drop Wobble Goal",true);
                        //this.wobbleArm.runToPosition(WobbleArm.POSITION.OPEN_POSITION);
                        this.wobbleArm.wobbleArm.setPower(-this.wobbleArm.REVERSE_POWER);
                        this.opMode.sleep(1000);
                        this.wobbleArm.upperClasp.setPosition(0);
                        this.wobbleArm.lowerClasp.setPosition(1);
                        this.debugTelemetery("Get back to origin", true);
                        this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,-2,-2,-2,-2,0, 0 );
                        this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED,STRAFE_TO_DEPOSIT,-STRAFE_TO_DEPOSIT,-STRAFE_TO_DEPOSIT,STRAFE_TO_DEPOSIT,0);
                        this.drivetrain.gyroTurn(this.drivetrain.TURN_SPEED,0);
                        break;
                    case ONE:
                        STRAFE_TO_DEPOSIT = 10.0;
                        DIST_TO_LINE = 16;
                        this.debugTelemetery("Line Up with box", true);
//                        this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED,STRAFE_TO_DEPOSIT,-STRAFE_TO_DEPOSIT,-STRAFE_TO_DEPOSIT,STRAFE_TO_DEPOSIT,0);
                        this.debugTelemetery("Drop Wobble Goal",true);
                        //this.wobbleArm.runToPosition(WobbleArm.POSITION.OPEN_POSITION);
                        this.wobbleArm.wobbleArm.setPower(-this.wobbleArm.REVERSE_POWER);
                        this.opMode.sleep(1000);
                        this.wobbleArm.upperClasp.setPosition(0);
                        this.wobbleArm.lowerClasp.setPosition(1);

                        this.debugTelemetery("Get back to origin", true);
                        this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,-DIST_TO_LINE,-DIST_TO_LINE,-DIST_TO_LINE,-DIST_TO_LINE, 0, 0);
                        this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED, -2, 2, 2, -2, 0);

                        break;
//                        final double STRAFE_TO_WOBBLE = 12.0;
//                        final double LINE_UP_TO_SHOOT = -18.0;
//                        this.wobbleArm.runToPosition(WobbleArm.POSITION.OPEN_POSITION);
//                        this.wobbleArm.setClaspPosition(WobbleArm.Position.OPEN_POSITION);
//                        this.drivetrain.gyroDrive(Drivetrain.DRIVE_SPEED, LINE_UP_TO_SHOOT, LINE_UP_TO_SHOOT, LINE_UP_TO_SHOOT, LINE_UP_TO_SHOOT, this.drivetrain.STRAIGHT,0);
//                        this.drivetrain.encoderDrive(Drivetrain.DRIVE_SPEED,STRAFE_TO_WOBBLE,-STRAFE_TO_WOBBLE,-STRAFE_TO_WOBBLE,STRAFE_TO_WOBBLE,0);
//                        this.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,this.drivetrain.BACKWARD)
                    case FOUR:
                        STRAFE_TO_DEPOSIT = 16.0;
                        DIST_TO_LINE = 35;
                        this.debugTelemetery("Line Up with box", true);
                        this.debugTelemetery("Drop Wobble Goal",true);
//                        this.wobbleArm.runToPosition(WobbleArm.POSITION.OPEN_POSITION);
                        this.wobbleArm.wobbleArm.setPower(-this.wobbleArm.REVERSE_POWER);
                        this.opMode.sleep(1000);
                        this.wobbleArm.upperClasp.setPosition(0);
                        this.wobbleArm.lowerClasp.setPosition(1);
                        this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,-2,-2,-2,-2,0, 0 );
                        this.debugTelemetery("Get back to origin", true);
                        this.debugTelemetery("strafe back");
                        this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED,STRAFE_TO_DEPOSIT,-STRAFE_TO_DEPOSIT,-STRAFE_TO_DEPOSIT,STRAFE_TO_DEPOSIT,0);
                        this.debugTelemetery("Drive back to shoot");
                        this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,-DIST_TO_LINE,-DIST_TO_LINE,-DIST_TO_LINE,-DIST_TO_LINE,this.drivetrain.STRAIGHT,0);
                        this.drivetrain.gyroTurn(this.drivetrain.TURN_SPEED, 0);
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

    public void raiseWobbleArm(){
        this.wobbleArm.wobbleArm.setPower(this.wobbleArm.REVERSE_POWER);
        this.wobbleArm.lowerClasp.setPosition(0);
        this.wobbleArm.upperClasp.setPosition(1);
        this.opMode.sleep(1000);
        this.wobbleArm.wobbleArm.setPower(0);


    }
    public void hitBackWall(){
        this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED,-30,30, 30, -30, 0);
        this.drivetrain.gyroTurn(this.drivetrain.TURN_SPEED,180);
        this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,60,60,60, 60,180,0);
        this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED, -27,27,27,-27,0);
        this.wobbleArm.rcClasp.setPosition(1);
        this.drivetrain.encoderDriveWO(this.drivetrain.DRIVE_SPEED,0,-100,-100,0,0);
        //this.encoderDrive(DRIVE_SPEED,  0,60,0,60,0);
    }
    public void ParkWith2WG(Drivetrain.NUM_OF_RINGS numOfRings){
        final double STRAFE_TO_LINE;
        final double DRIVE_TO_LINE;
        switch (numOfRings){
            case FOUR:
                STRAFE_TO_LINE = 6.0;
                DRIVE_TO_LINE = 15.0;
//                this.encoderDrive(DRIVE_SPEED,STRAFE_TO_LINE,STRAFE_TO_LINE,STRAFE_TO_LINE, STRAFE_TO_LINE,0);
                this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,180,0);
                break;
            case ONE:
                STRAFE_TO_LINE = 6.0;
                DRIVE_TO_LINE = 10.0;
//                this.encoderDrive(DRIVE_SPEED,STRAFE_TO_LINE,STRAFE_TO_LINE,STRAFE_TO_LINE, STRAFE_TO_LINE,0);
                this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,180,0);
                break;

            case ZERO:
                this.wobbleArm.rcClasp.setPosition(0.5);
                STRAFE_TO_LINE = 6.0;
                DRIVE_TO_LINE = 6.0;
//                this.encoderDrive(DRIVE_SPEED,STRAFE_TO_LINE,STRAFE_TO_LINE,STRAFE_TO_LINE, STRAFE_TO_LINE,0);
                this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,180,0);
                this.drivetrain.encoderDrive(this.drivetrain.DRIVE_SPEED,10,-10,-10,10,0);
                this.drivetrain.gyroDrive(this.drivetrain.DRIVE_SPEED,10,10,10,10,180,0);
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
