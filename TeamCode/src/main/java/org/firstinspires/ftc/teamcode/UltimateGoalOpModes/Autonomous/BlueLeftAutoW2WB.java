package org.firstinspires.ftc.teamcode.UltimateGoalOpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FullBase;
import org.firstinspires.ftc.teamcode.RingDetector;
import org.firstinspires.ftc.teamcode.UltimateGoalComponents.Drivetrain;
import org.firstinspires.ftc.teamcode.UltimateGoalComponents.Intake;

@Autonomous(name="BlueLeftAutoW2WB")
public class BlueLeftAutoW2WB extends LinearOpMode {

        private ElapsedTime runtime = new ElapsedTime();
        FullBase Base ;
        boolean flickerPositon =true;

        @Override
        public void runOpMode() {

            RingDetector detector = new RingDetector(this);
            Base = new FullBase(telemetry, this, hardwareMap, false);

            telemetry.addData("Status", "Initialized");
            telemetry.update();
            Base.init();
            telemetry.addLine("done with init");
            waitForStart();
            int decision = detector.getDecision();;
            telemetry.addData("Decision:", decision);
            telemetry.update();
            switch (decision) {

                case 1:
                    Base.intake.setArmMode(Intake.sucMode.ACTIVE);
                    this.sleep(1000);
                    Base.shooter.getToTargetSpeed(3880);
                    Base.drivetrain.driveToSecondBox();
                    Base.debugWait();
                    Base.depositWobble(FullBase.numOfRings.ONE, FullBase.wobbleNumber.FIRST);
                    Base.debugWait();
//                    Base.raiseWobbleArm();
                    Base.drivetrain.gyroTurn(Drivetrain.TURN_SPEED,-1);
                    Base.shootShots();
                    Base.hopper.hopperMover.setPosition(.63);
                    this.sleep(200);
                    Base.collectRing(FullBase.numOfRings.ONE);
                    Base.debugWait();
                    Base.hitBackWall(Drivetrain.NUM_OF_RINGS.ONE);
                    Base.ParkWith2WG(Drivetrain.NUM_OF_RINGS.ONE);
                  //  Base.drivetrain.Park(Drivetrain.NUM_OF_RINGS.ONE);

                    break;
//                    Base.drivetrain.lineUpWithLine();
//                    debugWait();
//                    Base.shootPowerShots();
//                    Base.debugWait();
//                    Base.depositWobble(FullBase.numOfRings.ONE, FullBase.wobbleNumber.SECOND);
//                    Base.debugWait();
//                    Base.drivetrain.Park(Drivetrain.NUM_OF_RINGS.ONE);


                case 4:
                    Base.intake.setArmMode(Intake.sucMode.ACTIVE);
                    this.sleep(1000);
                    Base.shooter.getToTargetSpeed(4025);
                    Base.driveToThirdBox();
                    Base.debugWait();
                    Base.depositWobble(FullBase.numOfRings.FOUR, FullBase.wobbleNumber.FIRST);
                    Base.debugWait();
//                    Base.drivetrain.lineUpWithLine();
//                    debugWait();
//                    Base.raiseWobbleArm();
                    Base.intake.intakeArm.setPosition(.6);
                    Base.shootShotsInCaseFour();
                    Base.drivetrain.ringBump();
                    Base.lineUpWithWG();
//                    Base.debugWait();
//                    Base.collectRing(FullBase.numOfRings.FOUR);
//                    Base.hitBackWall(Drivetrain.NUM_OF_RINGS.FOUR);
//                    Base.ParkWith2WG(Drivetrain.NUM_OF_RINGS.FOUR);
//                    //Base.depositWobble(FullBase.numOfRings.ONE, FullBase.wobbleNumber.SECOND);
//                    Base.debugWait();
//
//             //       Base.drivetrain.Park(Drivetrain.NUM_OF_RINGS.FOUR);
                    break;

                case 0:
                default:
                    Base.intake.setArmMode(Intake.sucMode.ACTIVE);
                    this.sleep(1000);
                    Base.drivetrain.driveToFirstBox();
                     Base.debugWait();
                    Base.shooter.getToTargetSpeed(3850);
//                    Base.shooter.getToTargetSpeed(3925);
                    Base.depositWobble(FullBase.numOfRings.ZERO, FullBase.wobbleNumber.FIRST);
                     Base.debugWait();
//                    Base.drivetrain.lineUpWithLine();
//                    debugWait();
//                    Base.shootPowerShots();
                    Base.debugTelemetery("Shoot Shots");
//                    Base.raiseWobbleArm();
                    Base.shootShotsinCaseZero();
                    Base.shooter.ShooterWheel.setPower(0);
                    Base.debugWait();
//                    Base.depositWobble(FullBase.numOfRings.ZERO, FullBase.wobbleNumber.SECOND);
//                    Base.debugWait();
                    Base.hitBackWall(Drivetrain.NUM_OF_RINGS.ZERO);
                    Base.debugTelemetery("Park");
                    Base.ParkWith2WG(Drivetrain.NUM_OF_RINGS.ZERO);
                    break;

            }
        }
}
