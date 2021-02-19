package org.firstinspires.ftc.teamcode.UltimateGoalOpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FullBase;
import org.firstinspires.ftc.teamcode.RingDetector;
import org.firstinspires.ftc.teamcode.UltimateGoalComponents.Drivetrain;

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
            int decision = detector.getDecision();
            telemetry.addData("Decision:", decision);
            telemetry.update();
            switch (decision) {

                case 1:
                    Base.drivetrain.driveToSecondBox();
                    Base.debugWait();
                    Base.depositWobble(FullBase.numOfRings.ONE, FullBase.wobbleNumber.FIRST);
                    Base.debugWait();
                    Base.raiseWobbleArm();
                    Base.shootShots();
                    Base.debugWait();
                    Base.drivetrain.Park(Drivetrain.NUM_OF_RINGS.ONE);
                    break;
//                    Base.drivetrain.lineUpWithLine();
//                    debugWait();
//                    Base.shootPowerShots();
//                    Base.debugWait();
//                    Base.depositWobble(FullBase.numOfRings.ONE, FullBase.wobbleNumber.SECOND);
//                    Base.debugWait();
//                    Base.drivetrain.Park(Drivetrain.NUM_OF_RINGS.ONE);


                case 4:
                    Base.drivetrain.driveToThirdBox();
                    Base.debugWait();
                    Base.depositWobble(FullBase.numOfRings.FOUR, FullBase.wobbleNumber.FIRST);
                    Base.debugWait();
//                    Base.drivetrain.lineUpWithLine();
//                    debugWait();
                    Base.raiseWobbleArm();
                    Base.shootShots();
                    Base.debugWait();
                    //Base.depositWobble(FullBase.numOfRings.ONE, FullBase.wobbleNumber.SECOND);
                    Base.debugWait();
                    Base.drivetrain.Park(Drivetrain.NUM_OF_RINGS.FOUR);
                    break;

                case 0:
                default:

                     Base.drivetrain.driveToFirstBox();
                     Base.debugWait();
                     Base.depositWobble(FullBase.numOfRings.ZERO, FullBase.wobbleNumber.FIRST);
                     Base.debugWait();
//                    Base.drivetrain.lineUpWithLine();
//                    debugWait();
//                    Base.shootPowerShots();
                    Base.debugTelemetery("Shoot Shots");
                    Base.raiseWobbleArm();
                    Base.shootShotsinCaseZero();
                    Base.shooter.ShooterWheel.setPower(0);
                    Base.debugWait();
//                    Base.depositWobble(FullBase.numOfRings.ZERO, FullBase.wobbleNumber.SECOND);
//                    Base.debugWait();
                    Base.hitBackWall();
                    Base.debugTelemetery("Park");
                    Base.ParkWith2WG(Drivetrain.NUM_OF_RINGS.ZERO);
                    break;

            }
        }
}
