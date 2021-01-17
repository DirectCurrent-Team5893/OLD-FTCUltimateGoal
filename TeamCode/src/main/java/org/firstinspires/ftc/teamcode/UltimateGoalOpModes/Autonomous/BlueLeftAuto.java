package org.firstinspires.ftc.teamcode.UltimateGoalOpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FullBase;
import org.firstinspires.ftc.teamcode.RingDetector;

@Autonomous(name="BlueAuto")
public class BlueLeftAuto extends LinearOpMode {

        private ElapsedTime runtime = new ElapsedTime();
        FullBase Base ;
        boolean flickerPositon =true;

        @Override
        public void runOpMode() {
            RingDetector detector = new RingDetector(this);
            Base = new FullBase(telemetry, this, hardwareMap);
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            Base.init();
            telemetry.addLine("done with init");
            int decision = detector.getDecision();
            telemetry.addData("Decision:", decision);
            telemetry.update();
            waitForStart();
            switch (decision) {
                case 1:
                    Base.drivetrain.driveToSecondBox();
                    break;

                case 4:
                    Base.drivetrain.driveToThirdBox();
                    break;

                case 0:
                default:
                    Base.drivetrain.driveToFirstBox();
                    break;
            }
        }
}
