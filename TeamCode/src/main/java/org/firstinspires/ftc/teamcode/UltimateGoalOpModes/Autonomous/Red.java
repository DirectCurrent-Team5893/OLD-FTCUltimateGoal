package org.firstinspires.ftc.teamcode.UltimateGoalOpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RingDetector;

@Autonomous(name="BlueTest")
public class Red extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RingDetector detector = new RingDetector(this);

        waitForStart();
        while(opModeIsActive()) {
            int rings = detector.getDecision();
            telemetry.addData("Decision: ", detector.getDecision());
            telemetry.update();
        }
    }
}
