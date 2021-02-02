package org.firstinspires.ftc.teamcode.UltimateGoalComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;
/*
@author Yahya ElGawady
 */
public class Shooter extends RobotComponent {



   private ElapsedTime runtime = new ElapsedTime();
   public DcMotor ShooterWheel;

    public final double STOP = 0;

    boolean stop = false;

    public Shooter(robotBase BASE) {
        super(BASE);

       ShooterWheel = base.getMapper().mapMotor("shooter", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void shoot(boolean button){
        if(button)
        getToTargetSpeed(1.0 );
    }


    public void getToTargetSpeed(int target_rpm){
//        double percentOfTotal = (double)(target_rpm)/6000;
       ShooterWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        ShooterWheel.setPower(percentOfTotal);
      if(target_rpm>5900)
      {
          ShooterWheel.setPower(1.0);
      } else
      {
          ShooterWheel.setPower(.9);
      }
    }

    public void getToTargetSpeed(double target_speed) {
        boolean target = false;
        double initEncoder;
        double finEncoder;
        double difEncoder;
        double timeRecorded;

        ShooterWheel.setPower(target_speed);
      /*
      do{
            while(!stop) {
                initEncoder = ShooterWheel.getCurrentPosition();
                timeRecorded = runtime.seconds();
                while (runtime.seconds() <= timeRecorded + 1) ;
                finEncoder = ShooterWheel.getCurrentPosition();

                difEncoder = finEncoder - initEncoder;

                if (difEncoder > 2700) {
                    target = true;
                } else {
                    target = false;
                }
            }
        }
        while(!target);
       */
    }

    @Override
    public void stop() {
        ShooterWheel.setPower(STOP);
        stop = true;
    }
    public void stop(boolean button) {
        if(button)
        ShooterWheel.setPower(STOP);
        stop = true;
    }

}
