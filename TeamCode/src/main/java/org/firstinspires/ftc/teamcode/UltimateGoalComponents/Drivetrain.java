package org.firstinspires.ftc.teamcode.UltimateGoalComponents;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;
import org.firstinspires.ftc.robotcontroller.internal.robotBase;

import java.util.ArrayList;


@SuppressWarnings("MoveFieldAssignmentToInitializer")
public class Drivetrain extends RobotComponent {

   
        public DcMotor frontLeft;
        public DcMotor frontRight;
        public DcMotor backLeft;
        public DcMotor backRight;
        public DcMotor[] motors = new DcMotor[4];

        public GyroSensor gyroSensor;

    private double stopBuffer = 0.05;
    public double amountError = 0.64;
   public final double STRAIGHT = 0;
    public final double BACKWARD = 180;




    static final double     COUNTS_PER_MOTOR_REV    = 753.2;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953;
    public static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_INCHES * 3.14159265);
//    static final double COUNTS_PER_INCH = 85.9;
//    static final double COUNTS_PER_INCH = 31.717;


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public static final double DRIVE_SPEED = 0.5;     // Nominal speed for better accuracy.
    public static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.10;     // Larger is more responsive, but also less stable



    public Drivetrain(robotBase BASE) {
        super(BASE);
        initMotors();
        try {
            initGyro();
        }
        catch (RuntimeException ex){
            base.getTelemetry().addLine("Error with Gyro init");
        }
    }

     void initMotors() {

               frontLeft = base().getMapper().mapMotor("frontLeft");
               motors[0] = frontLeft;

               backLeft = base().getMapper().mapMotor("backLeft");
               motors[1] = backLeft;
         frontRight = base().getMapper().mapMotor("frontRight", DcMotorSimple.Direction.REVERSE);
        motors[2] = frontRight;

         backRight = base().getMapper().mapMotor("backRight", DcMotorSimple.Direction.REVERSE);
        motors[3] = backRight;

        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);

     }

     void initGyro(){
        gyroSensor = base().getMapper().mapMRGyro("gyro");
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating());
         base().getTelemetry().addLine("Gyro Calibrated");
     }

    public void drive(double forward, double right, double turn) {

        forward = getProcessedInput(forward);
        right = getProcessedInput(right);
        turn = getProcessedInput(turn);

        double leftFrontPower = forward + right + turn;
        double leftBackPower = forward - right + turn;
        double rightFrontPower = forward - right - turn;
        double rightBackPower = forward + right - turn;
        double[] powers = {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};

        boolean needToScale = false;
        for (double power : powers) {
            if (Math.abs(power) > 1) {
                needToScale = true;
                break;
            }
        }
        if (needToScale) {
            double greatest = 0;
            for (double power : powers) {
                if (Math.abs(power) > greatest) {
                    greatest = Math.abs(power);
                }
            }
            leftFrontPower /= greatest;
            leftBackPower /= greatest;
            rightFrontPower /= greatest;
            rightBackPower /= greatest;
        }
        setPowers(powers);
    }
        public double[] getPowers () {
            double[] powers = {frontLeft.getPower(), backLeft.getPower(), frontRight.getPower(), backRight.getPower()};
            return powers;
        }

        public void setPowers ( double[] powers){
            frontLeft.setPower(powers[0]);
            backLeft.setPower(powers[1]);
            frontRight.setPower(powers[2]);
            backRight.setPower(powers[3]);
        }
        public void setPowers ( double power){
            for (DcMotor m : motors) {
                m.setPower(power);
            }
        }

        public double getAverageEncoders (ArrayList < DcMotor > dcmotors) {
            double sum = 0;
            for (DcMotor m : dcmotors) {
                sum += Math.abs(m.getCurrentPosition());
            }
            return (sum / (double) (dcmotors.size()));
        }


        public void stop () {
            setPowers(0);
        }

        public void setModes (DcMotor.RunMode runMode){
            for (DcMotor motor : motors) {
                motor.setMode(runMode);
            }
        }
        public void setZeroPowerBehaviors (DcMotor.ZeroPowerBehavior behavior){
            for (DcMotor motor : motors) {
                motor.setZeroPowerBehavior(behavior);
            }
        }

        private double getProcessedInput ( double hardInput){
            hardInput = Range.clip(hardInput, -1, 1);
            hardInput = Math.pow(hardInput, 3);
            return hardInput;
        }
    public void gyroDrive ( double speed,
                            double frontLeftInches, double frontRightInches, double backLeftInches,
                            double backRightInches,
                            double angle, double timeoutS){

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        double HalfMaxOne;
        double HalfMaxTwo;

        double max;

        double error;
        double steer;
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;

        double ErrorAmount;
        boolean goodEnough = false;

        // Ensure that the opmode is still active
        if (base().getOpMode().opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);


            // Set Target and Turn On RUN_TO_POSITION
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));
            // keep looping while we are still active, and BOTH motors are running.
            while (base.getOpMode().opModeIsActive() &&
                    (frontLeft.isBusy() && frontRight.isBusy()) && (backLeft.isBusy() && backRight.isBusy()) && !goodEnough) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (frontLeftInches < 0 && frontRightInches < 0 && backLeftInches < 0 && backRightInches < 0)
                    steer *= -1.0;

                frontLeftSpeed = speed - steer;
                backLeftSpeed = speed - steer;
                backRightSpeed = speed + steer;
                frontRightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                HalfMaxOne = Math.max(Math.abs(frontLeftSpeed), Math.abs(backLeftSpeed));
                HalfMaxTwo = Math.max(Math.abs(frontRightSpeed), Math.abs(backRightSpeed));
                max = Math.max(Math.abs(HalfMaxOne), Math.abs(HalfMaxTwo));
                if (max > 1.0) {
                    frontLeftSpeed /= max;
                    frontRightSpeed /= max;
                    backLeftSpeed /= max;
                    backRightSpeed /= max;
                }

                frontLeft.setPower(frontLeftSpeed);
                frontRight.setPower(frontRightSpeed);
                backLeft.setPower(backLeftSpeed);
                backRight.setPower(backRightSpeed);

                // Display drive status for the driver.
                base().getTelemetry().addData("Err/St", "%5.1f/%5.1f", error, steer);
                base().getTelemetry().addData("Target", "%7d:%7d", newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                base().getTelemetry().addData("Actual", "%7d:%7d", backLeft.getCurrentPosition(), backRight.getCurrentPosition(), frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                base().getTelemetry().addData("Speed", "%5.2f:%5.2f", backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
                base().getTelemetry().update();

                ErrorAmount = ((Math.abs(((newBackLeftTarget) - (backLeft.getCurrentPosition())))
                        + (Math.abs(((newFrontLeftTarget) - (frontLeft.getCurrentPosition()))))
                        + (Math.abs((newBackRightTarget) - (backRight.getCurrentPosition())))
                        + (Math.abs(((newFrontRightTarget) - (frontRight.getCurrentPosition()))))) / COUNTS_PER_INCH);
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

//    public void gyroDrive(double speed, double forwardInches, double rightInches, double angle){
//        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
//        int frontLeftCounts = (int) ((forwardInches + (1.414 * rightInches)) * COUNTS_PER_INCH);
//        int frontRightCounts = (int) ((forwardInches - (1.414 * rightInches)) * COUNTS_PER_INCH);
//
//        int frontLeftTarget = frontLeft.getCurrentPosition() + frontLeftCounts;
//        int backRightTarget = backRight.getCurrentPosition() + frontLeftCounts;
//        int frontRightTarget= frontRight.getCurrentPosition() + frontRightCounts;
//        int backLeftTarget = backLeft.getCurrentPosition() + frontRightCounts;
//
//        frontLeft.setTargetPosition(frontLeftTarget);
//        backRight.setTargetPosition(backRightTarget);
//        frontRight.setTargetPosition(frontRightTarget);
//        backLeft.setTargetPosition(backLeftTarget);
//
//        setModes(DcMotor.RunMode.RUN_TO_POSITION);
//
//        speed = Range.clip(Math.abs(speed), 0.0,1.0);
//        frontLeft.setPower(speed);
//        frontRight.setPower(speed);
//        backLeft.setPower(speed);
//        backRight.setPower(speed);
//
//        while (base.getOpMode().opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
//            double error = angle - gyroSensor.getHeading();
//            while (error > 180){
//                error -= 360;
//            }
//            while (error < -180){
//                error += 360;
//            }
//
//            double steer = Range.clip(error * 0.1, -1.0, 1.0);
//
//
//            double minusSteerSpeed = speed - steer;
//            double plusSteerSpeed = speed + steer;
//
//
//            double max = Math.max(Math.abs(minusSteerSpeed), Math.abs(plusSteerSpeed));
//            if (max > 1.0){
//                minusSteerSpeed /= max;
//                plusSteerSpeed /= max;
//            }
//
//            if (frontRight.getTargetPosition() > frontRight.getCurrentPosition()){
//                frontRight.setPower(plusSteerSpeed);
//            }
//            else{
//                frontRight.setPower(minusSteerSpeed);
//            }
//            if (backRight.getTargetPosition() > backRight.getCurrentPosition()){
//                backRight.setPower(plusSteerSpeed);
//            }
//            else{
//                backRight.setPower(minusSteerSpeed);
//            }
//
//            if (frontLeft.getTargetPosition() > frontLeft.getCurrentPosition()){
//                frontLeft.setPower(minusSteerSpeed);
//            }
//            else{
//                frontLeft.setPower(plusSteerSpeed);
//            }
//            if (backLeft.getTargetPosition() > backLeft.getCurrentPosition()){
//                backLeft.setPower(minusSteerSpeed);
//            }
//            else{
//                backLeft.setPower(plusSteerSpeed);
//            }
//
//            double sumEncoderError = Math.abs(frontLeft.getCurrentPosition() - frontLeft.getTargetPosition())+
//                    Math.abs(backLeft.getCurrentPosition() - backLeft.getTargetPosition()) +
//                    Math.abs(frontRight.getCurrentPosition() - frontRight.getTargetPosition()) +
//                    Math.abs(backRight.getCurrentPosition() - backRight.getTargetPosition());
//            double sumInchesError = sumEncoderError / COUNTS_PER_INCH;
//            if (sumInchesError < 2){
//                break;
//            }
//
//            // Display drive status for the driver.
//            base.getTelemetry().addData("Err/St",  "%5.1f/%5.1f",  error, steer);
//            base.getTelemetry().addData("Target",  "%7d:%7d",      frontLeftCounts,  frontRightCounts);
//            base.getTelemetry().addData("Actual",  "%7d:%7d",      frontLeft.getCurrentPosition(),
//                    frontRight.getCurrentPosition());
//            base.getTelemetry().addData("Speed",   "%5.2f:%5.2f",  minusSteerSpeed, plusSteerSpeed);
//            base.getTelemetry().update();
//
//        }
//        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
    public void gyroDrive(double speed, double forwardInches, double rightInches, double angle){
        double frontLeftCounts = ((forwardInches + (1.414 * rightInches)));
        double frontRightCounts =((forwardInches - (1.414 * rightInches)));
        gyroDrive(speed,frontLeftCounts,frontRightCounts,frontRightCounts,frontLeftCounts,angle,0);
    }

    public void encoderDrive(double speed, double xInches, double yInches, double timeout){
        double frontLeftDistance = yInches + (1.414 * xInches);
        double frontRightDistance = yInches - (1.414 * xInches);
        encoderDrive(speed, frontLeftDistance, frontRightDistance, frontRightDistance, frontLeftDistance, timeout);
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */


    public void gyroTurn ( double speed, double angle){

        // keep looping while we are still active, and not on heading.
        while (base().getOpMode().opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            base().getTelemetry().update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */


    public void gyroHold ( double speed, double angle, double holdTime){

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (base().getOpMode().opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            base().getTelemetry().update();


        }

        // Stop all motion;
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading ( double speed, double angle, double PCoeff){
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        frontLeft.setPower(leftSpeed);
        backLeft.setPower(leftSpeed);
        backRight.setPower(rightSpeed);
        frontRight.setPower(rightSpeed);

        // Display it for the driver.
        base().getTelemetry().addData("Target", "%5.2f", angle);
        base().getTelemetry().addData("Err/St", "%5.2f/%5.2f", error, steer);
        base().getTelemetry().addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError ( double targetAngle){

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyroSensor.getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer ( double error, double PCoeff){
        return Range.clip(error * PCoeff, -DRIVE_SPEED, 1);
    }

    public void encoderDrive ( double speed,
                               double frontLeftInches, double frontRightInches, double backLeftInches,
                               double backRightInches,
                               double timeoutS){
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        double ErrorAmount;
        boolean goodEnough = false;
        // Ensure that the opmode is still active
        if (base().getOpMode().opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (base().getOpMode().opModeIsActive() &&
                    (frontLeft.isBusy() && frontRight.isBusy()) && (backLeft.isBusy() && backRight.isBusy()) && !goodEnough) {

                // Display it for the driver.
                base().getTelemetry().addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                base().getTelemetry().addData("Path2", "Running at %7d :%7d",

                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(),
                        backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition());
                base().getTelemetry().addData("frontLeft", frontLeft.getCurrentPosition());
                base().getTelemetry().addData("backLeft", backLeft.getCurrentPosition());
                base().getTelemetry().addData("frontRight", frontRight.getCurrentPosition());
                base().getTelemetry().addData("backright", backRight.getCurrentPosition());

                base().getTelemetry().update();

                ErrorAmount = ((Math.abs(((newBackLeftTarget) - (backLeft.getCurrentPosition())))
                        + (Math.abs(((newFrontLeftTarget) - (frontLeft.getCurrentPosition()))))
                        + (Math.abs((newBackRightTarget) - (backRight.getCurrentPosition())))
                        + (Math.abs(((newFrontRightTarget) - (frontRight.getCurrentPosition()))))) / COUNTS_PER_INCH);
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }
            }

            // Stop all motion;

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void driveToFirstBox(){
        double DIST_TO_FIRST = 42;
            this.gyroDrive(DRIVE_SPEED,DIST_TO_FIRST,DIST_TO_FIRST,DIST_TO_FIRST,DIST_TO_FIRST,STRAIGHT,0);

        //checkEncoders();
    }

    public void lineUpWithLine(){
        double DIST_TO_LINE = 9;
        this.gyroTurn(TURN_SPEED,STRAIGHT);
        this.gyroDrive(DRIVE_SPEED,-DIST_TO_LINE,-DIST_TO_LINE,-DIST_TO_LINE,-DIST_TO_LINE,STRAIGHT,0);
        this.gyroTurn(TURN_SPEED,STRAIGHT);

    }

    public void driveToSecondBox(){
        final double DIST_TO_LINE = 60;
        final double DIST_TO_STRAFE = 22;
        this.encoderDrive(DRIVE_SPEED, -DIST_TO_STRAFE, DIST_TO_STRAFE, DIST_TO_STRAFE, -DIST_TO_STRAFE, 0);
        this.gyroDrive(DRIVE_SPEED, DIST_TO_LINE, DIST_TO_LINE, DIST_TO_LINE, DIST_TO_LINE, STRAIGHT,0);
        this.encoderDrive(DRIVE_SPEED, DIST_TO_STRAFE, -DIST_TO_STRAFE, -DIST_TO_STRAFE, DIST_TO_STRAFE, 0);
        this.gyroTurn(TURN_SPEED, 0);
    }
    public void driveToThirdBox(){
        double distToThree = 80;
        double avoidingRings = 21;
        this.encoderDrive(DRIVE_SPEED,-avoidingRings,avoidingRings,avoidingRings,-avoidingRings,0);
        this.gyroDrive(DRIVE_SPEED,distToThree,distToThree,distToThree,distToThree,0,0);
        //checkEncoders();
    }
    public enum NUM_OF_RINGS{
        ZERO, ONE, FOUR
    }
    public void Park(NUM_OF_RINGS numOfRings){
        final double STRAFE_TO_LINE;
        final double DRIVE_TO_LINE;
        switch (numOfRings){
            case FOUR:
                STRAFE_TO_LINE = 6.0;
                DRIVE_TO_LINE = 15.0;
//                this.encoderDrive(DRIVE_SPEED,STRAFE_TO_LINE,STRAFE_TO_LINE,STRAFE_TO_LINE, STRAFE_TO_LINE,0);
                this.gyroDrive(DRIVE_SPEED,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,STRAIGHT,0);
                break;
            case ONE:
                STRAFE_TO_LINE = 6.0;
                DRIVE_TO_LINE = 10.0;
//                this.encoderDrive(DRIVE_SPEED,STRAFE_TO_LINE,STRAFE_TO_LINE,STRAFE_TO_LINE, STRAFE_TO_LINE,0);
                this.gyroDrive(DRIVE_SPEED,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,STRAIGHT,0);
                break;

            case ZERO:
                STRAFE_TO_LINE = 6.0;
                DRIVE_TO_LINE = 6.5;
//                this.encoderDrive(DRIVE_SPEED,STRAFE_TO_LINE,STRAFE_TO_LINE,STRAFE_TO_LINE, STRAFE_TO_LINE,0);
                this.gyroDrive(DRIVE_SPEED,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,DRIVE_TO_LINE,STRAIGHT,0);
                break;
        }

    }

    public void checkEncoders(){
    while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
        base().getTelemetry().addData("FrontLeft:", frontLeft.getCurrentPosition());
        base().getTelemetry().addData("FrontRight:", frontRight.getCurrentPosition());
        base().getTelemetry().addData("BackLeft:", backLeft.getCurrentPosition());
        base().getTelemetry().addData("BackRight:", backRight.getCurrentPosition());
        base().getTelemetry().update();
        }
    }
}
