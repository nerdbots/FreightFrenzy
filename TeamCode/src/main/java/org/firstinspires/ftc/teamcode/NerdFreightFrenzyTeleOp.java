/*
Copyright 2018 FIRST Tech Challenge Team [Phone] SAMSUNG

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.acmerobotics.dashboard.FtcDashboard;

import java.util.Locale;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Disabled
@TeleOp(name="FreigntFrenzyTeleOp", group="Final")
@Config
public class NerdFreightFrenzyTeleOp extends LinearOpMode {
    private BNO055IMU imu;
    private DcMotor frontRightMotor;
    private DcMotor rearRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor rearLeftMotor;

    FtcDashboard ftcDashboard;


    //ARM Components

    //ARM Start

    private Servo leftGrab;
    private Servo rightGrab;
    private Servo leftArmServo;
    private Servo rightArmServo;
    private DcMotor leftArmMotor;
    private DcMotor rightArmMotor;

    public static double armKp = 0.01;
    public static double armKi = 0.0;
    public static double armKd = 0.0002;
    public static double maxPower = 0.4;

    private double previousTime = 0.0;
    private double currentError = 0.0;
    private double totalError = 0.0;

    private double deadBand = 10.0;
    private ElapsedTime elapsedTime;

    public static int armServoPosition = 0;
    public static int grabPosition = 0;

    public static int armTarget = 0;

    public static double rightArmTargetPosition;
    public static double leftArmTargetPosition;
    public static double leftGrabTargetPosition;
    public static double rightGrabTargetPosition;

    public static double armMotorsign = 1.0;

    double propError = 0;
    double intError = 0;
    double loopTime = 0;
    double prevDerError = 0;
    double derError = 0;
    double angletolerance = 0;
    double motorPower = 0;
    double currentTime = 0;
    double oldTime = 0;
    double deltaTime = 0;
    double startTime = 0;

    //ARM End
    Orientation angles;
    Acceleration gravity;

    Orientation lastAngles = new Orientation();

    double globalAngle = 0.0;


    private ElapsedTime ZPIDTime = new ElapsedTime();

    private ElapsedTime PIDTime = new ElapsedTime();

    private double ZPrevError = 0;


    private double ZTotalError = 0;


    private double ZSpeed = 0;


    private double ZDerror = 0;


    private double ZkP = 0.013; //0.011
    private double ZkI = 0.000; //0.000
    private double ZkD = 0.0013;//0.00145


    private double ZTar = 0;

    private double MaxSpeedZ = 1.0;


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "Rear_Left_Motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "Rear_Right_Motor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);


        resetAngle();

        //Initialize Arm Components
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");
        leftGrab = hardwareMap.get(Servo.class, "leftGrab");
        rightGrab = hardwareMap.get(Servo.class, "rightGrab");
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Positions to get in the intake
        rightArmServo.setPosition(1);
        leftArmServo.setPosition(0);
        leftGrab.setPosition(0.53);
        rightGrab.setPosition(0.55);
        //End Positions to get in the intake

        telemetry.addData("LeftGrabPos", leftGrab.getPosition());
        telemetry.addData("RightGrabPos", rightGrab.getPosition());
        telemetry.update();

        // End Initialize Arm Components

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)


        double positionPitch = 0.52;  // (MAX_POS - MIN_POS) / 2;
        double positionAngle = 0.15;  //(MAX_POS - MIN_POS) / 2; 0.25
        double tapeSpeed = 0.0;


        double LMP = 0;
        double RMP = 0;
        double FMP = 0;
        double BMP = 0;

        double FX = 0;
        double FY = 0;

        double CA = 0;

        double RSA = 0;

        double Mag = 0;
        double zMag = 0;

        double mult = 1; //THIS IS SPEED
        double multZ = 0.6;//0.3

        double power = 1;
        double upMult = 1;

        double joyX = 0;
        double joyY = 0;

        //int targetR = -80;
        //int targetF = 60;


        if (!imu.isGyroCalibrated()) {
            telemetry.addData("Gyro", "Not Initialized");
            telemetry.update();
        } else {
            telemetry.addData("Gyro", "Initialized");
            telemetry.update();
        }

        armTarget = 0;

        double armMotorPower = 0.0;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        waitForStart();
//        formatAngle(angles.angleUnit, angles.thirdAngle);
//
//        dashboardTelemetry.addData("Angle", angles.thirdAngle);
//        dashboardTelemetry.update();

        //   resetAngle();

        ZPIDTime.reset();

        elapsedTime = new ElapsedTime();

        //Initial Arm Target and Position.
        armTarget = 0;
        armServoPosition = 1;
        grabPosition = 0;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            currentTime = elapsedTime.seconds();
            loopTime = currentTime - oldTime;
            oldTime = currentTime;
            deltaTime = currentTime - startTime;

            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                joyX = gamepad1.left_stick_x;
                joyY = gamepad1.left_stick_y;
            }

            if (gamepad1.dpad_up) {
                joyX = 0;
                joyY = -1;
            } else if (gamepad1.dpad_down) {
                joyX = 0;
                joyY = 1;
            } else if (gamepad1.dpad_left) {
                joyX = -1;
                joyY = 0;
            } else if (gamepad1.dpad_right) {
                joyX = 1;
                joyY = 0;
            }


            if (gamepad1.y) {
                resetAngle();
            }


            if (gamepad1.b) {
                BNO055IMU.Parameters parametersb = new BNO055IMU.Parameters();

                parametersb.mode = BNO055IMU.SensorMode.IMU;
                parametersb.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parametersb.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parametersb.loggingEnabled = false;
                parametersb.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parametersb.loggingTag = "IMU";
                parametersb.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

                imu.initialize(parametersb);

            }


            zMag = (joyX * joyX) + (joyY * joyY);


            if (Math.sqrt(zMag) > 0.5) {
                ZTar = Math.atan2(-joyX, -joyY) * 180 / 3.14159;

            }


            if (gamepad1.right_bumper) {
                multZ = 0.3;
                mult = 0.3;
            } else {
                multZ = 0.6;
                mult = 1;
            }

            //ARM Components
            if (gamepad2.b) {

                maxPower = 0.6;
                armTarget = 80;
                armServoPosition = 1;
                dashboardTelemetry.addData("LeftMotorPos", leftArmMotor.getCurrentPosition());
                //But actually the right motor
                dashboardTelemetry.update();

            }
            if (gamepad2.x) {
                maxPower = 0.6;
                armTarget = 150;
                armServoPosition = 3;
            }
            if (gamepad2.y) {
                maxPower = 0.4;
                armTarget = 100;
                armServoPosition = 1;

            }


            //ARM


            CA = (Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) * 180 / 3.14) + 45;

            RSA = (CA - getAngle()) * 3.14 / 180;

            Mag = Math.sqrt((gamepad1.right_stick_y * gamepad1.right_stick_y) + (gamepad1.right_stick_x * gamepad1.right_stick_x));

            FX = -Math.sin(RSA) * Mag;
            FY = -Math.cos(RSA) * Mag;


            LMP = (ZSpeed * multZ) + FX; //multZ will only affect Z. This is because if joypad Z is zero then Z is zero.
            RMP = (ZSpeed * multZ) - FX;
            FMP = (ZSpeed * multZ) + FY;
            BMP = (ZSpeed * multZ) - FY;

            frontLeftMotor.setPower(RMP * mult);
            rearRightMotor.setPower(LMP * mult);


            rearLeftMotor.setPower(FMP * mult);
            frontRightMotor.setPower(BMP * mult);


            //Arm
            double armPidOutput = 0.0;

//            armPidOutput = getArmPIDOutput(armTarget,leftArmMotor.getCurrentPosition() * -1);
            armPidOutput = armPID(armTarget, leftArmMotor.getCurrentPosition() * -1);
            armMotorsign = Math.signum(armPidOutput);
            if (Math.abs(armPidOutput) > this.maxPower) {
                armMotorPower = armMotorsign * this.maxPower;
            } else {
                armMotorPower = armPidOutput;
            }
//            leftArmMotor.setPower(armMotorPower);
//            rightArmMotor.setPower(armMotorPower);
//            setBothArmServoPosition(armServoPosition);
            leftArmServo.setPosition(leftArmTargetPosition);
            rightArmServo.setPosition(1.0 - leftArmTargetPosition);
            setGrabPosition(0);

            //Arm


            //add telemetry


            telemetry.addData("X", FX);
            telemetry.addData("Y", FY);
            telemetry.addData("CA", CA);
            telemetry.addData("RSA", RSA);
            telemetry.addData("RA", getAngle());
//            dashboardTelemetry.addData("Angle", getAngle());
//            dashboardTelemetry.update();
            telemetry.addData("zMag", zMag);
            telemetry.addData("ZTar", ZTar);

            telemetry.addData("FREV", frontRightMotor.getCurrentPosition());
            telemetry.addData("FLEV", frontLeftMotor.getCurrentPosition());
            telemetry.addData("RREV", rearRightMotor.getCurrentPosition());
            telemetry.addData("RLEV", rearLeftMotor.getCurrentPosition());



            telemetry.addData("Status", "Running");
            telemetry.update();

            dashboardTelemetry.addData("LeftMotorPos", leftArmMotor.getCurrentPosition());
            dashboardTelemetry.update();
            //  }

        }
    }

    public void setBothArmServoPosition(int targetPos) {
        if (targetPos == 1) { //Bent Downwards to get in the intake
            rightArmServo.setPosition(1);
            leftArmServo.setPosition(0);
        } else if (targetPos == 0) { // Flat
            rightArmServo.setPosition(.5);
            leftArmServo.setPosition(.5);
        } else if (targetPos == 2) { // Bent downwards opposite of intake
            rightArmServo.setPosition(0);
            leftArmServo.setPosition(1);
        } else if (targetPos == 3) { // Ground Pickup
            rightArmServo.setPosition(0.3);
            leftArmServo.setPosition(0.7);
        }

    }

    private void setGrabPosition(int pos) {
        if (pos == 0) { //INSERT -  Get in the in take
            leftGrab.setPosition(0.53);
            rightGrab.setPosition(0.55);
        } else if (pos == 1) { // READY FOR INTAKE - Expand slightly to grab ball/cube
            leftGrab.setPosition(0.55);
            rightGrab.setPosition(0.53);
        } else if (pos == 2) { //GrAB
            leftGrab.setPosition(0.5);
            rightGrab.setPosition(0.65);
        } else if (pos == 3) { //Release
            leftGrab.setPosition(0.6);
            rightGrab.setPosition(0.5);
        } else {
            telemetry.addData("Error", "Invalid grabPosition Target");
            telemetry.update();
        }
    }

    private double armPID(double targetValue, double currentValue) {


        propError = (targetValue - currentValue);


        intError += (targetValue - currentValue) * loopTime;

        derError = ((targetValue - currentValue) - prevDerError) / loopTime;
        prevDerError = targetValue - currentValue;

        if (Math.abs(targetValue - currentValue) < angletolerance) {
//                    onTarget = true;
//                    runTest = false;
            motorPower = propError * armKp + intError * armKi;
            intError = 0;
        } else {
            motorPower = propError * armKp + intError * armKi + derError * armKd;
        }
//        if (Math.abs(targetValue - currentValue) < 0.5) {
//            motor1.setPower(0);
//       }
        double motorPowersin = Math.signum(motorPower);
        if (Math.abs(motorPower) > maxPower) {
            motorPower = maxPower * motorPowersin;

        }
        return /*-*/(motorPower);
    }

    public double getArmPIDOutput(double targetPosition, int currentPosition) {
        double pOutput = 0.0;
        double iOutput = 0.0;
        double dOutput = 0.0;
        double output = 0.0;

        final String funcName = "getOutput";

        //Before we get the error for this cycle, store the error from previous cycle in previous error.
        double prevError = currentError;
        //Store the current time. Will use this for calculating time between 2 cycles - delta time.
        double currTime = elapsedTime.seconds();
        //Find delta time, difference between time in this cycle and the previous.
        double deltaTime = currTime - previousTime;
        //Store the current time into previous time for using in next cycle.
        previousTime = currTime;
        //Call function to get the error based ont the set target and device input
        currentError = targetPosition - currentPosition;

        //Total error for finding I component is sum of errors in each cycle multiplied by delta time.
        totalError += (currentError * deltaTime);
        //Calculate P, I and D outputs.
        pOutput = this.armKp * currentError;
        iOutput = this.armKi * totalError;
        dOutput = deltaTime > 0.0 ? this.armKd * (currentError - prevError) / (deltaTime * 1000) : 0.0;
        //Total PID output.
        //Since we are using device input instead of error for calculating dOutput, we make it negative.
        output = pOutput + iOutput - dOutput;

        //Return the output to the caller.
        return output;
    }


    //0 is rearMotor 1 is frontMotor \/
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }


    //Function to get the angle of the Gyro sensor
    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle;
        deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));


    }
}
