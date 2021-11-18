/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//Redo field centric Auton to be consistent with the right triangle diagram.
@Disabled
@Autonomous(name="tooMuchPressure", group="Linear Opmode")
//@Disabled
public class tooMuchPressure extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private BNO055IMU imu;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    private DcMotor frontEncoder;
    private DcMotor rightEncoder;
    private DcMotor leftEncoder;
    private DcMotor backEncoder;

    Orientation angles;
    Acceleration gravity;

    Orientation             lastAngles = new Orientation();

    double robotAngleToField = 0;
    double robotAngleToTarget = 0;

    double robotTargetSpeed = 0.5;
    double robotTargetAngle = 0;

    double xPower = 0;
    double yPower = 0;
    double zPower = 0;

    double frontLeftMotorPower = 0;
    double frontRightMotorPower = 0;
    double rearLeftMotorPower = 0;
    double rearRightMotorPower = 0;

    double robotRadius = 9.75; //robot radius in inches
    double robotCircumference;
    double robotWheelRadius = 3.5;  //robot wheel radius in inches
    double robotWheelCircumference;
    double wheelRotPerRobotRot;

//    double flCounts;
//    double frCounts;
//    double rlCounts;
//    double rrCounts;

    double xPosition = 0;
    double yPosition = 0;

    double robotRotNewOpt = 0;
    double robotRotOldOpt = 0;
    double robotRotOpt = 0;
    //double robotRotDisplacementOptF = 0;
    double robotXdisplacementOpt = 0;
    double robotYdisplacementOpt = 0;
    double robotVectorByOdoOpt = 0;
    double robotVectorMagOpt = 0;
    double robotFieldAngleOpt = 0;
    double robotXdisplacementOptTot = 0;
    double robotYdisplacementOptTot = 0;
    double frontPositionOptical = 0;
    double rightPositionOptical = 0;
    double leftPositionOptical = 0;
    double backPositionOptical = 0;
    double [] robotPositionXYOptical;
    double frontDisplacementOld = 0;
    double leftDisplacementOld = 0;
    double rightDisplacementOld = 0;
    double rearDisplacementOld = 0;
    double frontDispNoRotTotOpt = 0;
    double leftDispNoRotTotOpt = 0;
    double rightDispNoRotTotOpt = 0;
    double rearDispNoRotTotOpt = 0;
    double omniDriveFactorOpt = 0;
    double xPositionOpt = 0;
    double yPositionOpt = 0;
    double flme;
    double frme;
    double rlme;
    double rrme;
    double flme_old = 0;
    double frme_old = 0;
    double rlme_old = 0;
    double rrme_old = 0;
    double flme_speed;
    double frme_speed;
    double rlme_speed;
    double rrme_speed;


    double robotRot = 0;
    double robotRotNew = 0;
    double robotRotOld = 0;
    double robotRotDisplacement = 0;
    double robotVectorByOdo = 0;
    double rfDisplacement = 0;
    double rfDisplacementNew = 0;
    double rfDisplacementOld = 0;
    double lfDisplacement = 0;
    double lfDisplacementNew = 0;
    double lfDisplacementOld = 0;
    double rrDisplacement = 0;
    double rrDisplacementNew = 0;
    double rrDisplacementOld = 0;
    double lrDisplacement = 0;
    double lrDisplacementNew = 0;
    double lrDisplacementOld = 0;
    double rfDispNoRot = 0;
    double lfDispNoRot = 0;
    double rrDispNoRot = 0;
    double lrDispNoRot = 0;
    double robotVectorMag = 0;
    double rfDispNoRotTot = 0;
    double lfDispNoRotTot = 0;
    double rrDispNoRotTot = 0;
    double lrDispNoRotTot = 0;
    double omniDriveAngle = 0;
    double omniDriveFactor = 0;
    //    double robotFieldPositionXTicks = 0;
//    double robotFieldPositionYTicks = 0;
    double robotFieldPositionX = 0;
    double robotFieldPositionY = 0;
    double robotFieldAngle = 0;
    double displacementX = 0;
    double displacementY = 0;
    double [] robotPositionXY;
    double robotXdisplacement = 0;
    double robotYdisplacement = 0;

    int targetPositionFLM = 0;
    int targetPositionFRM = 0;
    int targetPositionRLM = 0;
    int targetPositionRRM = 0;

    double currentTime = 0;
    double deltaTime = 0;
    double startTime = 0;
    double oldTime = 0;
    double loopTime = 0;

    private ElapsedTime elapsedTime = new ElapsedTime();

    //private final String instanceName;
    private boolean debugFlag = true;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "Rear_Left_Motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "Rear_Right_Motor");

        //frontEncoder = hardwareMap.get(DcMotor.class, "Front");
        rightEncoder = hardwareMap.get(DcMotor.class, "rightArmMotor");
        leftEncoder = hardwareMap.get(DcMotor.class, "Ducky_Disk");
        backEncoder = hardwareMap.get(DcMotor.class, "Intake");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        resetAngle();


        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        while (!isStopRequested() && !imu.isGyroCalibrated()) {

            sleep(50);
            idle();
        }

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        xPosition = 0;
        yPosition = 0;

        displacementX = 0;
        displacementY = 0;

        //frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //this.frontEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //this.frontEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        rightEncoder.setDirection(DcMotor.Direction.REVERSE);
        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        backEncoder.setDirection(DcMotor.Direction.FORWARD);

        xPositionOpt = 0;
        yPositionOpt = 0;


        targetPositionFLM = frontLeftMotor.getCurrentPosition() + 1000;
        targetPositionFRM = frontRightMotor.getCurrentPosition() + 1000;
        targetPositionRLM = rearLeftMotor.getCurrentPosition() + 1000;
        targetPositionRRM = rearRightMotor.getCurrentPosition() + 1000;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        startTime = elapsedTime.seconds();

        while (opModeIsActive() && !isStopRequested()) {

            //First calculate motor speeds for linear (x, y) motion

            currentTime = elapsedTime.seconds();
            loopTime = currentTime - oldTime;
            oldTime = currentTime;
            deltaTime = currentTime - startTime;

            frontLeftMotor.setTargetPosition(targetPositionFLM);
            frontRightMotor.setTargetPosition(targetPositionFRM);
            rearLeftMotor.setTargetPosition(targetPositionRLM);
            rearRightMotor.setTargetPosition(targetPositionRRM);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rearRightMotor.setPower(0.4);
            frontLeftMotor.setPower(0.4);
            frontRightMotor.setPower(0.4);
            rearLeftMotor.setPower(0.4);

            robotPositionXY = findDisplacementOptical();

            xPosition += robotPositionXY[4];
            yPosition += robotPositionXY[5];

            flme = frontLeftMotor.getCurrentPosition();
            frme = frontRightMotor.getCurrentPosition();
            rlme = rearLeftMotor.getCurrentPosition();
            rrme = rearRightMotor.getCurrentPosition();
            flme_speed = (flme - flme_old) / loopTime;
            frme_speed = (frme - frme_old) / loopTime;
            rlme_speed = (rlme - rlme_old) / loopTime;
            rrme_speed = (rrme - rrme_old) / loopTime;
            flme_old = flme;
            frme_old = frme;
            rlme_old = rlme;
            rrme_old = rrme;

            if (debugFlag) {
                RobotLog.d("tooMuchPressure - timeSinceStart %f, robotAngle %f, xPosition %f, yPosition %f, rightPositionOptical %f, leftPositionOptical %f, backPositionOptical %f, flme %f, flme_speed %f, frme %f, frme_speed %f,  rlme %f, rlme_speed %f,  rrme %f, rrme_speed %f",
                        deltaTime, robotRotNewOpt, xPosition, yPosition, rightPositionOptical, leftPositionOptical, backPositionOptical, flme, flme_speed, frme, frme_speed, rlme, rlme_speed, rrme, rrme_speed);
            }

            //robotTargetAngle = robotTargetAngle + 1;

            telemetry.addData("xDiplacement", xPosition);
            telemetry.addData("yDisplacement", yPosition);
            telemetry.addData("Right", rightPositionOptical);
            telemetry.addData("Left", leftPositionOptical);
            telemetry.addData("Back", backPositionOptical);
            telemetry.update();

        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        telemetry.addData("xDiplacement", xPosition);
        telemetry.addData("yDisplacement", yPosition);
        telemetry.addData("Right", rightPositionOptical);
        telemetry.addData("Left", leftPositionOptical);
        telemetry.addData("Back", backPositionOptical);
        telemetry.update();

    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robotAngleToField = 0;
    }


    //Function to get the angle of the Gyro sensor
    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        robotAngleToField += deltaAngle;

        lastAngles = angles;

        return robotAngleToField;
    }

    private double [] findDisplacementOptical(){

        //First, determine the robot z movement, so encoder ticks caused by z movement can be removed from x, y movement
        robotRotNewOpt = getAngle();
        robotRotOpt = robotRotNewOpt - robotRotOldOpt;
        robotRotOldOpt = robotRotNewOpt;

        //robot rotation (each loop) expressed in motor ticks (robot angle, ticks per degree robot rotation...determined through testing for each encoder wheel).
        //double robotRotDisplacementOptFront = robotRotOpt * 0; //21.390 ticks per degree of robot rotation
        double robotRotDisplacementOptRight = robotRotOpt * 40.554; //21.085each wheel is mounted slightly different on the bot
        double robotRotDisplacementOptLeft = robotRotOpt * 46.368; //21.318
        double robotRotDisplacementOptBack = robotRotOpt * 10.152; //21.093

        //measure encoder position
        //frontPositionOptical = frontEncoder.getCurrentPosition();
        frontPositionOptical = 0;
        rightPositionOptical = rightEncoder.getCurrentPosition();
        leftPositionOptical = leftEncoder.getCurrentPosition();
        backPositionOptical = backEncoder.getCurrentPosition();

        //encoder ticks for each sensor, each loop
        //double frontDisplacement = frontPositionOptical - frontDisplacementOld;
        double leftDisplacement = leftPositionOptical - leftDisplacementOld;
        double rightDisplacement = rightPositionOptical - rightDisplacementOld;
        double rearDisplacement = backPositionOptical - rearDisplacementOld;

        frontDisplacementOld = frontPositionOptical;
        leftDisplacementOld = leftPositionOptical;
        rightDisplacementOld = rightPositionOptical;
        rearDisplacementOld = backPositionOptical;

        //Now, remove the ticks caused by z movement from the total encoder count, each loop
        //double frontDispNoRot = frontDisplacement - robotRotDisplacementOptFront;
        double leftDispNoRot = leftDisplacement - robotRotDisplacementOptRight;
        double rightDispNoRot = rightDisplacement - robotRotDisplacementOptLeft;
        double rearDispNoRot = rearDisplacement - robotRotDisplacementOptBack;

        //This section was created for debugging
        //frontDispNoRotTotOpt += frontDispNoRot;
        leftDispNoRotTotOpt += leftDispNoRot;
        rightDispNoRotTotOpt += rightDispNoRot;
        rearDispNoRotTotOpt += rearDispNoRot;

        //The optical encoders are mounted inline to the robot preferred direction, so we can just use robot angle to target...difference between robot target angle and gyro angle
//        omniDriveAngle = robotAngleToTarget + 45;

        //Determine the omni-Drive wheel effective gear ratio
        if (Math.abs(Math.cos(robotAngleToTarget * Math.PI / 180)) > 0.707) {
            omniDriveFactorOpt = Math.abs(Math.cos(robotAngleToTarget * Math.PI / 180));
        }
        else if (Math.abs(Math.sin(robotAngleToTarget * Math.PI / 180)) > 0.707) {
            omniDriveFactorOpt = Math.abs(Math.sin(robotAngleToTarget * Math.PI / 180));
        }
        else {
            omniDriveFactorOpt = 1.0;
        }

        //calculate X displacement, and convert ticks to inches (2.362 = wheel diameter inches, 1440 = ticks per wheel rot), and account for robot angle and omni wheel effect
//        robotXdisplacementOpt = ((-frontDispNoRot + rearDispNoRot) / 2) * ((2.362 * Math.PI) / 8192);
        robotXdisplacementOpt = rearDispNoRot * ((2.362 * 1.01 * Math.PI) / 8192); //added 2% error factor to improve accuracy.  Maybe the wheels are bigger than 60mm
//        robotXdisplacementOpt = ((-frontDispNoRot + rearDispNoRot) / 2) * ((2.362 * Math.PI) / 1440) / omniDriveFactorOpt;
        //robotXdisplacementOptTot = ((rightDispNoRotTotOpt + leftDispNoRotTotOpt) / 2 + (-frontDispNoRotTotOpt + rearDispNoRotTotOpt) / 2) * ((2.362 * 1.01 * Math.PI) / 8192);
        //calculate Y displacement, and convert ticks to inches (2.362 = wheel diameter inches, 1440 = ticks per wheel rot), and account for robot angle and omni wheel effect
        robotYdisplacementOpt = ((rightDispNoRot - leftDispNoRot) / 2) * ((2.362 * 1.01 * Math.PI) / 8192);
//        robotYdisplacementOpt = ((rightDispNoRot - leftDispNoRot) / 2) * ((2.362 * Math.PI) / 1440) / omniDriveFactorOpt;
        //robotYdisplacementOptTot = (((rightDispNoRotTotOpt - leftDispNoRotTotOpt) / 2) + ((frontDispNoRotTotOpt + rearDispNoRotTotOpt) / 2)) * ((2.362 * 1.01 * Math.PI) / 8192);

        //Using inverse kinematics, calculate the robot driving direction, from the encoder measurements
        robotVectorByOdoOpt = Math.atan2(robotYdisplacementOpt, robotXdisplacementOpt) * 180 / Math.PI;

        //Now that we know the robot driving direction, calculate the driving distance, each loop
        robotVectorMagOpt = Math.sqrt((robotXdisplacementOpt * robotXdisplacementOpt) + (robotYdisplacementOpt * robotYdisplacementOpt));

        //The calculated robot vector is the direction in field centric.  Adding the robot gyro angle was an error.
        robotFieldAngleOpt = (robotVectorByOdoOpt + getAngle());
        //robotFieldAngleOpt = robotVectorByOdoOpt;

        //Now we know the driving direction and distance for each loop, use forward kinematics calculation to determine x, y movement, each loop
        double robotFieldPositionXOpt = robotVectorMagOpt * Math.cos(robotFieldAngleOpt * Math.PI / 180);  //field position in inches
        double robotFieldPositionYOpt = robotVectorMagOpt * Math.sin(robotFieldAngleOpt * Math.PI / 180);  //field position in inches

        //Add each x, y loop calculation, to track the robot location on the field
        xPositionOpt += robotFieldPositionXOpt;
        yPositionOpt += robotFieldPositionYOpt;

//        if (debugFlag) {
//            RobotLog.d("findDisplacementOptical - runTime %f, deltaTime %f, leftDispNoRot %f, rightDispNoRot %f, rearDispNoRot %f, xPositionOptical %f, yPositionOptical %f, xPower %f, yPower %f, zPower %f, robotRotNewOpt %f, robotVectorByOdoOpt %f, robotFieldAngleOpt %f",
//                    currentTime, loopTime, leftDispNoRot, rightDispNoRot, rearDispNoRot, xPositionOpt, yPositionOpt, xPower, yPower, zPower, robotRotNewOpt, robotVectorByOdoOpt, robotFieldAngleOpt);
//
//        }

        //Store the encoder positions and x, y locations in an array and return the values
        double [] positionOptical = {frontPositionOptical, rightPositionOptical, leftPositionOptical, backPositionOptical, xPositionOpt, yPositionOpt, robotVectorByOdoOpt};
        return positionOptical;


    }

    public double [] findDisplacement(double displacementX, double displacementY){

        robotRotNew = getAngle();
        robotRot = robotRotNew - robotRotOld;
        robotRotOld = robotRotNew;

        robotRotDisplacement = robotRot * 8.3106;  //robot rotation expressed in motor ticks (robot angle, then motor displacement, then wheel rotations, then ticks per rotation

        rfDisplacementNew = frontRightMotor.getCurrentPosition();
        lfDisplacementNew = frontLeftMotor.getCurrentPosition();
        lrDisplacementNew = rearLeftMotor.getCurrentPosition();
        rrDisplacementNew = rearRightMotor.getCurrentPosition();

        rfDisplacement = rfDisplacementNew - rfDisplacementOld;
        lfDisplacement = lfDisplacementNew - lfDisplacementOld;
        lrDisplacement = lrDisplacementNew - lrDisplacementOld;
        rrDisplacement = rrDisplacementNew - rrDisplacementOld;

        rfDisplacementOld = rfDisplacementNew;
        lfDisplacementOld = lfDisplacementNew;
        lrDisplacementOld = lrDisplacementNew;
        rrDisplacementOld = rrDisplacementNew;

        lfDispNoRot = lfDisplacement - robotRotDisplacement;
        rrDispNoRot = rrDisplacement - robotRotDisplacement;
        rfDispNoRot = rfDisplacement - robotRotDisplacement;
        lrDispNoRot = lrDisplacement - robotRotDisplacement;

        lfDispNoRotTot += robotRotDisplacement;
        rrDispNoRotTot += robotRotDisplacement;
        rfDispNoRotTot += robotRotDisplacement;
        lrDispNoRotTot += robotRotDisplacement;

        omniDriveAngle = robotAngleToTarget + 45;

//        if (Math.abs(Math.cos(omniDriveAngle * Math.PI / 180)) > 0.707) {
//            omniDriveFactor = Math.abs(Math.cos(omniDriveAngle * Math.PI / 180));
//        }
//        else if (Math.abs(Math.sin(omniDriveAngle * Math.PI / 180)) > 0.707) {
//            omniDriveFactor = Math.abs(Math.sin(omniDriveAngle * Math.PI / 180));
//        }
//        else {
//            omniDriveFactor = 1.0;
//        }
        omniDriveFactor = 1.0;

        //calculate X displacement, and convert ticks to inches (3.543 = wheel diameter inches, 560 = ticks per wheel rot), and account for robot angle
        robotXdisplacement = ((-rfDispNoRot - lfDispNoRot + rrDispNoRot + lrDispNoRot) / 4) * ((3.543 * Math.PI) / 540) / omniDriveFactor;
        //calculate Y displacement, and convert ticks to inches (3.543 = wheel diameter inches, 560 = ticks per wheel rot), and account for robot angle
        robotYdisplacement = ((rfDispNoRot - lfDispNoRot + rrDispNoRot - lrDispNoRot) / 4) * ((3.543 * Math.PI) / 540) / omniDriveFactor;

        robotVectorByOdo = Math.atan2(robotYdisplacement, robotXdisplacement) * 180 / Math.PI;

        robotVectorMag = Math.sqrt((robotXdisplacement * robotXdisplacement) + (robotYdisplacement * robotYdisplacement));

        robotFieldAngle = (robotVectorByOdo + getAngle());

        robotFieldPositionX = robotVectorMag * Math.cos(robotFieldAngle * Math.PI / 180);  //field position in inches
        robotFieldPositionY = robotVectorMag * Math.sin(robotFieldAngle * Math.PI / 180);  //field position in inches

//        robotFieldPositionX = robotFieldPositionXTicks * 0.02060; //convert ticks to inches
//        robotFieldPositionY = robotFieldPositionYTicks * 0.02060; //convert ticks to inches

        double [] displacement = {robotFieldPositionX, robotFieldPositionY};
        return displacement;

    }

}
