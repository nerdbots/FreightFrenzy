package teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.BlockDetector;

import java.util.ArrayList;

import treamcode.CurvePoint;

//import Functions.CurvePoint;
@Disabled
@Autonomous(name="visionPurePursuit", group="Auton")

public class visionPurePursuit extends LinearOpMode {

    private PurePursuitRobotMovement6_Quad2 myPurePursuitRobotMovement6_Quad2;
//    private wobble_Pickup mywobble_Pickup;

    boolean debugFlag = true;
    int robotPath = 4;

    boolean freightDetected = false;
    double freightDistance;
    double freightAngle;
    double freightAnglePark;

    double [] robotLocationXY;
    double robotLocationX;
    double robotLocationY;

    double [] robotTargetPoint;
    double robotTargetPointX;
    double robotTargetPointY;

    double robotLookAheadX;
    double robotLookAheadY;


    BlockDetector blockDetector;
    Recognition recognition;

    FtcDashboard dashboard;


    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        //Create a NerdBOT object
        myPurePursuitRobotMovement6_Quad2 = new PurePursuitRobotMovement6_Quad2(this);
//        mywobble_Pickup = new wobble_Pickup (this);

        myPurePursuitRobotMovement6_Quad2.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6_Quad2.initializeHardware();
//        mywobble_Pickup.wobbleInit();
        //Initialize the PID Calculators

//        telemetry.addData("Init", "Completed");
//        telemetry.update();


        blockDetector = new BlockDetector(this);
        blockDetector.initVuforia();
        blockDetector.initTfod();

        telemetry.addData("Okay press play", "now");
        telemetry.update();

        waitForStart();

        myPurePursuitRobotMovement6_Quad2.printI();

        myPurePursuitRobotMovement6_Quad2.resetITerm();

        myPurePursuitRobotMovement6_Quad2.printI();

        myPurePursuitRobotMovement6_Quad2.resetTimers();


        recognition = blockDetector.detectCube();

        freightDistance = blockDetector.getDistance(recognition);
        freightAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);


        if(recognition != null){
            telemetry.addData("Distance", blockDetector.getDistance(recognition));
            telemetry.addData("Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
        }

        telemetry.update();
        sleep (5000);



//        myPurePursuitRobotMovement6.goToPosition(0, 100, 1.0, 90, 0.3, 30);
//        myPurePursuitRobotMovement5.goToPosition(0,0, 1.0, 270, 0.2);

        robotPath = 4;

        freightDetected = true;

        if (recognition != null) {
//            freightDistance = 30;
//            freightAngle = 45;

            robotTargetPoint = myPurePursuitRobotMovement6_Quad2.calculateTargetPoint(freightDistance, freightAngle);

            robotLocationX = robotTargetPoint[0];
            robotLocationY = robotTargetPoint[1];
            robotTargetPointX = robotTargetPoint[2];
            robotTargetPointY = robotTargetPoint[3];
            robotLookAheadX = robotTargetPoint[4];
            robotLookAheadY = robotTargetPoint[5];
            freightAnglePark = robotTargetPoint[6];

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(robotLocationX, robotLocationY, 0.6, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(robotTargetPointX, robotTargetPointY, 0.6, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(robotLookAheadX, robotLookAheadY, 0.6, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 15, freightAnglePark, 2);

        }

//        if (freightDetected = true) {
//            freightDistance = 40;
//            freightAngle = 0;
//
//            robotTargetPoint = myPurePursuitRobotMovement6_Quad2.calculateTargetPoint(freightDistance, freightAngle);
//
//            robotLocationX = robotTargetPoint[0];
//            robotLocationY = robotTargetPoint[1];
//            robotTargetPointX = robotTargetPoint[2];
//            robotTargetPointY = robotTargetPoint[3];
//            robotLookAheadX = robotTargetPoint[4];
//            robotLookAheadY = robotTargetPoint[5];
//            freightAnglePark = robotTargetPoint[6];
//
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(robotLocationX, robotLocationY, 1, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(robotTargetPointX, robotTargetPointY, 1, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(robotLookAheadX, robotLookAheadY, 1, 0.3, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 26, freightAnglePark, 2);
//
//        }
//
//        if (freightDetected = true) {
//            freightDistance = 60;
//            freightAngle = -45;
//
//            robotTargetPoint = myPurePursuitRobotMovement6_Quad2.calculateTargetPoint(freightDistance, freightAngle);
//
//            robotLocationX = robotTargetPoint[0];
//            robotLocationY = robotTargetPoint[1];
//            robotTargetPointX = robotTargetPoint[2];
//            robotTargetPointY = robotTargetPoint[3];
//            robotLookAheadX = robotTargetPoint[4];
//            robotLookAheadY = robotTargetPoint[5];
//            freightAnglePark = robotTargetPoint[6];
//
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(robotLocationX, robotLocationY, 1, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(robotTargetPointX, robotTargetPointY, 1, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(robotLookAheadX, robotLookAheadY, 1, 0.3, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 26, freightAnglePark, 2);
//
//        }
//
//        if (freightDetected = true) {
//            freightDistance = 90;
//            freightAngle = 200;
//
//            robotTargetPoint = myPurePursuitRobotMovement6_Quad2.calculateTargetPoint(freightDistance, freightAngle);
//
//            robotLocationX = robotTargetPoint[0];
//            robotLocationY = robotTargetPoint[1];
//            robotTargetPointX = robotTargetPoint[2];
//            robotTargetPointY = robotTargetPoint[3];
//            robotLookAheadX = robotTargetPoint[4];
//            robotLookAheadY = robotTargetPoint[5];
//            freightAnglePark = robotTargetPoint[6];
//
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(robotLocationX, robotLocationY, 1, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(robotTargetPointX, robotTargetPointY, 1, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(robotLookAheadX, robotLookAheadY, 1, 0.3, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 26, freightAnglePark, 2);
//
//        }


//        if (robotPath == 4) {
////
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(0, 0, 1, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 80, 1, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(60, 80, 1, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(60, 20, 1, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(60, -40, 1, 0.4, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 30, 90, 2);
//
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(0, 0, 0.5, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 60, 0.5, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 100, 0.5, 0.3, 25, 180, 0.3));
//
//
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 30, 90, 2);
//
//
//        }else if (robotPath == 1) {
//
//
//            //Second Path to the Square 1
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(0, 0, 1.0, 0.3, 35, 0, 0.3));
//            allPoints.add(new CurvePoint(-15, 40, 1.0, 0.3, 35, 180, 0.3));
//            allPoints.add(new CurvePoint(6, 88, 1.0, 0.3, 35, 180, 0.3));
//            allPoints.add(new CurvePoint(27, 148, 1.0, 0.3, 35, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 35, 90, 3);
//
//            //        sleep(1000);
//            //        mywobble_Pickup.beginningDown();
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(6, 88, 1.0, 0.4, 35, 0, 0.3));
//            allPoints.add(new CurvePoint(18, 60, 1.0, 0.4, 35, 180, 0.3));
//            allPoints.add(new CurvePoint(21, 28, 1.0, 0.4, 35, 180, 0.3));
//            allPoints.add(new CurvePoint(21, -44, 1.0, 0.3, 35, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 35, 225, 1);
//
//            //        sleep(1000);
//            //        mywobble_Pickup.pickupWobble();
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(21, 28, 1.0, 0.4, 35, 0, 0.3));
//            allPoints.add(new CurvePoint(18, 60, 1.0, 0.4, 35, 180, 0.3));
//            allPoints.add(new CurvePoint(12, 80, 1.0, 0.4, 35, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 140, 1.0, 0.3, 35, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 30, 90, 3);
//
//            //        sleep(1000);
//            //        mywobble_Pickup.setDownWobble();
//
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(12, 86, 0.6, 0.4, 40, 0, 0.3));
//            allPoints.add(new CurvePoint(5, 60, 0.6, 0.4, 40, 180, 0.3));
//            allPoints.add(new CurvePoint(4, 10, 0.6, 0.4, 40, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 3);
//
//            //        sleep(1000);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(5, 60, 0.6, 0.4, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(3, 48, 0.6, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(1, 0, 0.6, 0.4, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 3);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(3, 48, 0.6, 0.4, 40, 0, 0.3));
//            allPoints.add(new CurvePoint(5, 60, 0.6, 0.4, 40, 180, 0.3));
//            allPoints.add(new CurvePoint(5, 110, 0.6, 0.4, 40, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 3);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(5, 60, 0.6, 0.4, 40, 0, 0.3));
//            allPoints.add(new CurvePoint(5, 70, 0.6, 0.4, 40, 180, 0.3));
//            allPoints.add(new CurvePoint(5, 130, 0.6, 0.4, 40, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 7, 90, 4);
//
//        }else {
//
//            //Third Path to the Square 0
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(0, 0, 1.0, 0.3, 35, 0, 0.3));
//            allPoints.add(new CurvePoint(-18, 66, 1.0, 0.3, 35, 180, 0.3));
//            allPoints.add(new CurvePoint(-24, 140, 1.0, 0.3, 35, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 35, 90, 3);
//
////        sleep(1000);
////        mywobble_Pickup.beginningDown();
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-18, 66, 1.0, 0.4, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(18, 50, 1.0, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(23, 28, 1.0, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(23, -44, 1.0, 0.3, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 30, 225, 1);
//
////        sleep(1000);
////        mywobble_Pickup.pickupWobble();
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(22, 28, 1.0, 0.4, 35, 0, 0.3));
//            allPoints.add(new CurvePoint(-12, 65, 1.0, 0.4, 35, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 140, 1.0, 0.3, 35, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 35, 90, 3);
//
////        sleep(1000);
////        mywobble_Pickup.setDownWobble();
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-12, 65, 0.6, 0.4, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(-12, 50, 0.6, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(-12, 0, 0.6, 0.4, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 4);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-12, 50, 0.6, 0.4, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(6, 60, 0.6, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(24, 100, 0.6, 0.4, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 2);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(6, 60, 0.6, 0.4, 35, 0, 0.3));
//            allPoints.add(new CurvePoint(6, 70, 0.6, 0.4, 35, 180, 0.3));
//            allPoints.add(new CurvePoint(6, 130, 0.6, 0.4, 35, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 7, 90, 4);
//
//        }

    }

}


