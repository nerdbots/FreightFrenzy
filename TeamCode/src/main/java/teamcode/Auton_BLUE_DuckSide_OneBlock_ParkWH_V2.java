package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ArmShoulderPositions;
import org.firstinspires.ftc.teamcode.DuckDetector;
import org.firstinspires.ftc.teamcode.FingerPositions;

import java.util.ArrayList;

import treamcode.CurvePoint;


@Autonomous(name="BLUE_DuckSide_1Block_ParkWH", group="Linear Opmode")

public class Auton_BLUE_DuckSide_OneBlock_ParkWH_V2 extends LinearOpMode {

    private PurePursuitRobotMovement6_Turn_MultiThread_V2 myPurePursuitRobotMovement6_Turn_MultiThread_V2;

    boolean debugFlag = true;

    double armDelay = 0.0;
    double shippingHubPark = 0;
    double shpX = 0;
    double shpY = 0;
    double shpX2 = 0;
    double shpY2 = 0;

    int purePursuitPath = 1;
    DuckDetector.DuckDeterminationPipeline.DuckPosition duckPosition;
    DuckDetector duckDetector;

    public  volatile ArmShoulderPositions shoulderPosition = ArmShoulderPositions.INTAKE;
    public  volatile FingerPositions fingerPosition = FingerPositions.ENTER_INTAKE;

    ElapsedTime OpmodeTimer = new ElapsedTime();
    double timeCheck = 0;
    int timeCheckCount = 0;


    @Override
    public void runOpMode() {

        //Create a NerdBOT object
        myPurePursuitRobotMovement6_Turn_MultiThread_V2 = new PurePursuitRobotMovement6_Turn_MultiThread_V2(this);
        myPurePursuitRobotMovement6_Turn_MultiThread_V2.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6_Turn_MultiThread_V2.initializeHardware();

        duckDetector = new DuckDetector(this);
        duckDetector.initDuckDetector();

        timeCheckCount = 0;

        telemetry.addData("NerdBOT", "Initialized");
        telemetry.update();

        waitForStart();

        OpmodeTimer.reset();

        myPurePursuitRobotMovement6_Turn_MultiThread_V2.startOdometryThread();

        duckPosition = duckDetector.getAnalysis();
        telemetry.addData("Analysis",duckDetector.getAnalysis());
        telemetry.update();

        duckDetector.closeCameraDevice();


        myPurePursuitRobotMovement6_Turn_MultiThread_V2.printI();

        myPurePursuitRobotMovement6_Turn_MultiThread_V2.resetITerm();

        myPurePursuitRobotMovement6_Turn_MultiThread_V2.printI();

        myPurePursuitRobotMovement6_Turn_MultiThread_V2.resetTimers();


        if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.CENTER)) {
            shoulderPosition = ArmShoulderPositions.LEVEL2;
            armDelay=0.5;
//            shippingHubPark = 25;
            shpX = -6;
            shpY = 28;
            shpX2 = -9;
            shpY2 = 24;
        }
        else if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.LEFT)){
            shoulderPosition = ArmShoulderPositions.LEVEL1;
            armDelay=0.5;
//            shippingHubPark = 29;
            shpX = -7;
            shpY = 31;
            shpX2 = -15;
            shpY2 = 25;
        }
        else if(duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.RIGHT)){
            shoulderPosition = ArmShoulderPositions.LEVEL3;
            armDelay = 0.0;
//            shippingHubPark = 30;
            shpX = -10;
            shpY = 33;
            shpX2 = -11;
            shpY2 = 27;
        }

        if (purePursuitPath == 1){

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.4, 0.3, 20, 0, 90));
            allPoints.add(new CurvePoint(-5, 15, 0.4, 0.3, 10, 180, -30));
            allPoints.add(new CurvePoint(-5, 20, 0.4, 0.3, 20, 180, -30));
            allPoints.add(new CurvePoint(shpX, shpY, 0.4, 0.3, 25, 180, -30));
            allPoints.add(new CurvePoint(-10, 80, 0.4, 0.3, 25, 180, -30));

            myPurePursuitRobotMovement6_Turn_MultiThread_V2.followCurveArm_V2(allPoints, 0, 10, -30, 2,
                    ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.0,0,"none", 0);


            myPurePursuitRobotMovement6_Turn_MultiThread_V2.setFingerPositions(FingerPositions.ENTER_INTAKE);

            sleep(300);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(shpX, shpY, 0.6, 0.4, 20, 0, 0));
            allPoints.add(new CurvePoint(5, 5, 0.6, 0.4, 20, 0, -150));
            allPoints.add(new CurvePoint(20, 9, 0.6, 0.4, 20, 180, -150));
            allPoints.add(new CurvePoint(60, 9, 0.6, 0.4, 20, 180, -150));

            myPurePursuitRobotMovement6_Turn_MultiThread_V2.followCurveArm_V2(allPoints, 0, 15, -150, 2,
                    shoulderPosition,ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.ENTER_INTAKE,0.7, 0,"none", 0);


            myPurePursuitRobotMovement6_Turn_MultiThread_V2.runMotor("duckyDisc",-0.35,3.75);

            sleep(12000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(22, 10, 0.6, 0.3, 25, 0, -160));
//            allPoints.add(new CurvePoint(10, 20, 0.6, 0.3, 25, 180, -180));
            allPoints.add(new CurvePoint(-10, 20, 0.6, 0.3, 25, 180, -180));
            allPoints.add(new CurvePoint(-10, -15, 0.6, 0.3, 25, 180, -180));
            allPoints.add(new CurvePoint(-18, -15, 0.6, 0.3, 25, 180, -180));
            allPoints.add(new CurvePoint(-36, -6, 0.6, 0.3, 25, 180, -180));
            allPoints.add(new CurvePoint(-55, -6, 0.6, 0.3, 25, 180, -180));
            allPoints.add(new CurvePoint(-81, -6, 0.6, 0.3, 25, 180, -180));
            allPoints.add(new CurvePoint(-110, -6, 0.6, 0.3, 25, 180, -180));

            myPurePursuitRobotMovement6_Turn_MultiThread_V2.followCurveArm_V2(allPoints, 0, 27, -180, 10,
                    ArmShoulderPositions.INTAKE, ArmShoulderPositions.INTAKE, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "none",0);

//            sleep(500);
//
//            if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.RIGHT)){
//
//                allPoints = new ArrayList<>();
//                allPoints.add(new CurvePoint(-36, -2, 0.4, 0.3, 20, 0, -180));
//                allPoints.add(new CurvePoint(-39, 5, 0.4, 0.3, 20, 0, -130));
//                allPoints.add(new CurvePoint(-39, shpY2, 0.4, 0.3, 20, 0, -130));
//                allPoints.add(new CurvePoint(-39, 60, 0.4, 0.3, 20, 0, -130));
//
//                myPurePursuitRobotMovement6_Turn_MultiThread_V2.followCurveArm_V2(allPoints, 0, 10, -130, 2,
//                        ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.5,0,"intake", -0.85);
//
//                myPurePursuitRobotMovement6_Turn_MultiThread_V2.setFingerPositions(FingerPositions.INTAKE_READY);
//
//            }else {
//
//                allPoints = new ArrayList<>();
//                allPoints.add(new CurvePoint(-36, -2, 0.30, 0.3, 20, 0, -180));
//                allPoints.add(new CurvePoint(-39, 5, 0.30, 0.3, 20, 0, -130));
//                allPoints.add(new CurvePoint(-39, 13, 0.30, 0.3, 20, 0, -130));
//                allPoints.add(new CurvePoint(-39, 60, 0.30, 0.3, 20, 0, -130));
////            allPoints.add(new CurvePoint(-36, -2, 0.5, 0.3, 20, 0, -180));
//////            allPoints.add(new CurvePoint(-20, 6, 0.6, 0.3, 20, 0, -140));
//////            allPoints.add(new CurvePoint(-15, 13, 0.6, 0.3, 20, 0, -100));
////            allPoints.add(new CurvePoint(-10, 5, 0.5, 0.3, 20, 0, -40));
////            allPoints.add(new CurvePoint(shpX2, shpY2, 0.5, 0.3, 20, 0, -40));
//////            allPoints.add(new CurvePoint(-7, 30, 0.6, 0.3, 25, 180, -20));
////            allPoints.add(new CurvePoint(-10, 65, 0.5, 0.3, 20, 180, -40));
//
//                myPurePursuitRobotMovement6_Turn_MultiThread_V2.followCurveArm_V2(allPoints, 0, 10, -130, 3,
//                        ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB, 0.5, 0, "intake", -0.85);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-39, 13, 0.30, 0.3, 20, 0, -130));
//            allPoints.add(new CurvePoint(-39, shpY2, 0.30, 0.3, 20, 0, -130));
//            allPoints.add(new CurvePoint(-39, 60, 0.30, 0.3, 20, 0, -130));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread_V2.followCurveArm_V2(allPoints, 0, 5, -130, 3,
//                    shoulderPosition, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.0,0,"intake", -0.85);
//
//
////                myPurePursuitRobotMovement6_Turn_MultiThread_V2.turnRobot(-130);
//
//                myPurePursuitRobotMovement6_Turn_MultiThread_V2.setFingerPositions(FingerPositions.INTAKE_READY);
//            }
//            sleep(300);
//
//            boolean didWeMissSecondBlock = false;
//            ArmShoulderPositions nextShoulderPosition = ArmShoulderPositions.LEVEL3;
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-36, shpY2, 0.6, 0.4, 25, 0, -120));
//            allPoints.add(new CurvePoint(-36, 8, 0.6, 0.4, 25, 0, -180));
//            allPoints.add(new CurvePoint(-31, -10, 0.6, 0.4, 25, 180, -180));
//            allPoints.add(new CurvePoint(-55, -10, 0.6, 0.4, 25, 180, -180));
//            allPoints.add(new CurvePoint(-89, 0, 0.6, 0.4, 25, 180, -180));
//            allPoints.add(new CurvePoint(-115, 0, 0.6, 0.4, 25, 180, -180));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread_V2.followCurveArm_V2(allPoints, 0, 15, -180, 7,
//                    shoulderPosition, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY,0.7, 0,"intake", -0.85);
//
//            if(myPurePursuitRobotMovement6_Turn_MultiThread_V2.checkIfBlockIsIn()){
//
//                nextShoulderPosition = shoulderPosition;
//            }
//
//            myPurePursuitRobotMovement6_Turn_MultiThread_V2.AutonBlockIntake();
//
//            sleep(500);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-85, 15, 0.6, 0.4, 25, 0, -180));
//            allPoints.add(new CurvePoint(-85, 2, 0.6, 0.4, 25, 0, -180));
//            allPoints.add(new CurvePoint(-85, -50, 0.6, 0.4, 25, 180, -180));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread_V2.followCurveArm_V2(allPoints, 0, 20, -180, 6,
//                    ArmShoulderPositions.INTAKE, ArmShoulderPositions.INTAKE, FingerPositions.GRAB, FingerPositions.GRAB,0.7, 0,"none", 0);
//
//
//            if (timeCheckCount < 1){
//                timeCheck = OpmodeTimer.seconds();
//                timeCheckCount += 1;
//            }
//
//
//            if(timeCheck < 25){
//                allPoints = new ArrayList<>();
//                allPoints.add(new CurvePoint(-89, -2, 0.8, 0.3, 25, 180, -180));
//                allPoints.add(new CurvePoint(-72, -6, 0.8, 0.3, 25, 180, -180));
//                allPoints.add(new CurvePoint(-52, -6, 0.8, 0.3, 25, 180, -180));
//                allPoints.add(new CurvePoint(-42, 0, 0.8, 0.3, 25, 180, -135));
//                allPoints.add(new CurvePoint(-40, 15, 0.8, 0.3, 25, 180, -135));
//                allPoints.add(new CurvePoint(-40, 27, 0.8, 0.3, 25, 180, -135));
//                allPoints.add(new CurvePoint(-40, 78, 0.8, 0.3, 25, 180, -135));
//
////                myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, -135, 3,
////                        ArmShoulderPositions.INTAKE, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB,0.5, 0,"intake", 0.5);
//
//                myPurePursuitRobotMovement6_Turn_MultiThread_V2.followCurveArm_V2(allPoints, 0, 20, -135, 2,
//                        ArmShoulderPositions.INTAKE, nextShoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.5, 0,"intake", 0.5);
//
//                myPurePursuitRobotMovement6_Turn_MultiThread_V2.setFingerPositions(FingerPositions.INTAKE_READY);
//
//                sleep(300);
//
//                allPoints = new ArrayList<>();
//                allPoints.add(new CurvePoint(-39, 21, 0.8, 0.4, 25, 180, -115));
//                allPoints.add(new CurvePoint(-37, 10, 0.8, 0.4, 25, 180, -180));
//                allPoints.add(new CurvePoint(-37, 0, 0.8, 0.4, 25, 180, -180));
//                allPoints.add(new CurvePoint(-37, -7, 0.8, 0.4, 25, 180, -180));
//                allPoints.add(new CurvePoint(-47, -5, 0.8, 0.4, 25, 180, -180));
//                allPoints.add(new CurvePoint(-57, -4, 0.8, 0.4, 25, 180, -180));
//                allPoints.add(new CurvePoint(-82, -2, 0.8, 0.4, 25, 180, -180));
//                allPoints.add(new CurvePoint(-115, -2, 0.8, 0.4, 25, 180, -180));
//
//                myPurePursuitRobotMovement6_Turn_MultiThread_V2.followCurveArm_V2(allPoints, 0, 15, -180, 8,
//                        ArmShoulderPositions.LEVEL3, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY,0.7, 0,"none", 0);
//
//
//            }else{
//                allPoints = new ArrayList<>();
//                allPoints.add(new CurvePoint(-70, -2, 0.6, 0.4, 25, 180, -180));
//                allPoints.add(new CurvePoint(-82, 6, 0.6, 0.4, 25, 180, -180));
//                allPoints.add(new CurvePoint(-115, 6, 0.6, 0.4, 25, 180, -180));
//
//                myPurePursuitRobotMovement6_Turn_MultiThread_V2.followCurveArm_V2(allPoints, 0, 5, -180, 5,
//                        ArmShoulderPositions.INTAKE, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY,0.7, 0,"none", 0);
//
//
//            }


        }

        myPurePursuitRobotMovement6_Turn_MultiThread_V2.stopOdometryThread();

    }

}

