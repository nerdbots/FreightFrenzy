package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ArmShoulderPositions;
import org.firstinspires.ftc.teamcode.DuckDetector;
import org.firstinspires.ftc.teamcode.FingerPositions;

import java.util.ArrayList;

import treamcode.CurvePoint;


@Autonomous(name="RED_DuckSide_1Block_ParkWH", group="Linear Opmode")

public class Auton_RED_DuckSide_OneBlockParkWH_V2 extends LinearOpMode {

    private PurePursuitRobotMovement6_Turn_MultiThread_V2 myPurePursuitRobotMovement6_Turn_MultiThread;

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
        myPurePursuitRobotMovement6_Turn_MultiThread = new PurePursuitRobotMovement6_Turn_MultiThread_V2(this);
        myPurePursuitRobotMovement6_Turn_MultiThread.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6_Turn_MultiThread.initializeHardware();

        duckDetector = new DuckDetector(this);
        duckDetector.initDuckDetector();

        timeCheckCount = 0;

        telemetry.addData("NerdBOT", "Initialized");
        telemetry.update();

        waitForStart();

        OpmodeTimer.reset();

        myPurePursuitRobotMovement6_Turn_MultiThread.startOdometryThread();

        duckPosition = duckDetector.getAnalysis();
        telemetry.addData("Analysis",duckDetector.getAnalysis());
        telemetry.update();

        duckDetector.closeCameraDevice();


        myPurePursuitRobotMovement6_Turn_MultiThread.printI();

        myPurePursuitRobotMovement6_Turn_MultiThread.resetITerm();

        myPurePursuitRobotMovement6_Turn_MultiThread.printI();

        myPurePursuitRobotMovement6_Turn_MultiThread.resetTimers();


        if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.CENTER)) {
            shoulderPosition = ArmShoulderPositions.LEVEL2;
            armDelay=0.7;
//            shippingHubPark = 25;
            shpX = 8;
            shpY = 23;
            shpX2 = 34;
            shpY2 = 24;
        }
        else if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.LEFT)){
            shoulderPosition = ArmShoulderPositions.LEVEL1;
            armDelay=0.7;
//            shippingHubPark = 29;
            shpX = 8;
            shpY = 26;
            shpX2 = 34;
            shpY2 = 29;
        }
        else if(duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.RIGHT)){
            shoulderPosition = ArmShoulderPositions.LEVEL3;
            armDelay = 0.0;
//            shippingHubPark = 30;
            shpX = 8;
            shpY = 28;
            shpX2 = 34;
            shpY2 = 27;
        }

        if (purePursuitPath == 1){

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.4, 0.3, 25, 0, 90));
            allPoints.add(new CurvePoint(0, 20, 0.4, 0.3, 25, 180, 225));
            allPoints.add(new CurvePoint(shpX, shpY, 0.4, 0.3, 25, 180, 225));
            allPoints.add(new CurvePoint(10, 80, 0.4, 0.3, 25, 180, 225));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 10, 225, 2,
                    ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.0,1,"none", 0);

            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.ENTER_INTAKE);

            sleep(300);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(shpX, shpY, 0.6, 0.4, 25, 0, 225));
            allPoints.add(new CurvePoint(-17, 7, 0.6, 0.4, 25, 180, 160));
            allPoints.add(new CurvePoint(-28, 7, 0.6, 0.4, 25, 180, 160));
            allPoints.add(new CurvePoint(-80, 7, 0.6, 0.4, 25, 180, 160));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 160, 2,
                    shoulderPosition,ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.ENTER_INTAKE,0.7, 0,"none", 0);

            myPurePursuitRobotMovement6_Turn_MultiThread.runMotor("duckyDisc",0.35,3.75);

            sleep(12000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-28, 7, 0.8, 0.3, 25, 0, 160));
            allPoints.add(new CurvePoint(0, 0, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(10, -2, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(18, -2, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(25, -2, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(35, -2, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(45, -5, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(55, -5, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(65, -5, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(79, -5, 0.8, 0.3, 25, 180, 0));
//            allPoints.add(new CurvePoint(89, -2, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(110, -5, 0.8, 0.3, 25, 180, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 0, 6,
                    ArmShoulderPositions.INTAKE, ArmShoulderPositions.INTAKE, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "none",0);

//            sleep(500);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(30, -2, 0.3, 0.3, 20, 0, 0));
//            allPoints.add(new CurvePoint(35, 20, 0.3, 0.3, 20, 0, -40));
//            allPoints.add(new CurvePoint(shpX2, shpY2, 0.3, 0.3, 20, 0, -40));
//            allPoints.add(new CurvePoint(35, 65, 0.3, 0.3, 20, 180, -40));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 10, -40, 2,
//                    ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.5,0,"intake", -0.85);
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.INTAKE_READY);
//
//            sleep(300);
//
//            boolean didWeMissSecondBlock = false;
//            ArmShoulderPositions nextShoulderPosition = ArmShoulderPositions.LEVEL3;
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(shpX2, shpY2, 0.6, 0.4, 25, 0, -40));
//            allPoints.add(new CurvePoint(32, 10, 0.6, 0.4, 25, 0, 0));
//            allPoints.add(new CurvePoint(32, -16, 0.6, 0.4, 25, 180, 0));
//            allPoints.add(new CurvePoint(55, -9, 0.6, 0.4, 25, 180, 0));
//            allPoints.add(new CurvePoint(85, -2, 0.6, 0.4, 25, 180, 0));
//            allPoints.add(new CurvePoint(115, -2, 0.6, 0.4, 25, 180, 0));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 0, 7,
//                    shoulderPosition, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY,0.7, 0,"intake", -0.85);
//
//            if(myPurePursuitRobotMovement6_Turn_MultiThread.checkIfBlockIsIn()){
//
//                nextShoulderPosition = shoulderPosition;
//            }
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.AutonBlockIntake();
//
//            sleep(500);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(85, 15, 0.6, 0.4, 25, 0, 0));
//            allPoints.add(new CurvePoint(85, -5, 0.6, 0.4, 25, 0, 0));
//            allPoints.add(new CurvePoint(85, -50, 0.6, 0.4, 25, 180, 0));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 20, 0, 6,
//                    ArmShoulderPositions.INTAKE, nextShoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.7, 0,"none", 0);
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
//                allPoints.add(new CurvePoint(85, -7, 0.8, 0.3, 25, 180, 0));
//                allPoints.add(new CurvePoint(72, -7, 0.8, 0.3, 25, 180, 0));
//                allPoints.add(new CurvePoint(52, -7, 0.8, 0.3, 25, 180, 0));
//                allPoints.add(new CurvePoint(42, -7, 0.8, 0.3, 25, 180, 0));
//                allPoints.add(new CurvePoint(32, 15, 0.8, 0.3, 25, 180, -40));
//                allPoints.add(new CurvePoint(34, 25, 0.8, 0.3, 25, 180, -40));
//                allPoints.add(new CurvePoint(34, 78, 0.8, 0.3, 25, 180, -40));
//
//                myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, -40, 2,
//                        ArmShoulderPositions.INTAKE, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB,0, 0,"intake", 0.5);
//
//                myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.INTAKE_READY);
//
//                sleep(300);
//
//                allPoints = new ArrayList<>();
//                allPoints.add(new CurvePoint(shpX2, shpY2, 0.8, 0.4, 25, 180, -40));
//                allPoints.add(new CurvePoint(32, 0, 0.8, 0.4, 25, 180, 0));
//                allPoints.add(new CurvePoint(37, -9, 0.8, 0.4, 25, 180, 0));
//                allPoints.add(new CurvePoint(47, -9, 0.8, 0.4, 25, 180, 0));
//                allPoints.add(new CurvePoint(57, -9, 0.8, 0.4, 25, 180, 0));
//                allPoints.add(new CurvePoint(79, -2, 0.8, 0.4, 25, 180, 0));
//                allPoints.add(new CurvePoint(115, -2, 0.8, 0.4, 25, 180, 0));
//
//                myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 0, 5,
//                        ArmShoulderPositions.LEVEL3, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY,0.7, 0,"none", 0);
//
//
//            }else{
//                allPoints = new ArrayList<>();
//                allPoints.add(new CurvePoint(75, -2, 0.6, 0.4, 25, 180, 0));
//                allPoints.add(new CurvePoint(81, 3, 0.6, 0.4, 25, 180, 0));
//                allPoints.add(new CurvePoint(115, 3, 0.6, 0.4, 25, 180, 0));
//
//                myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 5, 0, 5,
//                        ArmShoulderPositions.INTAKE, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY,0.7, 0,"none", 0);
//
//
//            }


        }

        myPurePursuitRobotMovement6_Turn_MultiThread.stopOdometryThread();

    }

}

