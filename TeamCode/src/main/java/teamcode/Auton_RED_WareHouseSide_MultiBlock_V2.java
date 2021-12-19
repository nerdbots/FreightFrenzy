package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ArmShoulderPositions;
import org.firstinspires.ftc.teamcode.DuckDetector;
import org.firstinspires.ftc.teamcode.FingerPositions;

import java.util.ArrayList;

import treamcode.CurvePoint;


@Autonomous(name="RED_WarehouseSide_3Block", group="Linear Opmode")

public class Auton_RED_WareHouseSide_MultiBlock_V2 extends LinearOpMode {

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
            armDelay=0.5;
            shippingHubPark = 28;
            shpX = -9;
            shpY = 28;
            shpX2 = -8;
            shpY2 = 30;
        }
        else if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.LEFT)){
            shoulderPosition = ArmShoulderPositions.LEVEL1;
            armDelay=0.5;
            shippingHubPark = 31;
            shpX = -9;
            shpY = 31;
            shpX2 = -8;
            shpY2 = 32;
        }
        else if(duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.RIGHT)){
            shoulderPosition = ArmShoulderPositions.LEVEL3;
            armDelay = 0.0;
            shippingHubPark = 33;
            shpX = -12;
            shpY = 30;
            shpX2 = -9;
            shpY2 = 33;
        }

        if (purePursuitPath == 1){

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.4, 0.3, 25, 180, 90));
            allPoints.add(new CurvePoint(-12, 26, 0.4, 0.3, 25, 180, -30));
            allPoints.add(new CurvePoint(shpX, shpY, 0.4, 0.3, 25, 180, -30));
            allPoints.add(new CurvePoint(-10, 80, 0.4, 0.3, 25, 180, -30));


            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 10, -30, 2,
                    ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.0,0,"none", 0);

//            myPurePursuitRobotMovement6_Turn_MultiThread.turnRobot(235);

            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.INTAKE_READY);

            sleep(300);
//            myPurePursuitRobotMovement6_Turn_MultiThread.turnRobot(270);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(shpX, shpY, 0.8, 0.3, 20, 180, -30));
            allPoints.add(new CurvePoint(-13, 8, 0.6, 0.3, 20, 180, 0));
            allPoints.add(new CurvePoint(-13, -4, 0.6, 0.3, 20, 180, 0));
            allPoints.add(new CurvePoint(-8, -12, 0.6, 0.3, 20, 180, 0));
            allPoints.add(new CurvePoint(0, -8, 0.6, 0.3, 20, 180, 0));
            allPoints.add(new CurvePoint(10, -2, 0.6, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(34, 0, 0.6, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(74, 0, 0.6, 0.3, 25, 180, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 0, 5,
                    shoulderPosition,ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY,0.7, 0,"none", 0);

            myPurePursuitRobotMovement6_Turn_MultiThread.AutonBlockIntake();

            sleep(1000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(28, 15, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(28, 0, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(28, -50, 0.8, 0.3, 25, 180, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 20, 0, 6,
                    ArmShoulderPositions.INTAKE,ArmShoulderPositions.INTAKE, FingerPositions.GRAB, FingerPositions.GRAB,0.7, 0,"none", 0);


            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(34, 0, 0.4, 0.3, 15, 180, 0));
            allPoints.add(new CurvePoint(15, -6, 0.6, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(0, -6, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(-12, -2, 0.8, 0.3, 25, 180, -30));
            allPoints.add(new CurvePoint(-12, 10, 0.8, 0.3, 25, 180, -30));
            allPoints.add(new CurvePoint(shpX2, shpY2, 0.8, 0.3, 25, 180, -30));
            allPoints.add(new CurvePoint(-10, 80, 0.8, 0.3, 25, 180, -30));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, -30, 2,
                    ArmShoulderPositions.INTAKE, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB,0.7, 0,"intake", 0.5);

            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.INTAKE_READY);

            sleep(300);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(shpX2, shpY2, 0.8, 0.3, 20, 180, -30));
            allPoints.add(new CurvePoint(-13, 8, 0.6, 0.3, 20, 180, 0));
            allPoints.add(new CurvePoint(-13, -4, 0.6, 0.3, 20, 180, 0));
            allPoints.add(new CurvePoint(-13, -12, 0.6, 0.3, 20, 180, 0));
            allPoints.add(new CurvePoint(0, -8, 0.6, 0.3, 20, 180, 0));
            allPoints.add(new CurvePoint(10, -2, 0.6, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(34, 0, 0.6, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(50, 0, 0.6, 0.3, 25, 180, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 0, 5,
                    ArmShoulderPositions.LEVEL3,ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY,0.7, 0,"none", 0);

            myPurePursuitRobotMovement6_Turn_MultiThread.AutonBlockIntake();

            sleep(1000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(28, 15, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(28, 0, 0.8, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(28, -50, 0.8, 0.3, 25, 180, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 20, 0, 6,
                    ArmShoulderPositions.INTAKE,ArmShoulderPositions.INTAKE, FingerPositions.GRAB, FingerPositions.GRAB,0.7, 0,"none", 0);

            if (timeCheckCount < 1){
                timeCheck = OpmodeTimer.seconds();
                timeCheckCount += 1;
            }

            if (timeCheck < 25) {

                allPoints = new ArrayList<>();
                allPoints.add(new CurvePoint(34, 0, 0.4, 0.3, 25, 180, 0));
                allPoints.add(new CurvePoint(10, -6, 0.6, 0.3, 25, 180, 0));
                allPoints.add(new CurvePoint(0, -6, 0.8, 0.3, 25, 180, 0));
                allPoints.add(new CurvePoint(-12, -2, 0.8, 0.3, 25, 180, -30));
                allPoints.add(new CurvePoint(-12, 10, 0.8, 0.3, 25, 180, -30));
                allPoints.add(new CurvePoint(shpX2, shpY2, 0.8, 0.3, 25, 180, -30));
                allPoints.add(new CurvePoint(-10, 80, 0.8, 0.3, 25, 180, -30));

                myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, -30, 2,
                        ArmShoulderPositions.INTAKE, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB, 0.7, 0, "intake", 0.5);

                myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.INTAKE_READY);

                sleep(300);

                allPoints = new ArrayList<>();
                allPoints.add(new CurvePoint(shpX2, shpY2, 0.8, 0.3, 20, 180, -30));
                allPoints.add(new CurvePoint(-13, 8, 0.6, 0.3, 20, 180, 0));
                allPoints.add(new CurvePoint(-13, -4, 0.6, 0.3, 20, 180, 0));
                allPoints.add(new CurvePoint(-13, -12, 0.6, 0.3, 20, 180, 0));
                allPoints.add(new CurvePoint(0, -8, 0.6, 0.3, 20, 180, 0));
                allPoints.add(new CurvePoint(10, -2, 0.6, 0.3, 25, 180, 0));
                allPoints.add(new CurvePoint(34, 0, 0.6, 0.3, 25, 180, 0));
                allPoints.add(new CurvePoint(50, 0, 0.6, 0.3, 25, 180, 0));

                myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 0, 5,
                        ArmShoulderPositions.LEVEL3, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY, 0.7, 0, "none", 0);

//                allPoints = new ArrayList<>();
//                allPoints.add(new CurvePoint(34, 0, 0.8, 0.4, 25, 0, 0));
//                allPoints.add(new CurvePoint(34, 22, 0.8, 0.4, 25, 180, 0));
//                allPoints.add(new CurvePoint(34, 60, 0.8, 0.4, 25, 180, 0));

//                myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, 0, 3);

            }else {
//                allPoints = new ArrayList<>();
//                allPoints.add(new CurvePoint(34, 0, 0.8, 0.4, 25, 0, 0));
//                allPoints.add(new CurvePoint(34, 22, 0.8, 0.4, 25, 180, 0));
//                allPoints.add(new CurvePoint(34, 60, 0.8, 0.4, 25, 180, 0));
//
//                myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, 0, 3);
             }

        }

        myPurePursuitRobotMovement6_Turn_MultiThread.stopOdometryThread();

    }

}

