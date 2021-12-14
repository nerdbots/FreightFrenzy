package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ArmShoulderPositions;
import org.firstinspires.ftc.teamcode.DuckDetector;
import org.firstinspires.ftc.teamcode.FingerPositions;

import java.util.ArrayList;

import treamcode.CurvePoint;


@Autonomous(name="BLUE_DuckSide_2BlockAP", group="Linear Opmode")

public class Auton_BLUE_DuckSide_TwoBlockAP_V2 extends LinearOpMode {

    private PurePursuitRobotMovement6_Turn_MultiThread_V2 myPurePursuitRobotMovement6_Turn_MultiThread;

    boolean debugFlag = true;

    double armDelay = 0.0;
    double shippingHubPark = 0;
    double shpX = 0;
    double shpY = 0;

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
//            shippingHubPark = 25;
            shpX = 2;
            shpY = 35;
        }
        else if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.LEFT)){
            shoulderPosition = ArmShoulderPositions.LEVEL1;
            armDelay=0.5;
//            shippingHubPark = 29;
            shpX = 2;
            shpY = 35;
        }
        else if(duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.RIGHT)){
            shoulderPosition = ArmShoulderPositions.LEVEL3;
            armDelay = 0.0;
//            shippingHubPark = 30;
            shpX = 4;
            shpY = 35;
        }

        if (purePursuitPath == 1){

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.5, 0.3, 10, 0, 90));
//            allPoints.add(new CurvePoint(0, 3, 0.4, 0.3, 10, 180, 50));
//            allPoints.add(new CurvePoint(0, 10, 0.4, 0.3, 10, 180, 30));
//            allPoints.add(new CurvePoint(0, 20, 0.4, 0.3, 10, 180, 0));
            allPoints.add(new CurvePoint(0, 30, 0.5, 0.3, 10, 180, 0));
            allPoints.add(new CurvePoint(0, shpY, 0.5, 0.3, 10, 180, 0));
            allPoints.add(new CurvePoint(-7, shpY, 0.5, 0.3, 25, 180, 0));
            allPoints.add(new CurvePoint(-80, 80, 0.5, 0.3, 25, 180, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 10, 0, 2,
                    ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.0,1,"none", 0);

            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.INTAKE_READY);

            sleep(300);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-7, shpY, 0.6, 0.4, 25, 0, 0));
            allPoints.add(new CurvePoint(22, 10, 0.6, 0.4, 25, 180, -160));
            allPoints.add(new CurvePoint(60, 9, 0.6, 0.4, 25, 180, -160));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, -160, 2,
                    shoulderPosition,ArmShoulderPositions.INTAKE, FingerPositions.INTAKE_READY, FingerPositions.ENTER_INTAKE,0.7, 0,"none", 0);

            myPurePursuitRobotMovement6_Turn_MultiThread.runMotor("duckyDisc",-0.5,4);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(22, 10, 0.6, 0.3, 10, 0, -180));
            allPoints.add(new CurvePoint(10, 12, 0.6, 0.3, 10, 180, -180));
            allPoints.add(new CurvePoint(5, 12, 0.6, 0.3, 25, 180, -180));
            allPoints.add(new CurvePoint(0, -3, 0.6, 0.3, 25, 180, -180));
            allPoints.add(new CurvePoint(-18, -3, 0.6, 0.3, 25, 180, -180));
            allPoints.add(new CurvePoint(-36, -2, 0.6, 0.3, 25, 180, -180));
            allPoints.add(new CurvePoint(-81, -2, 0.6, 0.3, 25, 180, -180));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 35, -180, 6,
                    ArmShoulderPositions.INTAKE, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY, 0, 0, "intake",-0.7);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-36, -2, 0.4, 0.3, 25, 0, -180));
            allPoints.add(new CurvePoint(-30, 6, 0.4, 0.3, 25, 0, -140));
            allPoints.add(new CurvePoint(-24, 13, 0.4, 0.3, 25, 0, -100));
            allPoints.add(new CurvePoint(-18, 20, 0.4, 0.3, 25, 0, -40));
            allPoints.add(new CurvePoint(-12, 27, 0.4, 0.3, 25, 0, -20));
            allPoints.add(new CurvePoint(-7, 30, 0.4, 0.3, 25, 180, -20));
            allPoints.add(new CurvePoint(36, 80, 0.4, 0.3, 25, 180, -20));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 10, -20, 3,
                    ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.0,0,"none", 0);

            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.INTAKE_READY);

            sleep(300);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-7, 30, 0.6, 0.4, 25, 0, -20));
            allPoints.add(new CurvePoint(-20, 18, 0.6, 0.4, 25, 0, -90));
            allPoints.add(new CurvePoint(-31, 0, 0.6, 0.4, 25, 180, -180));
            allPoints.add(new CurvePoint(-55, -7.5, 0.6, 0.4, 25, 180, -180));
            allPoints.add(new CurvePoint(-79, 0, 0.6, 0.4, 25, 180, -180));
            allPoints.add(new CurvePoint(-115, 0, 0.6, 0.4, 25, 180, -180));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, -180, 7,
                    shoulderPosition, ArmShoulderPositions.INTAKE, FingerPositions.INTAKE_READY, FingerPositions.ENTER_INTAKE,0.7, 0,"none", 0);


        }

        myPurePursuitRobotMovement6_Turn_MultiThread.stopOdometryThread();

    }

}

