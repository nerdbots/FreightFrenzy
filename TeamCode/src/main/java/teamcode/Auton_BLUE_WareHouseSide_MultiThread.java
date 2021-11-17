package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ArmShoulderPositions;
import org.firstinspires.ftc.teamcode.DuckDetector;
import org.firstinspires.ftc.teamcode.FingerPositions;

import java.util.ArrayList;

import Odometry.OdometryGlobalCoordinatePositionNERD;
import treamcode.CurvePoint;


@Autonomous(name="Auton_BLUE_WareHouseSide_MultiThread", group="Linear Opmode")

public class Auton_BLUE_WareHouseSide_MultiThread extends LinearOpMode {

    private PurePursuitRobotMovement6_Turn_MultiThread myPurePursuitRobotMovement6_Turn_MultiThread;

    boolean debugFlag = true;

    double armDelay = 0.0;
    double shippingHubPark = 0;

    int purePursuitPath = 1;
    DuckDetector.DuckDeterminationPipeline.DuckPosition duckPosition;
    DuckDetector duckDetector;

    public  volatile ArmShoulderPositions shoulderPosition = ArmShoulderPositions.INTAKE;
    public  volatile FingerPositions fingerPosition = FingerPositions.ENTER_INTAKE;


    @Override
    public void runOpMode() {


        telemetry.addData("Vision", "Completed");
        telemetry.update();

        //Create a NerdBOT object
        myPurePursuitRobotMovement6_Turn_MultiThread = new PurePursuitRobotMovement6_Turn_MultiThread(this);
        myPurePursuitRobotMovement6_Turn_MultiThread.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6_Turn_MultiThread.initializeHardware();


        duckDetector = new DuckDetector(this);
        duckDetector.initDuckDetector();


        telemetry.update();

        waitForStart();

        myPurePursuitRobotMovement6_Turn_MultiThread.startOdometryThread();

//        telemetry.addData("Analysis",duckDetector.getAnalysis());
//        telemetry.update();
//
//        duckPosition = duckDetector.getAnalysis();
//
//        duckDetector.closeCameraDevice();

        duckPosition = duckDetector.getAnalysis();
        telemetry.addData("Analysis",duckDetector.getAnalysis());
        telemetry.update();

        duckDetector.closeCameraDevice();


        myPurePursuitRobotMovement6_Turn_MultiThread.printI();

        myPurePursuitRobotMovement6_Turn_MultiThread.resetITerm();

        myPurePursuitRobotMovement6_Turn_MultiThread.printI();

        myPurePursuitRobotMovement6_Turn_MultiThread.resetTimers();


//        if(duckPosition.equals("LEFT"))
        if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.CENTER)) {
            shoulderPosition = ArmShoulderPositions.LEVEL2;
            armDelay=0.5;
            shippingHubPark = 22;
        }
        else if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.LEFT)){
            shoulderPosition = ArmShoulderPositions.LEVEL1;
            armDelay=0.5;
            shippingHubPark = 25;
        }
        else if(duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.RIGHT)){
            shoulderPosition = ArmShoulderPositions.LEVEL3;
            armDelay = 0.0;
            shippingHubPark = 27;
        }

//        if (duckPosition == DuckDetector.DuckDeterminationPipeline.DuckPosition.CENTER){
        if (purePursuitPath == 1){

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.4, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(8, shippingHubPark, 0.4, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(80, 80, 0.4, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 10, 235, 3, ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.0,0,"none", 0);

            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.ENTER_INTAKE);

            sleep(1000);
            myPurePursuitRobotMovement6_Turn_MultiThread.turnRobot(270);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(8, shippingHubPark, 0.6, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(15, -20, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-30, -2, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-60, 0, 0.6, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 180, 8, shoulderPosition,ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.ENTER_INTAKE,0, 0,"none", 0);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-30, -2, 0.4, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-30, 20, 0.4, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-30, 60, 0.4, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, 180, 3);

//            myPurePursuitRobotMovement6_Turn.runMotor("duckyDisc",1,4);
////            sleep(2000);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-24, 8, 0.8, 0.3, 40, 0, 0.3));
//            allPoints.add(new CurvePoint(0, -4, 0.8, 0.3, 40, 180, 0.3));
//            allPoints.add(new CurvePoint(83, -6, 0.8, 0.3, 40, 180, 0.3));
//            allPoints.add(new CurvePoint(130, -6, 0.8, 0.3, 40, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Turn.followCurve(allPoints, 0, 35, -180, 10);

            //----------------------------------------------

        }

        myPurePursuitRobotMovement6_Turn_MultiThread.stopOdometryThread();

    }

}

