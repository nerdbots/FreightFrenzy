package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ArmShoulderPositions;
import org.firstinspires.ftc.teamcode.DuckDetector;
import org.firstinspires.ftc.teamcode.FingerPositions;

import java.util.ArrayList;

import treamcode.CurvePoint;


@Autonomous(name="Auton_RED_DuckSide_TwoBlock_V2", group="Linear Opmode")

public class Auton_RED_DuckSide_Multithread_TwoBlock_V2 extends LinearOpMode {

    private PurePursuitRobotMovement6_Turn_MultiThread_V2 myPurePursuitRobotMovement6_Turn_Multithread;

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


        //Create a NerdBOT object
        myPurePursuitRobotMovement6_Turn_Multithread = new PurePursuitRobotMovement6_Turn_MultiThread_V2(this);
        myPurePursuitRobotMovement6_Turn_Multithread.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6_Turn_Multithread.initializeHardware();

        duckDetector = new DuckDetector(this);
        duckDetector.initDuckDetector();
        telemetry.addData("NerdBOT", "Initialized");
        telemetry.update();

        waitForStart();

        //Start Odo thread
        myPurePursuitRobotMovement6_Turn_Multithread.startOdometryThread();

        duckPosition = duckDetector.getAnalysis();
        telemetry.addData("Analysis",duckDetector.getAnalysis());
        telemetry.update();

        duckDetector.closeCameraDevice();

        myPurePursuitRobotMovement6_Turn_Multithread.printI();

        myPurePursuitRobotMovement6_Turn_Multithread.resetITerm();

        myPurePursuitRobotMovement6_Turn_Multithread.printI();

        myPurePursuitRobotMovement6_Turn_Multithread.resetTimers();


        if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.CENTER)) {
            shoulderPosition = ArmShoulderPositions.LEVEL2;
            armDelay=0.7;
            shippingHubPark = 22;
        }
        else if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.LEFT)){
            shoulderPosition = ArmShoulderPositions.LEVEL1;
            armDelay=0.7;
            shippingHubPark = 25;
        }
        else if(duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.RIGHT)){
            shoulderPosition = ArmShoulderPositions.LEVEL3;
            armDelay = 0.0;
            shippingHubPark = 27;
        }

        if (purePursuitPath == 1){
            //First Path to the Square 4

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.4, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(8, shippingHubPark, 0.4, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(80, 80, 0.4, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_Multithread.followCurveArm_V2(allPoints, 0, 10, 270, 3, ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0,0,"none", 0);

            myPurePursuitRobotMovement6_Turn_Multithread.turnRobot(235);

            myPurePursuitRobotMovement6_Turn_Multithread.setFingerPositions(FingerPositions.ENTER_INTAKE);

            sleep(500);

            myPurePursuitRobotMovement6_Turn_Multithread.turnRobot(270);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(14, shippingHubPark, 0.4, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-28, 7, 0.4, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-60, 0, 0.4, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_Multithread.followCurveArm_V2(allPoints, 0, 10, 150, 3, shoulderPosition,ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.ENTER_INTAKE,0.7, 0,"none", 0);

            myPurePursuitRobotMovement6_Turn_Multithread.runMotor("duckyDisc",0.8,4);

            myPurePursuitRobotMovement6_Turn_Multithread.turnRobot(0);


            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-28, 7, 0.6, 0.3, 30, 0, 0.3));
            allPoints.add(new CurvePoint(0, 0, 0.6, 0.3, 30, 180, 0.3));
            allPoints.add(new CurvePoint(30, -1, 0.6, 0.3, 30, 180, 0.3));
            allPoints.add(new CurvePoint(81, -3, 0.6, 0.3, 30, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_Multithread.followCurveArm_V2(allPoints, 0, 35, 0, 5, ArmShoulderPositions.INTAKE, ArmShoulderPositions.INTAKE, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "intake",-0.7);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(30, 0, 0.4, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(30, shippingHubPark, 0.4, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(30, 80, 0.4, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_Multithread.followCurveArm_V2(allPoints, 0, 10, -90, 3, ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.0,0,"none", 0);

            myPurePursuitRobotMovement6_Turn_Multithread.turnRobot(-20);

            myPurePursuitRobotMovement6_Turn_Multithread.setFingerPositions(FingerPositions.ENTER_INTAKE);

            sleep(500);

            myPurePursuitRobotMovement6_Turn_Multithread.turnRobot(-90);


            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(30, shippingHubPark, 0.6, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(25, -15, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(50, -8.5, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(75, -2, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(120, -20, 0.6, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_Multithread.followCurveArm_V2(allPoints, 0, 15, -180, 5, shoulderPosition,ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.ENTER_INTAKE,0, 0,"none", 0);

            //----------------------------------------------


        }
        myPurePursuitRobotMovement6_Turn_Multithread.stopOdometryThread();

    }

}

