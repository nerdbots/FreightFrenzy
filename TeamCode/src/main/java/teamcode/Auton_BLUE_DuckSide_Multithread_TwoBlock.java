package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ArmShoulderPositions;
import org.firstinspires.ftc.teamcode.DuckDetector;
import org.firstinspires.ftc.teamcode.FingerPositions;

import java.util.ArrayList;

import treamcode.CurvePoint;

@Disabled
@Autonomous(name="Auton_BLUE_DuckSide_TwoBlock", group="Linear Opmode")

public class Auton_BLUE_DuckSide_Multithread_TwoBlock extends LinearOpMode {

    private PurePursuitRobotMovement6_Turn_MultiThread myPurePursuitRobotMovement6_Turn_Multithread;

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
        myPurePursuitRobotMovement6_Turn_Multithread = new PurePursuitRobotMovement6_Turn_MultiThread(this);
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
            shippingHubPark = 26;
        }

        if (purePursuitPath == 1){
            //First Path to the Square 4

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.4, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(0, 7, 0.4, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-15, shippingHubPark, 0.4, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-80, 80, 0.4, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_Multithread.followCurveArm(allPoints, 0, 10, -90, 3, ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.0,1,"none", 0);

            myPurePursuitRobotMovement6_Turn_Multithread.turnRobot(-55);

            myPurePursuitRobotMovement6_Turn_Multithread.setFingerPositions(FingerPositions.ENTER_INTAKE);

            sleep(500);



            myPurePursuitRobotMovement6_Turn_Multithread.turnRobot(-90);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-15, shippingHubPark, 0.4, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(22, 10, 0.4, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(60, 9, 0.4, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_Multithread.followCurveArm(allPoints, 0, 15, -160, 3, shoulderPosition,ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.ENTER_INTAKE,0, 0,"none", 0);

            myPurePursuitRobotMovement6_Turn_Multithread.runMotor("duckyDisc",-0.8,4);


            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(22, 8, 0.6, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(10, 0, 0.6, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(0, -3, 0.6, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-18, -3, 0.6, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-36, -2, 0.6, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-81, -2, 0.6, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_Multithread.followCurveArm(allPoints, 0, 35, -180, 6, ArmShoulderPositions.INTAKE, ArmShoulderPositions.INTAKE, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "intake",-1);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-36, -1, 0.4, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-36, shippingHubPark, 0.4, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-36, 80, 0.4, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_Multithread.followCurveArm(allPoints, 0, 10, -90, 3, ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.0,0,"none", 0);

            myPurePursuitRobotMovement6_Turn_Multithread.turnRobot(-115);

            myPurePursuitRobotMovement6_Turn_Multithread.setFingerPositions(FingerPositions.ENTER_INTAKE);

            sleep(500);

            myPurePursuitRobotMovement6_Turn_Multithread.turnRobot(-75);


            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-36, shippingHubPark, 0.6, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-31, -13, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-55, -7.5, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-79, 0, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-115, 0, 0.6, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_Multithread.followCurveArm(allPoints, 0, 15, -180, 7, shoulderPosition,ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.ENTER_INTAKE,0, 0,"none", 0);

            //----------------------------------------------


        }
        myPurePursuitRobotMovement6_Turn_Multithread.stopOdometryThread();

    }

}

