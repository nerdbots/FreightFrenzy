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
@Autonomous(name="Auton_RED_DuckSide", group="Linear Opmode")

public class Auton_RED_DuckSide extends LinearOpMode {

    private PurePursuitRobotMovement6_Turn myPurePursuitRobotMovement6_Turn;

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
        myPurePursuitRobotMovement6_Turn = new PurePursuitRobotMovement6_Turn(this);
        myPurePursuitRobotMovement6_Turn.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6_Turn.initializeHardware();


        duckDetector = new DuckDetector(this);
        duckDetector.initDuckDetector();

        telemetry.addData("NerdBOT", "Initialized");
        telemetry.update();

        waitForStart();

        duckPosition = duckDetector.getAnalysis();
        telemetry.addData("Analysis",duckDetector.getAnalysis());
        telemetry.update();

        duckDetector.closeCameraDevice();


        myPurePursuitRobotMovement6_Turn.printI();

        myPurePursuitRobotMovement6_Turn.resetITerm();

        myPurePursuitRobotMovement6_Turn.printI();

        myPurePursuitRobotMovement6_Turn.resetTimers();

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

        if (purePursuitPath == 1){
            //First Path to the Square 4

//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(0, 0, 0.4, 0.3, 40, 0, 0.3));
//            allPoints.add(new CurvePoint(0, 50, 0.4, 0.3, 40, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 100, 0.4, 0.3, 40, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Turn.followCurve(allPoints, 0, 15, 180, 3);

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.4, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(8, shippingHubPark, 0.4, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(80, 80, 0.4, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 10, 270, 3, ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.0,0,"none", 0);

            myPurePursuitRobotMovement6_Turn.turnRobot(225);

            myPurePursuitRobotMovement6_Turn.setFingerPositions(FingerPositions.ENTER_INTAKE);

            sleep(1000);

//            myPurePursuitRobotMovement6_Turn.turnRobot(-45);
//            sleep(2000);

//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(23, 22, 0.4, 0.4, 15, 0, 0.3));
//            allPoints.add(new CurvePoint(0, 5, 0.4, 0.4, 15, 180, 0.3));
//            allPoints.add(new CurvePoint(40, -2, 0.4, 0.4, 15, 180, 0.3));
//            allPoints.add(new CurvePoint(85, -2, 0.4, 0.4, 15, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 10, 0, 3, shoulderPosition,ArmShoulderPositions.INTAKE,FingerPositions.ENTER_INTAKE,FingerPositions.ENTER_INTAKE,armDelay,1,"intake", -1);
//
////            allPoints = new ArrayList<>();
////            allPoints.add(new CurvePoint(-27, 22, 0.6, 0.3, 25, 0, 0.3));
////            allPoints.add(new CurvePoint(15, 10, 0.6, 0.3, 25, 180, 0.3));
////            allPoints.add(new CurvePoint(80, 0, 0.6, 0.3, 25, 180, 0.3));
////
//////   11_09        myPurePursuitRobotMovement6_Turn.followCurve(allPoints, 0, 15, -120, 3);
////            myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 10, 240, 3, ArmShoulderPositions.HOME,FingerPositions.ENTER_INTAKE,FingerPositions.ENTER_INTAKE,0.5,0,"none",0);
////
////            allPoints = new ArrayList<>();
////            allPoints.add(new CurvePoint(15, 10, 0.6, 0.4, 25, 0, 0.3));
////            allPoints.add(new CurvePoint(-34, 10, 0.6, 0.4, 25, 180, 0.3));
////            allPoints.add(new CurvePoint(-85, 10, 0.6, 0.4, 25, 180, 0.3));
////
////            myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 10, 200, 3, ArmShoulderPositions.INTAKE,FingerPositions.ENTER_INTAKE,FingerPositions.INTAKE_READY,0,0,"intake", -1);
//
//            myPurePursuitRobotMovement6_Turn.runMotor("intake", -1,1);
//
////            sleep(2000);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(40, 0, 0.4, 0.4, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(23, shippingHubPark, 0.4, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(-10, 60, 0.4, 0.4, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 10, -45, 3, ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0,1,"none", 0);
//            myPurePursuitRobotMovement6_Turn.turnRobot(-90);
//            myPurePursuitRobotMovement6_Turn.setFingerPositions(FingerPositions.ENTER_INTAKE);

//            sleep(2000);

            myPurePursuitRobotMovement6_Turn.turnRobot(270);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(14, shippingHubPark, 0.6, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-27, 6, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-60, 0, 0.6, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 15, 160, 3, shoulderPosition,ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.ENTER_INTAKE,0, 0,"none", 0);

            myPurePursuitRobotMovement6_Turn.runMotor("duckyDisc",1,4);
//            sleep(2000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-27, 8, 0.8, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(0, -4, 0.8, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(83, -6, 0.8, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(130, -6, 0.8, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn.followCurve(allPoints, 0, 35, 180, 10);

            //----------------------------------------------


        }
    }

}

