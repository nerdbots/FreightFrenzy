package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DuckDetector;

import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.*;
import treamcode.CurvePoint;


@Autonomous(name="PurePursuitOpMode_Original_Turn", group="Linear Opmode")

public class PurePursuitOpMode_Original_Turn extends LinearOpMode {

    private PurePursuitRobotMovement6_Turn myPurePursuitRobotMovement6_Turn;

    boolean debugFlag = true;

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
        myPurePursuitRobotMovement6_Turn = new PurePursuitRobotMovement6_Turn(this);
        myPurePursuitRobotMovement6_Turn.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6_Turn.initializeHardware();


        duckDetector = new DuckDetector(this);
        duckDetector.initDuckDetector();
        telemetry.addData("Analysis",duckDetector.getAnalysis());
        telemetry.update();

        waitForStart();

        telemetry.addData("Analysis",duckDetector.getAnalysis());
        telemetry.update();

        duckPosition = duckDetector.getAnalysis();

        duckDetector.closeCameraDevice();


//        if(duckPosition.equals("LEFT"))
        if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.CENTER)) {
            shoulderPosition = ArmShoulderPositions.LEVEL2;
        }
        else if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.LEFT)){
            shoulderPosition = ArmShoulderPositions.LEVEL1;
        }
        else if(duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.RIGHT)){
            shoulderPosition = ArmShoulderPositions.LEVEL3;
        }

//        if (duckPosition == DuckDetector.DuckDeterminationPipeline.DuckPosition.CENTER){
        if (purePursuitPath == 1){
            //First Path to the Square 4

//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(0, 0, 0.4, 0.3, 40, 0, 0.3));
//            allPoints.add(new CurvePoint(0, 50, 0.4, 0.3, 40, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 100, 0.4, 0.3, 40, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Turn.followCurve(allPoints, 0, 15, 180, 3);

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.35, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-27, 22, 0.35, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-80, 80, 0.35, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 10, 270, 3, shoulderPosition, FingerPositions.GRAB, 0,0.5,"none", 0);
//            myPurePursuitRobotMovement6_Turn.turnRobot(-45);
//            sleep(2000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-27, 22, 0.6, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(15, 10, 0.6, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(80, 0, 0.6, 0.3, 25, 180, 0.3));

//   11_09        myPurePursuitRobotMovement6_Turn.followCurve(allPoints, 0, 15, -120, 3);
            myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 15, 240, 3, ArmShoulderPositions.HOME,FingerPositions.ENTER_INTAKE,0.5,0,"none",0);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(15, 10, 0.6, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-34, 10, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-85, 10, 0.6, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 15, 200, 3, ArmShoulderPositions.INTAKE,FingerPositions.ENTER_INTAKE,0,0,"intake", -1);

            myPurePursuitRobotMovement6_Turn.runMotor("intake", -1,1);

//            sleep(2000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-34, 10, 0.3, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-25, 24, 0.3, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(0, 48, 0.3, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 10, 200, 3, shoulderPosition, FingerPositions.GRAB, 0,0.5,"none", 0);

            myPurePursuitRobotMovement6_Turn.turnRobot(270);

//            sleep(2000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-25, 24, 0.6, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(19, 9, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(60, 9, 0.6, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 15, 200, 3, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, 0.5, 0,"none", 0);

            myPurePursuitRobotMovement6_Turn.runMotor("duckyDisc",-1,3);
//            sleep(2000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(19, 9, 0.8, 0.3, 40, 0, 0.3));
            allPoints.add(new CurvePoint(-24, 2, 0.8, 0.3, 40, 180, 0.3));
            allPoints.add(new CurvePoint(-83, -1, 0.8, 0.3, 40, 180, 0.3));
            allPoints.add(new CurvePoint(-130, -1, 0.8, 0.3, 40, 180, 0.3));

            myPurePursuitRobotMovement6_Turn.followCurve(allPoints, -0.25, 35, 180, 7);

            //----------------------------------------------


        }
    }

}

