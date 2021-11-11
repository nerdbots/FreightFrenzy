package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DuckDetector;

import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.*;
import treamcode.CurvePoint;


@Autonomous(name="FreightFrenzy_Auton_Blue_Disk", group="Linear Opmode")

public class FreightFrenzy_Auton_Blue_Disk extends LinearOpMode {

    private PurePursuitRobotMovement6_Turn myPurePursuitRobotMovement6_Turn;

    boolean debugFlag = true;

    int purePursuitPath = 1;
    DuckDetector.DuckDeterminationPipeline.DuckPosition duckPosition;
    DuckDetector duckDetector;

    public volatile ArmShoulderPositions shoulderPosition = ArmShoulderPositions.INTAKE;
    public volatile FingerPositions fingerPosition = FingerPositions.ENTER_INTAKE;


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
            allPoints.add(new CurvePoint(0, 0, 0.4, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(27, 22, 0.4, 0.3, 25, 180, 0.3));
            allPoints.add(new CurvePoint(80, 80, 0.4, 0.3, 25, 180, 0.3));

  //11_09          myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 15, 90, 3, shoulderPosition, FingerPositions.GRAB,0, "none", 0);

            sleep(2000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(27, 22, 0.4, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(34, 10, 0.4, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(45, -30, 0.4, 0.4, 25, 180, 0.3));

            //11_09        myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 10, 120, 3, ArmShoulderPositions.INTAKE,FingerPositions.ENTER_INTAKE,0,"intake", 1);

            //11_09          myPurePursuitRobotMovement6_Turn.runMotor("intake", 1,2);

            sleep(2000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(34, 10, 0.4, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(25, 22, 0.4, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(0, 48, 0.4, 0.4, 25, 180, 0.3));

            //11_09      myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 10, 90, 3, shoulderPosition, FingerPositions.GRAB, 0,"none", 0);

            sleep(2000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(25, 22, 0.4, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-21, 8, 0.4, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(60, 8, 0.4, 0.4, 25, 180, 0.3));

            //11_09     myPurePursuitRobotMovement6_Turn.followCurveArm(allPoints, 0, 10, 160, 1, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE,0, "none", 0);

            //11_09      myPurePursuitRobotMovement6_Turn.runMotor("duckyDisc",1,2);
            sleep(2000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-21, 8, 0.8, 0.3, 40, 0, 0.3));
            allPoints.add(new CurvePoint(24, 2, 0.8, 0.3, 40, 180, 0.3));
            allPoints.add(new CurvePoint(83, -1, 0.8, 0.3, 40, 180, 0.3));
            allPoints.add(new CurvePoint(130, -1, 0.8, 0.3, 40, 180, 0.3));

            //11_09      myPurePursuitRobotMovement6_Turn.followCurve(allPoints, -0.25, 35, 180, 5);

            //----------------------------------------------



        }
    }

}

