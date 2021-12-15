package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ArmShoulderPositions;
import org.firstinspires.ftc.teamcode.DuckDetector;
import org.firstinspires.ftc.teamcode.FingerPositions;

import java.util.ArrayList;

import treamcode.CurvePoint;


@Autonomous(name="BLUE_WarehouseSide_MultiBlock", group="Linear Opmode")

public class Auton_BLUE_WareHouseSide_MultiBlock_V2 extends LinearOpMode {

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


    @Override
    public void runOpMode() {

        //Create a NerdBOT object
        myPurePursuitRobotMovement6_Turn_MultiThread = new PurePursuitRobotMovement6_Turn_MultiThread_V2(this);
        myPurePursuitRobotMovement6_Turn_MultiThread.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6_Turn_MultiThread.initializeHardware();

        duckDetector = new DuckDetector(this);
        duckDetector.initDuckDetector();

        telemetry.addData("NerdBOT", "Initialized");
        telemetry.update();

        waitForStart();

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
            shippingHubPark = 25;
            shpX = 2;
            shpY = 27;
        }
        else if (duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.LEFT)){
            shoulderPosition = ArmShoulderPositions.LEVEL1;
            armDelay=0.5;
            shippingHubPark = 29;
            shpX = 2;
            shpY = 29;
        }
        else if(duckPosition.equals(DuckDetector.DuckDeterminationPipeline.DuckPosition.RIGHT)){
            shoulderPosition = ArmShoulderPositions.LEVEL3;
            armDelay = 0.0;
            shippingHubPark = 30;
            shpX = 4;
            shpY = 28;
        }

        if (purePursuitPath == 1){

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.4, 0.3, 25, 180, 90));
            allPoints.add(new CurvePoint(shpX, shpY, 0.4, 0.3, 25, 180, 225));
            allPoints.add(new CurvePoint(0, 68, 0.4, 0.3, 25, 180, 225));


            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 225, 2, ArmShoulderPositions.INTAKE, shoulderPosition, FingerPositions.GRAB, FingerPositions.GRAB,0.0,0,"none", 0);

            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.INTAKE_READY);

            sleep(300);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(shpX, shpY, 0.8, 0.3, 20, 180, 225));
            allPoints.add(new CurvePoint(5, -4, 0.6, 0.3, 20, 180, 180));
            allPoints.add(new CurvePoint(0, -12, 0.6, 0.3, 20, 180, 180));
            allPoints.add(new CurvePoint(-10, -8, 0.6, 0.3, 20, 180, 180));
            allPoints.add(new CurvePoint(-20, -2, 0.6, 0.3, 25, 180, 180));
            allPoints.add(new CurvePoint(-35, 0, 0.6, 0.3, 25, 180, 180));
            allPoints.add(new CurvePoint(-70, 0, 0.6, 0.3, 25, 180, 180));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 180, 5, shoulderPosition,ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY,0.7, 0,"none", 0);

            myPurePursuitRobotMovement6_Turn_MultiThread.AutonBlockIntake();

            sleep(1000);

            //Reset robot position along the wall before exiting the warehouse
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-28, 15, 0.8, 0.3, 15, 180, 180));
            allPoints.add(new CurvePoint(-28, 0, 0.8, 0.3, 25, 180, 180));
            allPoints.add(new CurvePoint(-28, -50, 0.8, 0.3, 25, 180, 180));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 180, 6, ArmShoulderPositions.INTAKE, ArmShoulderPositions.INTAKE, FingerPositions.GRAB, FingerPositions.GRAB,0.7, 0,"none", 0.0);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-35, 0, 0.4, 0.3, 15, 180, 180));
            allPoints.add(new CurvePoint(-20, -6, 0.8, 0.3, 25, 180, 180));
            allPoints.add(new CurvePoint(0, -6, 0.8, 0.3, 25, 180, 245));
            allPoints.add(new CurvePoint(5, -2, 0.8, 0.3, 25, 180, 245));
            allPoints.add(new CurvePoint(10, 24, 0.8, 0.3, 25, 180, 245));
            allPoints.add(new CurvePoint(20, 72, 0.8, 0.3, 25, 180, 245));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 245, 2, ArmShoulderPositions.INTAKE, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB,0.7, 0,"intake", 0.5);

            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.INTAKE_READY);

            sleep(300);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(12, 20, 0.6, 0.4, 25, 180, 245));
            allPoints.add(new CurvePoint(6, 0, 0.6, 0.4, 25, 180, 180));
            allPoints.add(new CurvePoint(0, -12, 0.6, 0.4, 25, 180, 180));
            allPoints.add(new CurvePoint(-10, -8, 0.6, 0.4, 25, 180, 180));
            allPoints.add(new CurvePoint(-20, -4, 0.6, 0.4, 25, 180, 180));
            allPoints.add(new CurvePoint(-37, 0, 0.6, 0.4, 25, 180, 180));
            allPoints.add(new CurvePoint(-72, 0, 0.6, 0.4, 25, 180, 180));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 180, 5, ArmShoulderPositions.LEVEL3, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY,0.7, 0,"none", 0);

            myPurePursuitRobotMovement6_Turn_MultiThread.AutonBlockIntake();

            sleep(1000);

            //Reset robot position along the wall before exiting the warehouse
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-28, 15, 0.8, 0.3, 15, 180, 180));
            allPoints.add(new CurvePoint(-28, 0, 0.8, 0.3, 25, 180, 180));
            allPoints.add(new CurvePoint(-28, -50, 0.8, 0.3, 25, 180, 180));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 180, 6, ArmShoulderPositions.INTAKE, ArmShoulderPositions.INTAKE, FingerPositions.GRAB, FingerPositions.GRAB,0.7, 0,"none", 0.0);


            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-37, 0, 0.4, 0.3, 15, 180, 180));
            allPoints.add(new CurvePoint(-20, -4, 0.8, 0.3, 25, 180, 180));
            allPoints.add(new CurvePoint(0, -4, 0.8, 0.3, 25, 180, 245));
            allPoints.add(new CurvePoint(5, -2, 0.8, 0.3, 25, 180, 245));
            allPoints.add(new CurvePoint(10, 26, 0.8, 0.3, 25, 180, 245));
            allPoints.add(new CurvePoint(20, 78, 0.8, 0.3, 25, 180, 245));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 245, 2, ArmShoulderPositions.INTAKE, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB,0.7, 0.5,"intake", 0.5);

            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.INTAKE_READY);

            sleep(300);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(10, 21, 0.6, 0.4, 25, 180, 245));
            allPoints.add(new CurvePoint(5, 0, 0.6, 0.4, 25, 180, 180));
            allPoints.add(new CurvePoint(0, -10, 0.6, 0.4, 25, 180, 180));
            allPoints.add(new CurvePoint(-10, -6, 0.6, 0.4, 25, 180, 180));
            allPoints.add(new CurvePoint(-20, -4, 0.6, 0.4, 25, 180, 180));
            allPoints.add(new CurvePoint(-39, 3, 0.6, 0.4, 25, 180, 180));
            allPoints.add(new CurvePoint(-74, 3, 0.6, 0.4, 25, 180, 180));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 180, 5, ArmShoulderPositions.LEVEL3, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY,0.7, 0,"none", 0);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-39, 3, 0.8, 0.4, 25, 0, 180));
            allPoints.add(new CurvePoint(-39, 22, 0.8, 0.4, 25, 180, 180));
            allPoints.add(new CurvePoint(-39, 60, 0.8, 0.4, 25, 180, 180));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, 180, 3);


//            myPurePursuitRobotMovement6_Turn_MultiThread.AutonBlockIntake();
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-39, 0, 0.4, 0.3, 15, 180, 180));
//            allPoints.add(new CurvePoint(-20, -4, 0.8, 0.3, 25, 180, 180));
//            allPoints.add(new CurvePoint(0, -4, 0.8, 0.3, 25, 180, 235));
//            allPoints.add(new CurvePoint(5, -2, 0.8, 0.3, 25, 180, 235));
//            allPoints.add(new CurvePoint(8, 26, 0.8, 0.3, 25, 180, 235));
//            allPoints.add(new CurvePoint(14, 60, 0.8, 0.3, 25, 180, 235));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 235, 2, ArmShoulderPositions.INTAKE, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB,0, 0,"intake", 0.5);
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.INTAKE_READY);
//
//            sleep(300);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(8, 22, 0.8, 0.4, 25, 180, 235));
//            allPoints.add(new CurvePoint(6, 0, 0.8, 0.4, 25, 180, 180));
//            allPoints.add(new CurvePoint(0, -6, 0.8, 0.4, 25, 180, 180));
//            allPoints.add(new CurvePoint(-10, -2, 0.8, 0.4, 25, 180, 180));
//            allPoints.add(new CurvePoint(-20, -2, 0.8, 0.4, 25, 180, 180));
//            allPoints.add(new CurvePoint(-41, 2, 0.8, 0.4, 25, 180, 180));
//            allPoints.add(new CurvePoint(-76, 2, 0.8, 0.4, 25, 180, 180));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 180, 6, ArmShoulderPositions.LEVEL3, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY,0.7, 0,"none", 0);

//            myPurePursuitRobotMovement6_Turn_MultiThread.AutonBlockIntake();
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-41, 0, 0.4, 0.3, 25, 180, 180));
//            allPoints.add(new CurvePoint(-20, -6, 0.8, 0.3, 25, 180, 180));
//            allPoints.add(new CurvePoint(0, -6, 0.8, 0.3, 25, 180, 235));
//            allPoints.add(new CurvePoint(4, -2, 0.8, 0.3, 25, 180, 235));
//            allPoints.add(new CurvePoint(6, 23, 0.8, 0.3, 25, 180, 235));
//            allPoints.add(new CurvePoint(10, 60, 0.8, 0.3, 25, 180, 235));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 235, 2, ArmShoulderPositions.INTAKE, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB,0, 0,"intake", 0.5);
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.INTAKE_READY);
//
//            sleep(300);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(6, 23, 0.8, 0.4, 25, 180, 235));
//            allPoints.add(new CurvePoint(4, 0, 0.8, 0.4, 25, 180, 180));
//            allPoints.add(new CurvePoint(0, -6, 0.8, 0.4, 25, 180, 180));
//            allPoints.add(new CurvePoint(-10, -2, 0.8, 0.4, 25, 180, 180));
//            allPoints.add(new CurvePoint(-20, -2, 0.8, 0.4, 25, 180, 180));
//            allPoints.add(new CurvePoint(-43, 3, 0.8, 0.4, 25, 180, 180));
//            allPoints.add(new CurvePoint(-78, 3, 0.8, 0.4, 25, 180, 180));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm_V2(allPoints, 0, 15, 180, 6, ArmShoulderPositions.LEVEL3, ArmShoulderPositions.INTAKE, FingerPositions.ENTER_INTAKE, FingerPositions.INTAKE_READY,0.7, 0,"none", 0);



        }

        myPurePursuitRobotMovement6_Turn_MultiThread.stopOdometryThread();

    }

}

