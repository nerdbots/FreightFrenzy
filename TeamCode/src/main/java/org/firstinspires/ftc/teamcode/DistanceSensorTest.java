package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
//@Disabled
@TeleOp(name = "DistanceSensor",group = "Sensor")
public class DistanceSensorTest extends LinearOpMode {

     @Override
     public void runOpMode(){
          ColorSensor colorSensor;

          boolean blockIsIn = false;


          RevBlinkinLedDriver blinkinLedDriver;
          waitForStart();
          colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
          blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");

          while(opModeIsActive()){

               RevBlinkinLedDriver.BlinkinPattern pattern;

               Telemetry.Item patternName;
               Telemetry.Item display;
               Deadline ledCycleDeadline;
               Deadline gamepadRateLimit;

               int light = colorSensor.alpha();

               telemetry.addData("Blue", colorSensor.blue());
               telemetry.addData("Red", colorSensor.red());
               telemetry.addData("Green", colorSensor.green());
               telemetry.addData("Light", light);
               telemetry.addData("Hue", colorSensor.argb());
               telemetry.addData("BlockIsIn", blockIsIn);
               telemetry.update();


               pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
               blinkinLedDriver.setPattern(pattern);

               if(light > 200){
                    blockIsIn = true;
               }
               else if(light < 200){
                    blockIsIn = false;
               }
               if(blockIsIn == false) {

                    pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    blinkinLedDriver.setPattern(pattern);
               }

               else if(blockIsIn == true) {
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    blinkinLedDriver.setPattern(pattern);
               }

          }
     }
}
