/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class AutonIntakeBlockUntilDetection extends LinearOpMode
{
    private DcMotor frontRightMotor;
    private DcMotor rearRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor rearLeftMotor;

    private DcMotor intakeMotor;

    private boolean isBlockIn = false;
    ColorSensor colorSensor;
    RevBlinkinLedDriver blinkinLedDriver;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    @Override
    public void runOpMode()
    {
        rearLeftMotor = hardwareMap.get(DcMotor.class, "Rear_Left_Motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "Rear_Right_Motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        waitForStart();
        timer.reset();
        boolean freightDetected = false;

        while(opModeIsActive() && isBlockIn == false && !isStopRequested()) {
            timer.reset();
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            while(!(colorSensor.alpha() > 200) && timer.seconds() <= 2) {
                rearRightMotor.setPower(0.35);
                frontLeftMotor.setPower(-0.35);
                rearLeftMotor.setPower(-0.35);
                frontRightMotor.setPower(0.35);
                intakeMotor.setPower(0.5);
                telemetry.addData("timer 1", timer.seconds());
                telemetry.update();

            }
            if(colorSensor.alpha() > 200){  blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN); }
            timer2.reset();
            while(timer2.seconds() < 0.5) {
                rearRightMotor.setPower(-0.35);
                frontLeftMotor.setPower(0.35);
                rearLeftMotor.setPower(0.35);
                frontRightMotor.setPower(-0.35);
                intakeMotor.setPower(0);
                telemetry.addData("timer 2", timer2.seconds());
                telemetry.update();
            }
            rearRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            rearLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            if(colorSensor.alpha() > 200) {
                intakeMotor.setPower(-0.5);
            }
            if(colorSensor.alpha() > 200)  { isBlockIn = true; }

        }
    }

}
