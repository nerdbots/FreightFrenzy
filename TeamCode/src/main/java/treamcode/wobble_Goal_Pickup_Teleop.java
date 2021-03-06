/*
Copyright 2018 FIRST Tech Challenge Team [Phone] SAMSUNG

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package treamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Disabled
@TeleOp(name="Wobble_Pickup", group="Final")
public class wobble_Goal_Pickup_Teleop extends LinearOpMode {
    private DcMotor wobbleMotor;
    Servo wobbleServo;

    private ElapsedTime Timer = new ElapsedTime();


    boolean pressedOnce = false;



    @Override
    public void runOpMode() {
        wobbleMotor = hardwareMap.get(DcMotor.class, "Left");
        wobbleServo = hardwareMap.get(Servo.class, "wobble_Goal_Servo");

        Timer.reset();

        waitForStart();


        if(opModeIsActive()) {
            Timer.reset();
            while(Timer.seconds() < 0.25) {
                wobbleMotor.setPower(-0.55);
            }
            wobbleMotor.setPower(0);

            wobbleServo.setPosition(0);
        }

        while (opModeIsActive()) {


//up

        if(gamepad1.a && pressedOnce == false) {
            pressedOnce = true;

            wobbleServo.setPosition(1);

            sleep(500);

            Timer.reset();
            while(Timer.seconds() < 0.53) {
                wobbleMotor.setPower(0.9);
            }
            wobbleMotor.setPower(0);

        }
        //down
            if(gamepad1.a && pressedOnce == true) {
                pressedOnce = false;


                Timer.reset();
                while(Timer.seconds() < 0.17) {
                    wobbleMotor.setPower(-0.55);
                }

                wobbleMotor.setPower(0);
                sleep(500);
                wobbleServo.setPosition(0);
      //          sleep(500);

            }


        }



        }






}

