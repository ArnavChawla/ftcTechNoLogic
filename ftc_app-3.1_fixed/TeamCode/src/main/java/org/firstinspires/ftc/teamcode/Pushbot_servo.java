/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwarePushbot_TuesdayClass;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
//create the teleop so that it can be used in the app
@TeleOp(name="Main teleop", group="Pushbot")

public class Pushbot_servo extends OpMode{

    /* Declare OpMode members. */
            //
            HardwarePushbot_TuesdayClass robot = new HardwarePushbot_TuesdayClass(); // use the class created to define a Pushbot's hardware
            double currentPosition = 0.7;
            double          currentPosition2  = 0.2;
            double rackPower = 0.35;//the speed at which the rack and pinion moves
            double clawOffset2 = -0.4;// Servo mid position


            @Override
            public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        //get the hardware map and set the positions to meet the constrains
            robot.init(hardwareMap);
            robot.myServo.setPosition(0.6);
            robot.myServo2.setPosition(0.3);
        }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){


        //code to move the rack and pinion
        if(gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0)
        {
            telemetry.addData("Press only one button at a time", "abc");
            return;
        }
        // if upper touch sensor triggered set power to 0 and disable upper direction
        else if (gamepad1.right_trigger == 1 && !robot.uTouchSensor.isPressed())
        {
            robot.clawMotor.setPower(rackPower);
        }
        // if lower touch sensor triggered set power to 0 and disable lower direction
        else if (gamepad1.left_trigger == 1 && !robot.dTouchSensor.isPressed())
        {
            robot.clawMotor.setPower(-rackPower);
        }
        else
        {
            robot.clawMotor.setPower(0);
        }
        //code to actually move the robot
        double left;
        double right;
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);
        //limit the amount that the servos can move clipping the range
        currentPosition = Range.clip(currentPosition, 0.4, 0.8 );
            currentPosition2 = Range.clip(currentPosition2,0.2 , 0.45);
            // if the right bumper is pressed move the servos inward to pick up the blocks
            if (gamepad1.right_bumper) {
                robot.myServo.setPosition(currentPosition + 0.1);
                robot.myServo2.setPosition(currentPosition2 - 0.1);

                    currentPosition += 0.1;
                    currentPosition2 -= 0.1;

            }
            // if the left bumper move them outward to drop the blocks
            if (gamepad1.left_bumper) {
                robot.myServo.setPosition(currentPosition - 0.1);
                robot.myServo2.setPosition(currentPosition2 + 0.1);

                currentPosition -= 0.1;
                currentPosition2 += 0.1;

            }

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
