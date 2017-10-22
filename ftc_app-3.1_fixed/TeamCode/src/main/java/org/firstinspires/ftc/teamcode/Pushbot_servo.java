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

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop servo arnav", group="Pushbot")

public class Pushbot_servo extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot_TuesdayClass robot = new HardwarePushbot_TuesdayClass(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          currentPosition  = 0.7;
    double          currentPosition2  = 0.2 ;
    double clawOffset2 = -0.4;// Servo mid position
    final double    CLAW_SPEED  = 0.02 ;
    private ElapsedTime runtime = new ElapsedTime();

    // sets rate to move servo
//Servo myServo;

    /*
     * Code to run ONCE when the driver hits INIT
     */
        @Override
        public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
            robot.init(hardwareMap);
            robot.myServo.setPosition(1.0 );
            robot.myServo2.setPosition(0.0);

            // Send telemetry message to signify robot waiting;
            runtime.reset();
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

//        double left;
//        double right;
//
//        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
//        left = -gamepad1.left_trigger;
//        right = gamepad1.right_trigger;
//        robot.leftMotor.setPower(left);
//        robot.rightMotor.setPower(right);
        double left;
        double right;
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);

        currentPosition = Range.clip(currentPosition, 0.4, 0.8 );
            currentPosition2 = Range.clip(currentPosition2,0.2 , 0.45);
            telemetry.addData("Say", "Hello Driver");
            if (gamepad1.right_bumper) {
                robot.myServo.setPosition(currentPosition + 0.1);
                robot.myServo2.setPosition(currentPosition2 - 0.1);

                    currentPosition += 0.1;
                    currentPosition2 -= 0.1;

            }
            if (gamepad1.left_bumper) {
                robot.myServo.setPosition(currentPosition - 0.1);
                robot.myServo2.setPosition(currentPosition2 + 0.1);

                    currentPosition -= 0.1;
                    currentPosition2 += 0.1;

            }

        telemetry.addData("servo", currentPosition);
        telemetry.addData("servo2", currentPosition2);
        if(gamepad1.dpad_down)
        {
            runtime.reset();
        }
        runtime.reset();
        if(gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0)
        {
            telemetry.addData("Press only one button at a time", "abc");
            return;
        }

        if(gamepad1.right_trigger > 0)
        {
            robot.clawMotor.setPower(gamepad1.right_trigger);
        }
        if(gamepad1.left_trigger > 0)
        {
            robot.clawMotor.setPower(-gamepad1.left_trigger);
        }

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
