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
    double currentPosition2  = 0.2;

    double rackPower = 0.5;//the speed at which the rack and pinion moves
    double rack2UpPower = 0.8;

    double clawOffset2 = -0.4;// Servo mid position


    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        //get the hardware map and set the positions to meet the constrains
		robot.init(hardwareMap);
        initArm();
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
        } else if(gamepad1.right_trigger == 1)//go up button is pressed
		
		{
        if(!robot.uSensor2.isPressed())// 2nd beam can move up
        {
            telemetry. addLine("Motor two up");
            robot.clawMotor2.setPower(-rackPower);//2nd beam go up
        }
        else//2nd beam reached top
        {
           robot.clawMotor2.setPower(0);//stop 2nd beam motor
            telemetry. addLine("Motor two off");
            if(!robot.uSensor1.isPressed())//1st beam can go up
            {
                telemetry. addLine("Motor one up");

                robot.clawMotor1.setPower(-rack2UpPower);//1st beam go up

            }
            else//1st beam reached top2
            {
                robot.clawMotor1.setPower(0);//stop 1st beam motor
                telemetry. addLine("Motor one off");
            }
        }
//      motor one and motor two are switched
        //motor one needs negative power to go up
        //motor two needs negative power to go up
        //

    } else if (gamepad1.left_trigger == 1)//go down button pressed
    {
        if(!robot.dSensor1.isPressed())//1st beam can move down
        {
            telemetry. addLine("Motor one down");
            robot.clawMotor1.setPower(0.2);//1st beam go down
        }
        else//1st beam reached bottom
        {
            robot.clawMotor1.setPower(0);//stop 1st beam motor
            telemetry. addLine("Motor one off");
            if(!robot.dSensor2.isPressed())//2nd beam can go down
            {
                robot.clawMotor2.setPower(rackPower);//2nd beam go down
                telemetry. addLine("Motor two down");
            }
            else//2nd beam reached bottom
            {
                robot.clawMotor2.setPower(0);//stop 2nd beam motor
                telemetry. addLine("Motor two off");
            }
        }
    } else{

        robot.clawMotor2.setPower(0);
        robot.clawMotor1.setPower(0);
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
		currentPosition2 = Range.clip(currentPosition2, 0.2 , 0.45);
		
		// if the right bumper is pressed move the servos inward to pick up the blocks
		if (gamepad1.right_bumper) 
		{
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

    public void initArm()
    {
        //initilize the arm
        robot.shoulder.setPosition(0);
        robot.elbow.setPosition(0);
        robot.wrist.setPosition(0.5);
    }

}
