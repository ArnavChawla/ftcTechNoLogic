/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="test", group ="Concept")

public class ConceptVuMarkIdentification_test_OpMode extends CommonDriverFunctions {



    /**a
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    public void runCorner1()
    {

    }

    @Override public void runOpMode() {
        robot.init(hardwareMap);

        //super.runOpMode();


        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        //telemetry.addData("VuMark", "%s visible", vuMark);

        //while (opModeIsActive())
        // {
        //

        goStraight(10);
        telemetry.addData("Run complete", "");
        telemetry.update();

        //}
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }



	public void initArm()
	{
	    //initilize the arm
        robot.shoulder.setPosition(0);
        robot.elbow.setPosition(0);
        robot.wrist.setPosition(0.5);
	}

	 public void retractArm()
    {

        double shoulderAngle = (100* 0.005);
        double elbowAngle = (100* 0.009);

        for(int i = 0; i <100; i++)
        {
            //shoulder angle - 0.005, elbow 0.01
            shoulderAngle = shoulderAngle - 0.005;
            elbowAngle = elbowAngle - 0.009;
            robot.elbow.setPosition(elbowAngle);
            robot.shoulder.setPosition(shoulderAngle);
            if(i==50)//half way
            {
                robot.wrist.setPosition(0.5);
            }
        }
    }

    public void extendArm()
    {

        double shoulderAngle =0;
        double elbowAngle = 0;
        for(int i = 0; i <100; i++)
        {
            //shoulder angle - 0.005, elbow 0.01
            shoulderAngle = shoulderAngle + 0.005;
            elbowAngle = elbowAngle + 0.009;
            robot.elbow.setPosition(elbowAngle);
            robot.shoulder.setPosition(shoulderAngle);
        }
    }

    public void ThrowJewelRedTile()
    {
        if(isRedColorLeft() == true)
        {
            //see red on left
            //move to right to push blue ball
            for(int i = 0; i <= 10; i++)
            {
                robot.wrist.setPosition(0.5 + 0.05*i);
            }
        }
        else
        {
            //see blue on left
            //move left to push blue ball
            for(int i = 0; i <= 10; i++)
            {
                robot.wrist.setPosition(0.5 - 0.05*i);
            }
        }

        //robot.wrist.setPosition(0.5);//bring to center
    }


    public boolean isRedColorLeft()
    {

        double wristPosition = 0.5;
        while( (robot.jewelSensor.red()< 4 ) && (robot.jewelSensor.blue()< 4) && (wristPosition > 0.25)) {
            wristPosition = wristPosition - 0.005;
            robot.wrist.setPosition(wristPosition);
            
			telemetry.addData("Red  ", robot.jewelSensor.red());
            telemetry.addData("Blue ", robot.jewelSensor.blue());
            telemetry.addData("shoulder angle ", robot.shoulder.getPosition());
            telemetry.addData("elbow angle ", robot.elbow.getPosition());
            telemetry.addData("wrist angle ", robot.wrist.getPosition());
            telemetry.update();
        }

       //see color
        if(robot.jewelSensor.red()>robot.jewelSensor.blue())
        {
            return true;
        }
        else
        {
            return false;
        }
    }



}
