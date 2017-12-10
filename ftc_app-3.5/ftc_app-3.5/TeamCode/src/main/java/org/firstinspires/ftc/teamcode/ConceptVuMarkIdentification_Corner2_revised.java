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

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

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

@Autonomous(name="Corner2_R", group ="Concept")

public class ConceptVuMarkIdentification_Corner2_revised extends CommonDriverFunctions {



    public void gotoCryptoboxColumnAndDropGlyphCorner2(RelicRecoveryVuMark cryptoColumn)
    {
        double variableDistance = 0;
        if(cryptoColumn == RelicRecoveryVuMark.LEFT)
        {
            variableDistance = 22.5;

        }
        else if (cryptoColumn == RelicRecoveryVuMark.RIGHT)
        {
            variableDistance = 7.5;
        }
        else
        {
            // whether center is recognized or whether vumark is not recognized, we go to the center
            // and drop the relic there
            variableDistance = 15;
        }

        goStraightInches(25);
        turnRobotInDegrees(-90);
        goStraightInches(variableDistance);
        turnRobotInDegrees(90);

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        robot.ting2();
        robot.myServo.setPosition(1);
        robot.myServo2.setPosition(0);

        goStraightInches(-6);
        robot.myServo.setPosition(0.2);
        robot.myServo2.setPosition(0.6);
        goStraightInchesTout(14,2);
        goStraightInches(-4);
    }

    @Override public void runOpMode() {
        robot.init(hardwareMap);


	    boolean bLedOn = true;
        // Set the LED in the beginning
        robot.jewelSensor.enableLed(bLedOn);
		
		initArm();
        setClawServoPositions1();


        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        setClawServoPositions2();

        robot.ting();
		extendArm();
        ThrowJewelRedTile();
        retractArm();


        RelicRecoveryVuMark vuMark = getPictograph();

        telemetry.addData("VuMark", "%s visible", vuMark);

        if (opModeIsActive()) { // this runs one single time, there is no reason to put it in a loop

            gotoCryptoboxColumnAndDropGlyphCorner2(vuMark);
            telemetry.update();
        }
    }




}
