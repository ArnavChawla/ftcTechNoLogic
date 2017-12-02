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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name="Corner2", group ="Concept")

public class ConceptVuMarkIdentification_Corner2 extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";
    HardwarePushbot_TuesdayClass robot = new HardwarePushbot_TuesdayClass();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime pictoTime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1220 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.25;
    OpenGLMatrix lastLocation = null;


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {
        robot.init(hardwareMap);

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */

		boolean didRun = false;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AY2KQyL/////AAAAGXS4X/lQOk/IjLKvqAMnGZwUa2bzXyB+9U0qpjzUtC75gupc1qaq33ijadEDvuneV699tFrKTLAf1n2FG39Mqjhf88N33OpPuJtyx0n41oPfecHfJUWKY2EptbsHIf/Ii0NsU4LeBd6W68KviHWJMf3I1bxyv6zqwrbB+emaFpC7loL1U+Etxby2DiT4GLRzJ5HZuhKw/Om+hgvZGC9iAsynldVLLzl40VEfVQV8RIGFm6Z+Dd/cILvSwFxZ60NpghZjEOz3Q3yM0OipQWJxEclf3gb984aOr8IbnlFtEJv4HAUfZF/t4eOu90BiXyhue6eXnxJZttd9FVtIa+m1AUvJKf4BaaZb5v0ovCXo5ABB\n";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

	    boolean bLedOn = true;
        // Set the LED in the beginning
        robot.jewelSensor.enableLed(bLedOn);
		
		initArm();



        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();
       
	    robot.myServo.setPosition(0.2);
        robot.myServo2.setPosition(0.6);
        robot.ting();
		extendArm();
        ThrowJewelRedTile();
        retractArm();

        pictoTime.reset();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (pictoTime.seconds() <= 10 )
        {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if(vuMark != RelicRecoveryVuMark.UNKNOWN)
            {

                break;
            }
        }

        telemetry.addData("VuMark", "%s visible", vuMark);

        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */



            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }

				if(vuMark == RelicRecoveryVuMark.CENTER && !didRun)
                {

				     encoderDrive(DRIVE_SPEED, -25/2,-25/2, 10);
                     encoderDrive(TURN_SPEED,   4.8, -4.8, 4.0);
                     encoderDrive(DRIVE_SPEED, -15/2,-15/2, 10);
                     encoderDrive(TURN_SPEED,   -4.8, 4.8, 4.0);
                     encoderDrive(DRIVE_SPEED, -5/2,-5/2, 10);
                     
		             robot.leftMotor.setPower(0);
                     robot.rightMotor.setPower(0);
                    robot.ting2();
					 robot.myServo.setPosition(1);
                     robot.myServo2.setPosition(0);

                     encoderDrive(0.3, -8/2, -8/2, 10);
                     robot.rightMotor.setPower(0);
                     robot.leftMotor.setPower(0);
                    
					 didRun = true;
                }
                else if(vuMark == RelicRecoveryVuMark.LEFT && !didRun)
                {

			         encoderDrive(DRIVE_SPEED, -25/2,-25/2, 10);
                     encoderDrive(TURN_SPEED,   4.8, -4.8, 4.0);
                     encoderDrive(DRIVE_SPEED, ((-15/2) -3.75),((-15/2) -3.75), 10);
                     encoderDrive(TURN_SPEED,   -4.8, 4.8, 4.0);
                     encoderDrive(DRIVE_SPEED, -5/2,-5/2, 10);

                     robot.rightMotor.setPower(0);
                     robot.leftMotor.setPower(0);
                    robot.ting2();
                     robot.myServo.setPosition(1);
                     robot.myServo2.setPosition(0);
                    
					 encoderDrive(0.3, -8/2, -8/2, 10);
                     robot.rightMotor.setPower(0);
                     robot.leftMotor.setPower(0);
                    
					 didRun = true;
                }
                else  if (vuMark == RelicRecoveryVuMark.RIGHT && !didRun)
                {

				    encoderDrive(DRIVE_SPEED, -25/2,-25/2, 10);
                    encoderDrive(TURN_SPEED,   4.8, -4.8, 4.0);
                    encoderDrive(DRIVE_SPEED, ((-15/2) + 3.75),((-15/2) + 3.75), 10);
                    encoderDrive(TURN_SPEED,   -4.8, 4.8, 4.0);
                    encoderDrive(DRIVE_SPEED, -5/2,-5/2, 10);
                    
					robot.rightMotor.setPower(0);
                    robot.leftMotor.setPower(0);
                    robot.ting2();
                    robot.myServo.setPosition(1);
                    robot.myServo2.setPosition(0);

                    encoderDrive(0.3, -8/2, -8/2, 10);
                    robot.rightMotor.setPower(0);
                    robot.leftMotor.setPower(0);

                    didRun = true;
                }



            }
            else 
			{
			    if(!didRun)
                {

				     encoderDrive(DRIVE_SPEED, -25/2,-25/2, 10);
                     encoderDrive(TURN_SPEED,   4.8, -4.8, 4.0);
                     encoderDrive(DRIVE_SPEED, -15/2,-15/2, 10);
                     encoderDrive(TURN_SPEED,   -4.8, 4.8, 4.0);
                     encoderDrive(DRIVE_SPEED, -5/2,-5/2, 10);
                     
		             robot.leftMotor.setPower(0);
                     robot.rightMotor.setPower(0);
                    robot.ting2();
					 robot.myServo.setPosition(1);
                     robot.myServo2.setPosition(0);

                     encoderDrive(0.3, -8/2, -8/2, 10);
                     robot.rightMotor.setPower(0);
                     robot.leftMotor.setPower(0);
                    
					 didRun = true;
                }
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // N    ote: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)  &&     (robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) ) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);


                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }



            // Stop all motion;

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
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

        double shoulderAngle = 0;
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

    public void ThrowJewelBlueTile()
    {
        if(isRedColorLeft() == false)
        {
            //see blue on left
            //move to right to push red ball
            for(int i = 0; i <= 10; i++)
            {
                robot.wrist.setPosition(0.5 + 0.05*i);
            }
        }
        else
        {
            //see red on left
            //move left to push red ball
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
        while( (robot.jewelSensor.red()< 4 ) && (robot.jewelSensor.blue()< 4) && (wristPosition > 0.25) ) {
            wristPosition = wristPosition - 0.005;
            robot.wrist.setPosition(wristPosition);
            
			/*telemetry.addData("Red  ", robot.jewelSensor.red());
            telemetry.addData("Blue ", robot.jewelSensor.blue());
            telemetry.addData("shoulder angle ", robot.shoulder.getPosition());
            telemetry.addData("elbow angle ", robot.elbow.getPosition());
            telemetry.addData("wrist angle ", robot.wrist.getPosition());
            telemetry.update();*/
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
