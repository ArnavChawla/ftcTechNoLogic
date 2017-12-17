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
 * Created by Anu Chawla on 05-Dec-17.
 */


public class CommonDriverFunctions extends LinearOpMode {
    HardwarePushbot_TuesdayClass robot = new HardwarePushbot_TuesdayClass();
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1220 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.25;
    VuforiaLocalizer vuforia;

    private ElapsedTime pictoTime = new ElapsedTime();

    @Override public void runOpMode() {
        robot.init(hardwareMap);
    }

    public void setClawServoPositions1()
    {
        robot.myServo.setPosition(0.7);
        robot.myServo2.setPosition(0.1);
    }

    public void setClawServoPositions2()
    {
        robot.myServo.setPosition(0.2);
        robot.myServo2.setPosition(0.6);
    }

    public void goStraightInches(double distance)
    {
        encoderDrive(DRIVE_SPEED, -distance/2, -distance/2, 10);
    }

    public void moveLeftMotor(double distance)
    {
        encoderDriveOneMotor(DRIVE_SPEED, -distance/2, 0, 10);
    }

    public void moveRightMotor(double distance)
    {
        encoderDriveOneMotor(DRIVE_SPEED, 0, -distance/2, 10);
    }

    public void goStraightInchesTout(double distance, double timeOut)
    {
        encoderDrive(DRIVE_SPEED, -distance/2, -distance/2, timeOut);
    }


    public void turnRobotInDegrees(double degrees)
    {
        double turnRatio = 4.8/90;

        encoderDrive(TURN_SPEED,-(turnRatio*degrees), (turnRatio*degrees), 5);
    }


    public RelicRecoveryVuMark getPictograph()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AY2KQyL/////AAAAGXS4X/lQOk/IjLKvqAMnGZwUa2bzXyB+9U0qpjzUtC75gupc1qaq33ijadEDvuneV699tFrKTLAf1n2FG39Mqjhf88N33OpPuJtyx0n41oPfecHfJUWKY2EptbsHIf/Ii0NsU4LeBd6W68KviHWJMf3I1bxyv6zqwrbB+emaFpC7loL1U+Etxby2DiT4GLRzJ5HZuhKw/Om+hgvZGC9iAsynldVLLzl40VEfVQV8RIGFm6Z+Dd/cILvSwFxZ60NpghZjEOz3Q3yM0OipQWJxEclf3gb984aOr8IbnlFtEJv4HAUfZF/t4eOu90BiXyhue6eXnxJZttd9FVtIa+m1AUvJKf4BaaZb5v0ovCXo5ABB\n";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        pictoTime.reset();

        while (pictoTime.seconds() <= 4 )
        {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if(vuMark != RelicRecoveryVuMark.UNKNOWN)
            {
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

                break;
            }
        }

        return vuMark;
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


            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(20);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(20);


            robot.leftMotor.setTargetPosition(newLeftTarget);
            sleep(20);
            robot.rightMotor.setTargetPosition(newRightTarget);
            sleep(20);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(speed);
            sleep(20);
            robot.rightMotor.setPower(speed);
            sleep(20);

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
            robot.leftMotor.setPower(0);
            sleep(20);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            //robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move
        }


    }

    public void encoderDriveOneMotor(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);


            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(20);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(20);


            robot.leftMotor.setTargetPosition(newLeftTarget);
            sleep(20);
            robot.rightMotor.setTargetPosition(newRightTarget);
            sleep(20);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(speed);
            sleep(20);
            robot.rightMotor.setPower(speed);
            sleep(20);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // N    ote: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)  &&     (robot.leftMotor.isBusy() || robot.rightMotor.isBusy()) ) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            sleep(20);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            //robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move
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
