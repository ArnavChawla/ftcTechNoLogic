package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Anu Chawla on 05-Dec-17.
 */


public class CommonDriverFunctions_sanjeev extends LinearOpMode {
    HardwarePushbot_sanjeev robot = new HardwarePushbot_sanjeev();
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

                //!!!!!!! COMMENT THIS SECTION OUT BEFORE USING
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.update();
                while(pictoTime.seconds() < 10)
                {
                    // this is a 10 second unnecessary loop to prove that the pictograph works
                }
                //!!!! COMMENT THIS HERE

                break;
            }
        }

        /** we illustrate it nevertheless, for completeness. */


        return vuMark;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }



}
