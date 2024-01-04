package org.firstinspires.ftc.teamcode.drive.opmode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

    /*
     * This OpMode illustrates the basics of TensorFlow Object Detection, using
     * the easiest way.
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
     */


@Disabled
@Autonomous(name = "Auto Left Close Side", preselectTeleOp = "MightyHawks_RR")
public class Auto_Left_Close_Side extends LinearOpMode {

        private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

        /**
         * The variable to store our instance of the TensorFlow Object Detection processor.
         */
        private TfodProcessor tfod;
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pixel",
    };

        /**
         * The variable to store our instance of the vision portal.
         */
        private VisionPortal visionPortal;

        @Override
        public void runOpMode() {

            initTfod();
            int timeBack;
            double ticks_per_revolution;
            double wheel_circumference;
            double ticks_per_inch;
            int ticks_to_destination;
            int distance_to_travel;

            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            waitForStart();

            if (opModeIsActive()) {
                while (opModeIsActive()) {

                    telemetryTfod();

                    // Push telemetry to the Driver Station.
                    telemetry.update();

                    // Save CPU resources; can resume streaming when needed.
                    if (gamepad1.dpad_down) {
                        visionPortal.stopStreaming();
                    } else if (gamepad1.dpad_up) {
                        visionPortal.resumeStreaming();
                    }

                    // Share the CPU.
                    sleep(20);
                }
            }

            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();

        }   // end runOpMode()

        /**
         * Initialize the TensorFlow Object Detection processor.
         */
        private void initTfod() {

            // Create the TensorFlow processor the easy way.
            tfod = TfodProcessor.easyCreateWithDefaults();


            // Create the vision portal the easy way.
            if (USE_WEBCAM) {
                visionPortal = VisionPortal.easyCreateWithDefaults(
                        hardwareMap.get(WebcamName.class, "Webcam 1"),
                        tfod);
            } else {
                visionPortal = VisionPortal.easyCreateWithDefaults(
                        BuiltinCameraDirection.BACK, tfod);
            }

        }   // end method initTfod()

        /**
         * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
         */
        private void telemetryTfod() {

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }   // end for() loop

        }   // end method telemetryTfod()

}   // end class

