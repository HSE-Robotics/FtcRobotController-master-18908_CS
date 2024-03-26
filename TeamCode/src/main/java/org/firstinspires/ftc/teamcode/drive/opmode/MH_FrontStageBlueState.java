/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Blue Front Stage State", group = "Concept")

public class MH_FrontStageBlueState extends LinearOpMode {
    private DcMotor BR;
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BL;
    private DcMotor ArmRotator;
    private Servo RServo;
    private Servo LServo;
    private Servo Wrist;
    private DcMotor SliderMotor;
    private DcMotor Rightslider;
    int Initial_Position;
    ElapsedTime elapsedTime;
    double ticks_per_revolution;
    double wheel_circumference;
    double ticks_per_inch;
    int ticks_to_destination;
    int distance_to_travel;
    double speed;
    int initialPosition;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    //private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/BlueCone.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "azul cone",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {


        initTfod();

        int timeBack;


        BR = hardwareMap.get(DcMotor.class, "BR");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        ArmRotator = hardwareMap.get(DcMotor.class, "ArmRotator");
        RServo = hardwareMap.get(Servo.class, "RServo");
        LServo = hardwareMap.get(Servo.class, "LServo");
        LServo.setDirection(Servo.Direction.REVERSE);
        RServo.setPosition(0.75);
        LServo.setPosition(0.75);
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        SliderMotor = hardwareMap.get(DcMotor.class,"SliderMotor");
        //Rightslider = hardwareMap.get(DcMotor.class, "Rightslider");

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        Wrist.setPosition(0.75);

        waitForStart();
        elapsedTime = new ElapsedTime();
        wheel_circumference = 11.775;
        ticks_per_revolution = 528;
        ticks_per_inch = ticks_per_revolution / wheel_circumference;
        distance_to_travel = 60;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);

        if (opModeIsActive() && !isStopRequested()) {
            while (opModeIsActive() && !isStopRequested()) {


                initialPosition = ArmRotator.getCurrentPosition();

                ArmRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ArmRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ArmRotator.setDirection(DcMotor.Direction.FORWARD);

                SliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                SliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                SliderMotor.setDirection(DcMotor.Direction.REVERSE);



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

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Wrist.setPosition(0.25);

        wheel_circumference = 11.775;
        ticks_per_revolution = 528;
        ticks_per_inch = ticks_per_revolution / wheel_circumference;
        distance_to_travel = 60;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        speed = 0.25;
        boolean objDetected = false;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {

            String cubePosition = "";
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            double w = recognition.getWidth() ;
            double h = recognition.getHeight() ;
            double startingX, startingY, stackX, stackY;
            double initHeading = Math.PI * 1.5;

            double currElTime = elapsedTime.seconds();

            startingX = 16;
            startingY = 64.00;
            /*
            Pose2d startingPose = new Pose2d(startingX, startingY, initHeading);
            drive.setPoseEstimate(startingPose);
            */

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addData("- PositionCube", "%s", cubePosition);
            int initialPositionForeArmMotor, sliderInitialPosition, scoringPositionSlider, scoringPositionArmRotator;
            double armSpeed, sliderSpeed, x_score, y_score;
            armSpeed = 1.0;
            sliderSpeed = 0.75;
            initialPositionForeArmMotor = ArmRotator.getCurrentPosition();
            sliderInitialPosition = SliderMotor.getCurrentPosition();
            scoringPositionSlider = 3400;

            scoringPositionArmRotator = 2700;
            if((x>450 && x<675) && (y>90 && y < 300) && (w>50 && w<120) && (h>50 && h<130)){
                cubePosition = "Right";
            }else if((x>0 && x<120) && (y>80 && y < 300) && (w>50 && w<120) && (h>50 && h<130)){
                cubePosition = "Left";
            }else if((x>150 && x<360) && (y>100 && y < 260) && (w>50 && w<120) && (h>50 && h<130)){
                cubePosition = "Center";
            }else if(currElTime > 12.00){
                cubePosition = "Center";
            }

            switch(cubePosition){

                case "Center":
                    objDetected = true;
                    RServo.setPosition(0.75);
                    LServo.setPosition(0.75);
                    sleep(150);
                    ArmRotator.setTargetPosition(initialPositionForeArmMotor + 100);
                    ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmRotator.setPower(armSpeed);
                    cubePosition = "Center";

                    telemetry.addData("Target Position:", cubePosition);
                    telemetry.update();
                    x_score = 35.50;
                    y_score = 42.00;

                    TrajectorySequence PurplePixelCenter = drive.trajectorySequenceBuilder(new Pose2d(12.00, 62.00, Math.toRadians(270.00)))
                            .lineTo(new Vector2d(12.00, 34.00))
                            .addDisplacementMarker(() -> {
                                RServo.setPosition(0.25);
                            })
                            .lineTo(new Vector2d(12.00, 45.00))
                            .lineToLinearHeading(new Pose2d(36.00, 42.00, Math.toRadians(0.00)))
                            .build();
                    drive.setPoseEstimate(PurplePixelCenter.start());
                    drive.followTrajectorySequence(PurplePixelCenter);
                    sleep(500);

                    sleep(1000000);
                    break;
                case "Left":
                    objDetected = true;
                    RServo.setPosition(0.75);
                    LServo.setPosition(0.75);
                    sleep(150);
                    ArmRotator.setTargetPosition(initialPositionForeArmMotor + 100);
                    ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmRotator.setPower(armSpeed);
                    cubePosition = "Left";

                    telemetry.addData("Target Position:", cubePosition);
                    telemetry.update();
                    TrajectorySequence PurplePixelLeft = drive.trajectorySequenceBuilder(new Pose2d(12.00, 62.00, Math.toRadians(270.00)))
                            .lineTo(new Vector2d(12.00, 35.00))
                            .addDisplacementMarker(() -> {
                                RServo.setPosition(0.25);
                            })
                            .lineTo(new Vector2d(10.00, 32.00))
                            .lineToLinearHeading(new Pose2d(10.00, 32.00, Math.toRadians(0.00)))
                            .build();
                    drive.setPoseEstimate(PurplePixelLeft.start());
                    drive.followTrajectorySequence(PurplePixelLeft);
                    sleep(500);

                    sleep(1000000);
                    break;

                case "Right":
                    objDetected = true;
                    RServo.setPosition(0.75);
                    LServo.setPosition(0.75);
                    sleep(150);
                    ArmRotator.setTargetPosition(initialPositionForeArmMotor + 100);
                    ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmRotator.setPower(armSpeed);
                    cubePosition = "Right";

                    telemetry.addData("Target Position:", cubePosition);
                    telemetry.update();
                    // scoringPositionSlider = 3000;


                    TrajectorySequence PurplePixel = drive.trajectorySequenceBuilder(new Pose2d(12.00, 62.00, Math.toRadians(270.00)))
                            .splineToLinearHeading(new Pose2d(12.00, 30.00, Math.toRadians(180.00)), Math.toRadians(180))
                            .lineTo(new Vector2d(0.00,30.00))
                            .build();
                    drive.setPoseEstimate(PurplePixel.start());
                    drive.followTrajectorySequence(PurplePixel);

                    sleep(500);
                    RServo.setPosition(0.25);

                    SliderMotor.setTargetPosition(sliderInitialPosition);
                    SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SliderMotor.setPower(sliderSpeed);

                    while(SliderMotor.isBusy()){
                        idle();
                    }



                    sleep(500);

                    ArmRotator.setTargetPosition(initialPositionForeArmMotor);
                    ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmRotator.setPower(armSpeed);
                    while(ArmRotator.isBusy() ){
                        idle();
                    }
                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position", "%.0f / %.0f", x, y);
                    telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                    telemetry.addData("- PositionCube", "%s", cubePosition);
                    telemetry.update();
                    sleep(5000);
                    requestOpModeStop();
                    sleep(1000000);
                    break;


            }
            /*
            if((x > 150 && x < 360) && (y>100 && y < 260)){

                objDetected = true;
                RServo.setPosition(0.75);
                LServo.setPosition(0.75);
                sleep(150);
                ArmRotator.setTargetPosition(initialPositionForeArmMotor + 100);
                ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmRotator.setPower(armSpeed);
                cubePosition = "Center";

                telemetry.addData("Target Position:", cubePosition);
                telemetry.update();
                x_score = 35.50;
                y_score = 42.00;

                TrajectorySequence PurplePixel = drive.trajectorySequenceBuilder(new Pose2d(12.00, 62.00, Math.toRadians(270.00)))
                        .lineTo(new Vector2d(12.00, 36.00))
                        .addDisplacementMarker(() -> {
                            RServo.setPosition(0.25);
                            ArmRotator.setTargetPosition(initialPositionForeArmMotor + 300);
                            ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            ArmRotator.setPower(armSpeed);
                            */
            /*while(ArmRotator.isBusy() ){
                                idle();
                            }*//*
                            Wrist.setPosition(0.75);
                        })
                        .lineTo(new Vector2d(12.00, 45.00))
                        .lineToLinearHeading(new Pose2d(36.00, 42.00, Math.toRadians(0.00)))
                        .addSpatialMarker(new Vector2d(30.00, 42.00),()->{
                            ArmRotator.setTargetPosition(initialPositionForeArmMotor + scoringPositionArmRotator);
                            ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            ArmRotator.setPower(armSpeed);
                            *//*while(ArmRotator.isBusy() ){
                                idle();
                            }*//*

                            SliderMotor.setTargetPosition(sliderInitialPosition + scoringPositionSlider);
                            SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            SliderMotor.setPower(sliderSpeed);
                            *//*while(SliderMotor.isBusy()){
                                idle();
                            }*//*
                            sleep(250);
                            Wrist.setPosition(0.0);
                        })
                        .build();
                drive.setPoseEstimate(PurplePixel.start());
                drive.followTrajectorySequence(PurplePixel);
                sleep(500);

                //Drop the Yellow Pixel
                TrajectorySequence YellowPixel = drive.trajectorySequenceBuilder(new Pose2d(36.00, 42.00, Math.toRadians(0.00)))
                        .lineTo(new Vector2d(45.00, 32.00))
                        .build();
                drive.setPoseEstimate(YellowPixel.start());
                drive.followTrajectorySequence(YellowPixel);
                sleep(250);
                LServo.setPosition(0.5);
                sleep(250);
                Wrist.setPosition(0.75);

                TrajectorySequence StartParking = drive.trajectorySequenceBuilder(new Pose2d(45.00, 34.00, Math.toRadians(0.00)))
                        .lineTo(new Vector2d(33.00, 42.00))
                        .build();
                drive.setPoseEstimate(StartParking.start());
                drive.followTrajectorySequence(StartParking);
                SliderMotor.setTargetPosition(sliderInitialPosition);
                SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderMotor.setPower(sliderSpeed);

                while(SliderMotor.isBusy()){
                    idle();
                }

                sleep(500);
                TrajectorySequence Parking = drive.trajectorySequenceBuilder(new Pose2d(33.00, 42.00, Math.toRadians(0.00)))
                        .lineTo(new Vector2d(33.00, 55.00))
                        .lineTo(new Vector2d(44.00, 60.00))
                        .build();
                drive.setPoseEstimate(Parking.start());
                drive.followTrajectorySequence(Parking);


                sleep(1500);

                ArmRotator.setTargetPosition(initialPositionForeArmMotor);
                ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmRotator.setPower(armSpeed);
                while(ArmRotator.isBusy() ){
                    idle();
                }

                sleep(1000000);
                break;

            }else if((x > 0 && x < 120) && (y>80 && y < 300)){

                objDetected = true;
                RServo.setPosition(0.75);
                LServo.setPosition(0.75);
                sleep(150);
                ArmRotator.setTargetPosition(initialPositionForeArmMotor + 100);
                ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmRotator.setPower(armSpeed);
                cubePosition = "Left";

                telemetry.addData("Target Position:", cubePosition);
                telemetry.update();
                TrajectorySequence PurplePixel = drive.trajectorySequenceBuilder(new Pose2d(12.00, 62.00, Math.toRadians(270.00)))
                        .lineTo(new Vector2d(12.00, 35.00))
                        .waitSeconds(0.250)
                        .lineToLinearHeading(new Pose2d(10.00, 32.00, Math.toRadians(0.00)))
                        .build();
                drive.setPoseEstimate(PurplePixel.start());
                drive.followTrajectorySequence(PurplePixel);

                sleep(1000);
                RServo.setPosition(0.25);
                ArmRotator.setTargetPosition(initialPositionForeArmMotor + 300);
                ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmRotator.setPower(armSpeed);
                while(ArmRotator.isBusy() ){
                    idle();
                }
                Wrist.setPosition(0.75);

                TrajectorySequence YellowPixel = drive.trajectorySequenceBuilder(new Pose2d(10.00,32.00,Math.toRadians(0.00)))
                        .lineTo(new Vector2d(9.00, 46.00))
                        .waitSeconds(0.250)
                        .lineTo(new Vector2d(39.00, 38.00))
                        .addSpatialMarker(new Vector2d(18.00, 44.00),()->{
                            ArmRotator.setTargetPosition(initialPositionForeArmMotor + scoringPositionArmRotator);
                            ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            ArmRotator.setPower(armSpeed);
                            *//*while(ArmRotator.isBusy() ){
                                idle();
                            }*//*

                            SliderMotor.setTargetPosition(sliderInitialPosition + scoringPositionSlider);
                            SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            SliderMotor.setPower(sliderSpeed);
                            *//*while(SliderMotor.isBusy()){
                                idle();
                            }*//*
                            Wrist.setPosition(0.0);
                        })
                        .build();
                drive.setPoseEstimate(YellowPixel.start());
                drive.followTrajectorySequence(YellowPixel);
                sleep(500);
                LServo.setPosition(0.5);
                sleep(250);
                Wrist.setPosition(0.75);


                SliderMotor.setTargetPosition(sliderInitialPosition);
                SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderMotor.setPower(sliderSpeed);

                while(SliderMotor.isBusy()){
                    idle();
                }

                TrajectorySequence Parking = drive.trajectorySequenceBuilder(new Pose2d(37.00, 41.00, Math.toRadians(0.00)))
                        .lineTo(new Vector2d(33.00, 55.00))
                        .lineTo(new Vector2d(38.00, 60.00))
                        .build();
                drive.setPoseEstimate(Parking.start());
                drive.followTrajectorySequence(Parking);


                sleep(500);

                ArmRotator.setTargetPosition(initialPositionForeArmMotor);
                ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmRotator.setPower(armSpeed);
                sleep(1000000);
                break;

            }else if((x > 450 && x < 675) && (y>90 && y < 300)){

                objDetected = true;
                RServo.setPosition(0.75);
                LServo.setPosition(0.75);
                sleep(150);
                ArmRotator.setTargetPosition(initialPositionForeArmMotor + 100);
                ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmRotator.setPower(armSpeed);
                cubePosition = "Right";

                telemetry.addData("Target Position:", cubePosition);
                telemetry.update();
               // scoringPositionSlider = 3000;


                TrajectorySequence PurplePixel = drive.trajectorySequenceBuilder(new Pose2d(12.00, 62.00, Math.toRadians(270.00)))
                        .splineToLinearHeading(new Pose2d(12.00, 30.00, Math.toRadians(180.00)), Math.toRadians(180))
                        .lineTo(new Vector2d(0.00,30.00))
                        .build();
                drive.setPoseEstimate(PurplePixel.start());
                drive.followTrajectorySequence(PurplePixel);

                sleep(500);
                RServo.setPosition(0.25);
                ArmRotator.setTargetPosition(initialPositionForeArmMotor + 300);
                ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmRotator.setPower(armSpeed);
                while(ArmRotator.isBusy() ){
                    idle();
                }
                Wrist.setPosition(0.75);

                TrajectorySequence YellowPixel = drive.trajectorySequenceBuilder(new Pose2d(6.00, 30.00, Math.toRadians(180.00)))
                        .lineTo(new Vector2d(16.00, 30.00))
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(35.00, 41.00, Math.toRadians(0.00)))
                        .addSpatialMarker(new Vector2d(35.00, 41.00),()->{
                            ArmRotator.setTargetPosition(initialPositionForeArmMotor + scoringPositionArmRotator);
                            ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            ArmRotator.setPower(armSpeed);
                           *//* while(ArmRotator.isBusy() ){
                                idle();
                            }*//*

                            SliderMotor.setTargetPosition(sliderInitialPosition + 2900);
                            SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            SliderMotor.setPower(sliderSpeed);
                           *//* while(SliderMotor.isBusy()){
                                idle();
                            }*//*
                            sleep(350);
                            Wrist.setPosition(0.0);
                        })
                        .lineTo(new Vector2d(41.00, 32.00))
                        .build();
                drive.setPoseEstimate(YellowPixel.start());
                drive.followTrajectorySequence(YellowPixel);

                sleep(500);
                LServo.setPosition(0.5);
                sleep(250);
                Wrist.setPosition(0.75);

                sleep(500);
                SliderMotor.setTargetPosition(sliderInitialPosition);
                SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderMotor.setPower(sliderSpeed);

                while(SliderMotor.isBusy()){
                    idle();
                }

                TrajectorySequence Parking = drive.trajectorySequenceBuilder(new Pose2d(41.00, 32.00, Math.toRadians(0.00)))
                        .lineTo(new Vector2d(38.00, 55.00))
                        .lineTo(new Vector2d(48.00, 58.00))
                        .build();
                drive.setPoseEstimate(Parking.start());
                drive.followTrajectorySequence(Parking);


                sleep(500);

                ArmRotator.setTargetPosition(initialPositionForeArmMotor);
                ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmRotator.setPower(armSpeed);
                while(ArmRotator.isBusy() ){
                    idle();
                }
                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                telemetry.addData("- PositionCube", "%s", cubePosition);
                telemetry.update();
                sleep(1000000);
                break;
            }
*/



        }   // end for() loop

    }   // end method telemetryTfod()

    private void stopR(){
        BR.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        FL.setPower(0.0);
    }
    private void stopArm(){
        ArmRotator.setPower(0.0);
    }
    private void armToScore(){
        ArmRotator.setTargetPosition(ArmRotator.getCurrentPosition() + 600);
        ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmRotator.setPower(1);
    }

    private void armDown(int initialPosition){
        ArmRotator.setTargetPosition(initialPosition);
        ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmRotator.setPower(1);
        while (ArmRotator.isBusy()) {

        }
        ArmRotator.setPower(0.0);
    }
    /**
     * Drive Backwards / Forward
     */
    private void drive(double Power, double distance_to_travel, double ticks_per_inch) {
        int ticks_to_destination;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        BL.setTargetPosition(BL.getCurrentPosition() + ticks_to_destination);
        FL.setTargetPosition(FL.getCurrentPosition()+ticks_to_destination);
        FR.setTargetPosition(FR.getCurrentPosition()+ticks_to_destination);
        BR.setTargetPosition(BR.getCurrentPosition()+ticks_to_destination);
        FR.setPower(Power);
        BR.setPower(Power);
        BL.setPower(Power);
        FL.setPower(Power);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (BL.isBusy() && BR.isBusy() &&
                FL.isBusy() && FR.isBusy()) {
            //telemetry.addData("BL - ", BL.getCurrentPosition() + " - " + BL.getTargetPosition());
            //  telemetry.addData("FL", "%.0i - %.0i", FL.getCurrentPosition(), FL.getTargetPosition());
            // telemetry.addData("FR", "%.0i - %.0i", FR.getCurrentPosition(), FR.getTargetPosition());
            // telemetry.addData("BR", "%.0i - %.0i", BR.getCurrentPosition(), BR.getTargetPosition());
            // telemetry.update();
        }
    }
    private void driveBack(double Power, double distance_to_travel, double ticks_per_inch) {
        int ticks_to_destination;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        BL.setTargetPosition(BL.getCurrentPosition() - ticks_to_destination);
        FL.setTargetPosition(FL.getCurrentPosition()-ticks_to_destination);
        FR.setTargetPosition(FR.getCurrentPosition()-ticks_to_destination);
        BR.setTargetPosition(BR.getCurrentPosition()-ticks_to_destination);
        FR.setPower(Power);
        BR.setPower(Power);
        BL.setPower(Power);
        FL.setPower(Power);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (BL.isBusy() && BR.isBusy() &&
                FL.isBusy() && FR.isBusy()) {
            //telemetry.addData("BL", "%.0f - %.0f", BL.getCurrentPosition(), BL.getTargetPosition());
            //telemetry.addData("FL", "%.0f - %.0f", FL.getCurrentPosition(), FL.getTargetPosition());
            //telemetry.addData("FR", "%.0f - %.0f", FR.getCurrentPosition(), FR.getTargetPosition());
            //telemetry.addData("BR", "%.0f - %.0f", BR.getCurrentPosition(), BR.getTargetPosition());

        }
    }
    private void rotateLeft(double Power, double distance_to_travel, double ticks_per_inch) {
        int ticks_to_destination;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        BL.setTargetPosition(BL.getCurrentPosition() - ticks_to_destination);
        FL.setTargetPosition(FL.getCurrentPosition()-ticks_to_destination);
        FR.setTargetPosition(FR.getCurrentPosition()+ticks_to_destination);
        BR.setTargetPosition(BR.getCurrentPosition()+ticks_to_destination);
        FR.setPower(Power);
        BR.setPower(Power);
        BL.setPower(Power);
        FL.setPower(Power);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (BL.isBusy() && BR.isBusy() &&
                FL.isBusy() && FR.isBusy()) {

        }
    }
    private void rotateRight(double Power, double distance_to_travel, double ticks_per_inch) {
        int ticks_to_destination;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        BL.setTargetPosition(BL.getCurrentPosition() + ticks_to_destination);
        FL.setTargetPosition(FL.getCurrentPosition()+ticks_to_destination);
        FR.setTargetPosition(FR.getCurrentPosition()-ticks_to_destination);
        BR.setTargetPosition(BR.getCurrentPosition()-ticks_to_destination);
        FR.setPower(Power);
        BR.setPower(Power);
        BL.setPower(Power);
        FL.setPower(Power);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (BL.isBusy() && BR.isBusy() &&
                FL.isBusy() && FR.isBusy()) {

        }
    }
    /**
     * Strafe Right
     */
    private void strafe_left(double Power, double distance_to_travel, double ticks_per_inch) {
        int ticks_to_destination;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        BL.setTargetPosition(BL.getCurrentPosition() + ticks_to_destination);
        FL.setTargetPosition(FL.getCurrentPosition() - ticks_to_destination);
        FR.setTargetPosition(FR.getCurrentPosition() + ticks_to_destination);
        BR.setTargetPosition(BR.getCurrentPosition() - ticks_to_destination);
        FR.setPower(Power);
        BR.setPower(Power);
        BL.setPower(Power);
        FL.setPower(Power);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (BL.isBusy() && BR.isBusy() &&
                FL.isBusy() && FR.isBusy()) {

        }
    }
    private void strafe_right(double Power, double distance_to_travel, double ticks_per_inch) {
        int ticks_to_destination;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        BL.setTargetPosition(BL.getCurrentPosition() - ticks_to_destination);
        FL.setTargetPosition(FL.getCurrentPosition() + ticks_to_destination);
        FR.setTargetPosition(FR.getCurrentPosition() - ticks_to_destination);
        BR.setTargetPosition(BR.getCurrentPosition() + ticks_to_destination);
        FR.setPower(Power);
        BR.setPower(Power);
        BL.setPower(Power);
        FL.setPower(Power);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (BL.isBusy() && BR.isBusy() &&
                FL.isBusy() && FR.isBusy()) {

        }
    }
}   // end class
