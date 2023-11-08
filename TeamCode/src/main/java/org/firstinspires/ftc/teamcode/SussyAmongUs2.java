package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "SussyAmongUs2")
public class SussyAmongUs2 extends LinearOpMode {

    private Servo _1;
    private Servo _2;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor slidwr;
    private Servo gripper;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int slidwrSpeed;
        double speed;
        int initialPotition;
        double y;
        double x;
        double rx;
        double denominator;

        //_1 = hardwareMap.get(Servo.class, "1");
        //_2 = hardwareMap.get(Servo.class, "2");

        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        slidwr = hardwareMap.get(DcMotor.class, "slidwr");
        gripper = hardwareMap.get(Servo.class, "gripper");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // gripper_close = true;
            slidwrSpeed = 1;
            speed = 0.6;

            //_1.setDirection(Servo.Direction.REVERSE);
            //_2.setDirection(Servo.Direction.FORWARD);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slidwr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slidwr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Reverse the right side motors

            rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

            gripper.setDirection(Servo.Direction.REVERSE);
            initialPotition = slidwr.getCurrentPosition() + 85;
            slidwr.setDirection(DcMotorSimple.Direction.FORWARD);
            slidwr.setTargetPosition(initialPotition);
            slidwr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidwr.setPower(slidwrSpeed);
            gripper.scaleRange(0.5, 1);
            while (opModeIsActive()) {
                if (gamepad1.right_bumper) {
                    gripper.setPosition(0.99);
                    //_1.setPosition(1);
                    //_2.setPosition(1);
                }
                if (gamepad1.left_bumper) {
                    gripper.setPosition(0);
                    // _1.setPosition(0);
                    //_2.setPosition(0);
                }

                // Remember, this is reversed!
                y = -gamepad1.left_stick_y * speed;
                x = gamepad1.left_stick_x * speed * 1.1;
                // Counteract imperfect strafing
                rx = gamepad1.right_stick_x * speed;

                // Denominator is the largest motor power
                // (absolute value) or 1
                // This ensures all the powers mantain
                // the same ratio, but only when at least one is
                // out of the range [-
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                //denominator = JavaUtil.averageOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;
                // Make sure your ID's match your configuration
                rightFront.setPower(frontRightPower);
                rightRear.setPower(backRightPower);
                leftFront.setPower(frontLeftPower);
                leftRear.setPower(backLeftPower);


                if (gamepad1.a) {
                    slidwr.setTargetPosition(initialPotition + 1850);
                    slidwr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidwr.setPower(slidwrSpeed);
                } else if (gamepad1.x) {
                    slidwr.setTargetPosition(initialPotition + 2900);
                    slidwr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidwr.setPower(slidwrSpeed);
                } else if (gamepad1.y) {
                    slidwr.setTargetPosition(initialPotition + 4000);
                    slidwr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidwr.setPower(slidwrSpeed);
                } else if (gamepad1.b) {
                    slidwr.setTargetPosition(initialPotition);
                    slidwr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidwr.setPower(slidwrSpeed);
                }
                if (gamepad1.start) {
                    initialPotition = slidwr.getCurrentPosition();
                }
                if (gamepad1.dpad_up) {
                    slidwr.setTargetPosition(slidwr.getCurrentPosition() + 100);
                    slidwr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidwr.setPower(slidwrSpeed);
                }
                if (gamepad1.dpad_down) {
                    slidwr.setTargetPosition(slidwr.getCurrentPosition() - 100);
                    slidwr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidwr.setPower(slidwrSpeed);
                }
                telemetry.addData("gripper", gripper.getPosition());
                telemetry.addData("gripper ", gripper.getPosition());
                telemetry.addData("slider position", slidwr.getCurrentPosition());
                telemetry.addData("starting Pos.", initialPotition);
                telemetry.update();
            }
        }
    }
}