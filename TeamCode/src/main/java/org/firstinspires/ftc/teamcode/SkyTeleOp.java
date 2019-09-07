package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

@TeleOp(name="SkyTeleOp", group="Drive-Type OpModes")

public class SkyTeleOp extends OpMode {
    // Declare OpMode variables for use.
    //All servo variables are in DEGREES.
    //As a general rule, use uppercase first letters for hardware mapping,
    //and use lowercase first letters for variables.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor DriveMotor1 = null; // front right
    private DcMotor DriveMotor2 = null; // front left
    private DcMotor DriveMotor3 = null; // back right
    private DcMotor DriveMotor4 = null; // back left


    private double motorSpeed = 1;


    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DriveMotor1 = hardwareMap.get(DcMotor.class, "DriveMotor1");
        DriveMotor2 = hardwareMap.get(DcMotor.class, "DriveMotor2");
        DriveMotor3 = hardwareMap.get(DcMotor.class, "DriveMotor3");
        DriveMotor4 = hardwareMap.get(DcMotor.class, "DriveMotor4");

        telemetry.addData("Status", "Initialized again");




    }



    @Override
    public void loop() {
        //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

        // 4 Motor controller
        if (gamepad1.x);
        else if (gamepad1.left_stick_y > 0.5) {      //Forward
            DriveMotor1.setPower(1);
            DriveMotor2.setPower(-1);
            DriveMotor3.setPower(1);
            DriveMotor4.setPower(-1);
        } else if (gamepad1.left_stick_y < -0.5) {       //Backwards
            DriveMotor1.setPower(-1);
            DriveMotor2.setPower(1);
            DriveMotor3.setPower(-1);
            DriveMotor4.setPower(1);
        } else if (gamepad1.left_stick_x < -0.5) {      //Right Translation
            DriveMotor1.setPower(-1);
            DriveMotor2.setPower(-1);
            DriveMotor3.setPower(1);
            DriveMotor4.setPower(1);
        } else if (gamepad1.left_stick_x > 0.5) {       //Left Translation
            DriveMotor1.setPower(1);
            DriveMotor2.setPower(1);
            DriveMotor3.setPower(-1);
            DriveMotor4.setPower(-1);
        } else if (gamepad1.right_stick_x > 0.5) {      //Left
            DriveMotor1.setPower(-1);
            DriveMotor2.setPower(-1);
            DriveMotor3.setPower(-1);
            DriveMotor4.setPower(-1);
        } else if (gamepad1.right_stick_x < -0.5) {       //Right
            DriveMotor1.setPower(1);
            DriveMotor2.setPower(1);
            DriveMotor3.setPower(1);
            DriveMotor4.setPower(1);
        } else {    // 0 set power for the 4 train drive system
            motorSpeed = 0;
            DriveMotor1.setPower(0);
            DriveMotor2.setPower(0);
            DriveMotor3.setPower(0);
            DriveMotor4.setPower(0);

        }
        if (gamepad1.b);
        else if (gamepad1.left_stick_y > 0.5) {      //Forward
            DriveMotor1.setPower(.25);
            DriveMotor2.setPower(-.25);
            DriveMotor3.setPower(.25);
            DriveMotor4.setPower(-.25);
        } else if (gamepad1.left_stick_y < -0.5) {       //Backwards
            DriveMotor1.setPower(-.25);
            DriveMotor2.setPower(.25);
            DriveMotor3.setPower(-.25);
            DriveMotor4.setPower(.25);
        } else if (gamepad1.left_stick_x < -0.5) {      //Right Translation
            DriveMotor1.setPower(-.25);
            DriveMotor2.setPower(-.25);
            DriveMotor3.setPower(.25);
            DriveMotor4.setPower(.25);
        } else if (gamepad1.left_stick_x > 0.5) {       //Left Translation
            DriveMotor1.setPower(.25);
            DriveMotor2.setPower(.25);
            DriveMotor3.setPower(-.25);
            DriveMotor4.setPower(-.25);
        } else if (gamepad1.right_stick_x > 0.5) {      //Left
            DriveMotor1.setPower(-.25);
            DriveMotor2.setPower(-.25);
            DriveMotor3.setPower(-.25);
            DriveMotor4.setPower(-.25);
        } else if (gamepad1.right_stick_x < -0.5) {       //Right
            DriveMotor1.setPower(.25);
            DriveMotor2.setPower(.25);
            DriveMotor3.setPower(.25);
            DriveMotor4.setPower(.25);
        } else {    // 0 set power for the 4 train drive system
            motorSpeed = 0;
            DriveMotor1.setPower(0);
            DriveMotor2.setPower(0);
            DriveMotor3.setPower(0);
            DriveMotor4.setPower(0);

        }


    }
}







