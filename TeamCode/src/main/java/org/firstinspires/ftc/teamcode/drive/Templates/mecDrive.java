package org.firstinspires.ftc.teamcode.drive.Templates;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.DualPad;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@TeleOp
public class mecDrive extends LinearOpMode
{
    //Create Robot Hardware Object
    RobotHardware robot = new RobotHardware();
    DualPad gpad = new DualPad();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize Hardware and reverse motors
        telemetry.addData("Status", "Initialized");

        waitForStart();

        while(opModeIsActive()) {
            gpad.mergePads(gamepad1, gamepad2);

            double jy = -gpad.left_stick_y - gpad.right_stick_y; // forward
            double jx = gpad.right_stick_x;  // strafing
            double jw = gpad.left_stick_x;   // turning

            robot.driveYXW(jy,jx,jw);

            telemetry.update();
        }
    }

}
