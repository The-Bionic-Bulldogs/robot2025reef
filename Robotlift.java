package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    // Create instances of the TalonFX motors for the elevator
    private TalonFX elevatorMotorLeft;
    private TalonFX elevatorMotorRight;

    // Create an instance of the Xbox controller
    private XboxController xboxController;

    // Elevator speed constants (tune these values)
    private final double MAX_SPEED = 0.75; // Max speed for elevator
    private final double DEADZONE = 0.05;  // Deadzone for joystick input

    // PID controller (optional: only if you plan to use PID for precise control)
    private PIDController elevatorPIDController;
    private double targetPosition = 0;

    @Override
    public void robotInit() {
        // Initialize the TalonFX motors on CAN IDs 1 and 2 (modify to match your setup)
        elevatorMotorLeft = new TalonFX(1);
        elevatorMotorRight = new TalonFX(2);

        // Initialize the Xbox controller on USB port 0
        xboxController = new XboxController(0);

        // Set the right motor to be inverted, so both motors move together
        elevatorMotorRight.setInverted(true);

        // Initialize PID controller (if using for position control)
        elevatorPIDController = new PIDController(0.1, 0.0, 0.0);
    }

    @Override
    public void teleopPeriodic() {
        // Get joystick input for elevator control (left Y-axis)
        double elevatorInput = xboxController.getLeftY();

        // Apply deadzone to ignore small controller movements
        if (Math.abs(elevatorInput) < DEADZONE) {
            elevatorInput = 0;
        }

        // Scale joystick input to the max speed
        double elevatorSpeed = elevatorInput * MAX_SPEED;

        // Send elevator speed to both motors
        elevatorMotorLeft.set(TalonFX.ControlMode.PercentOutput, elevatorSpeed);
        elevatorMotorRight.set(TalonFX.ControlMode.PercentOutput, elevatorSpeed);

        // Optional: if using PID to control the elevator position:
        // Get the current position of the elevator from the TalonFX encoder
        double currentPosition = elevatorMotorLeft.getSelectedSensorPosition();

        // Get the elevator position from the Xbox controller (can be replaced with buttons for preset positions)
        if (xboxController.getAButtonPressed()) {
            targetPosition = 1000; // example position in encoder units
        } else if (xboxController.getBButtonPressed()) {
            targetPosition = 2000; // another example position
        }

        // Use PID controller to calculate the speed needed to reach target position
        double pidOutput = elevatorPIDController.calculate(currentPosition, targetPosition);
        
        // Send PID output to motors (this is for position control; replace if using manual control)
        elevatorMotorLeft.set(TalonFX.ControlMode.PercentOutput, pidOutput);
        elevatorMotorRight.set(TalonFX.ControlMode.PercentOutput, pidOutput);

        // Display the current elevator position on the SmartDashboard for debugging
        SmartDashboard.putNumber("Elevator Position", currentPosition);
    }
}
