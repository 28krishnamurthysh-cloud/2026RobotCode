package frc.robot.subsystems.shooterDemo;

import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants.ShooterDemoConstants;

public class ShooterDemo extends SubsystemBase{
    private TalonFX shooterMotor;
    private double shooterSetpoint;
    private SimpleMotorFeedforward shooterFFController;
    private PIDController shooterPIDController;
    private double shooterTolerance;
    private boolean runShooter;

    private SparkMax feederMotor;
    private double feederSetpoint;
    private SimpleMotorFeedforward feederFFController;
    private PIDController feederPIDController;
    private double feederTolerance;
    private boolean runFeeder;

    public ShooterDemo() {
        Preferences.initDouble("S_Setpoint", 0);
        Preferences.initDouble("S_kS", 0);
        Preferences.initDouble("S_kV", 0);
        Preferences.initDouble("S_kP", 0);
        Preferences.initDouble("S_Tolerance", 0);

        Preferences.initDouble("F_Setpoint", 0);
        Preferences.initDouble("F_kS", 0);
        Preferences.initDouble("F_kV", 0);
        Preferences.initDouble("F_kP", 0);
        Preferences.initDouble("F_Tolerance", 0);

        shooterMotor = new TalonFX(0);
        shooterMotor.getConfigurator().apply(ShooterDemoConstants.shooterConfig);

        shooterSetpoint = 0;

        shooterFFController = new SimpleMotorFeedforward(0, 0);
        shooterPIDController = new PIDController(0, 0, 0);

        shooterTolerance = 0;

        runShooter = false;

        //

        feederMotor = new SparkMax(1, MotorType.kBrushless);
        feederMotor.configure(ShooterDemoConstants.feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feederSetpoint = 0;

        feederFFController = new SimpleMotorFeedforward(0, 0);
        feederPIDController = new PIDController(0, 0, 0);

        feederTolerance = 0;

        runFeeder = false;
    }

    public void resetParameters() {
        shooterSetpoint = Preferences.getDouble("S_Setpoint", shooterSetpoint);
        shooterFFController.setKs(Preferences.getDouble("S_kS", shooterFFController.getKs()));
        shooterFFController.setKv(Preferences.getDouble("S_kV", shooterFFController.getKv()));
        shooterPIDController.setP(Preferences.getDouble("S_kP", shooterPIDController.getP()));
        shooterTolerance = Preferences.getDouble("S_Tolerance", shooterTolerance);

        feederSetpoint = Preferences.getDouble("F_Setpoint", feederSetpoint);
        feederFFController.setKs(Preferences.getDouble("F_kS", feederFFController.getKs()));
        feederFFController.setKv(Preferences.getDouble("F_kV", feederFFController.getKv()));
        feederPIDController.setP(Preferences.getDouble("F_kP", feederPIDController.getP()));
        feederTolerance = Preferences.getDouble("F_Tolerance", feederTolerance);
    }

    public void runShooter() {
        double currVel = shooterMotor.getVelocity().getValueAsDouble();
        shooterMotor.setVoltage(
            shooterFFController.calculate(shooterSetpoint) 
            + (Math.abs(currVel - shooterSetpoint) > shooterTolerance ?
                shooterPIDController.calculate(currVel, shooterSetpoint) : 0)
        );
    }

    public void runFeeder() {
        double currVel = feederMotor.getEncoder().getVelocity();
        feederMotor.setVoltage(
            feederFFController.calculate(feederSetpoint) 
            + (Math.abs(currVel - feederSetpoint) > feederTolerance ?
                feederPIDController.calculate(currVel, feederSetpoint) : 0)
        );
    }

    public Command cmdResetParameters() {
        return runOnce(()-> {
            resetParameters();
        });
    }

    public Command cmdRunShooter() {
        return runOnce(()-> {
            runShooter = true;
        });
    }

    public Command cmdRunFeeder() {
        return runOnce(()-> {
            runFeeder = true;
        });
    }

    public Command cmdCutShooter() {
        return runOnce(()-> {
            runShooter = false;
            shooterMotor.setVoltage(0);
        });
    }

    public Command cmdCutFeeder() {
        return runOnce(()-> {
            runFeeder = false;
            feederMotor.setVoltage(0);
        });
    }

    @Override
    public void periodic() {
        if(runShooter) {
            runShooter();
        }

        if(runFeeder) {
            runFeeder();
        }
    }
}
