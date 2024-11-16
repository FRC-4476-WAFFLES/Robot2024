package frc.robot.utils;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.hardware.TalonFX;

public class CurrentCheck {
    public boolean IsStalled(TalonFX talonFX, double CurrentThreshold){
        return talonFX.getStatorCurrent().getValueAsDouble() > CurrentThreshold;
        
    }
}
