package frc.robot.HumanInterfaces;

public class OperatorController {

    /* Singleton infratructure*/
    private static OperatorController inst = null;
    public static synchronized OperatorController getInstance() {
        if (inst == null)
            inst = new OperatorController();
        return inst;
    }

    private OperatorController(){
        
    }

    public void update() {
        
    }
        
    
}
