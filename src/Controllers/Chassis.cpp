#include "lemlib/api.hpp" 
#include "Chassis.hpp"


pros::MotorGroup left_motor_group({-1, -2, -3}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({4, 5, 6}, pros::MotorGears::blue);


lemlib::Drivetrain drivetrain(
    &left_motor_group,        
    &right_motor_group,       
    11,                            
    lemlib::Omniwheel::NEW_275,     
    450,                           
    0                              
);



lemlib::OdomSensors sensors(
    nullptr,  
    nullptr,  
    nullptr,  
    nullptr,  
    &imu      
);



lemlib::ControllerSettings latControlNoMogo(10, 
                                              0, 
                                              3, 
                                              3, 
                                              1, 
                                              100, 
                                              3, 
                                              500, 
                                              20 
);

lemlib::ControllerSettings angControlNoMogo(2, 
                                              0, 
                                              10, 
                                              3, 
                                              1, 
                                              100, 
                                              3, 
                                              500, 
                                              0 
);


lemlib::Chassis chassis(drivetrain,
                        latControlNoMogo,
                        angControlNoMogo,
                        sensors);
