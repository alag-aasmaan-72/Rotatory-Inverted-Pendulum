def sysCall_init():

    sim = require('sim')
    
    global K
    global u
    global t
    global A

    self.reference_frame  = sim.getObject('/reference_frame')
    self.spherical_joint  = sim.getObject('/reference_frame/spherical_joint')
    self.bike_respondable = sim.getObject('/reference_frame/bike_respondable')
    self.front_motor      = sim.getObject('/reference_frame/bike_respondable/front_motor')
    self.drive_motor      = sim.getObject('/reference_frame/bike_respondable/drive_motor')

    print("  front_motor =", self.front_motor)
    print("  bike_respondable =", self.bike_respondable)
    
    
    self.theta_ref = 0 
    self.theta_dot_ref = 0 
    self.alpha_ref = 0 
    self.alpha_dot_ref = 0 
    
    

    
    
    #K = [-0.4472136 , 88.5965224 , -7.55838142 , 5.91954188]
    
    K = [-0.4472136 , 29.54992239, -2.6614169 ,  1.94874702] 
  
    
   
    
   
    
    

def sysCall_actuation():
    linear_vel, angular_vel = sim.getObjectVelocity(self.bike_respondable)

    
    orientation = sim.getObjectOrientation(self.bike_respondable, self.reference_frame)
    theta = orientation[1]
    theta_dot = angular_vel[1]
    
    alpha = orientation[2]
    
    alpha_dot = angular_vel[2]
    
    #print("orientation =", theta)
    #print("alpha =", alpha)
    #print("alpha_dot =", alpha_dot)
    #print("theta_dot =", theta_dot)
    #print("theta =", theta)
    
    
    x1 = self.theta_ref - theta 
    x2 = (self.alpha_ref - alpha)
    x3 = (self.theta_dot_ref - theta_dot)*3.5
    x4 = (self.alpha_dot_ref - alpha_dot)
    
    u  = -K[0]*x1 +K[1]*x2 - K[2]*x3 + K[3]*x4
    
    
    
    
    
    
    
    sim.setJointTargetVelocity(self.front_motor, 2*u )
    


    
    print("u =", u)
    #print("alpha =", alpha)
    #print("alpha_dot =", alpha_dot)
    #print("theta_dot =", theta_dot)
    #print("theta =", theta)
    
    


def sysCall_sensing():
    
    message,data,data2 = sim.getSimulatorMessage()
    if (message == sim.message_keypress):
        if (data[0]==2007):   # forward up arrow
            
            sim.setJointTargetVelocity(self.drive_motor, 2)
            
        if (data[0]==2008):    # backward up arrow 
       
            sim.setJointTargetVelocity(self.drive_motor, -2) 

        if (data[0]==2009):  #left arrow key 
            self.theta_ref= self.theta_ref + 0.1
            
        if (data[0]==2010):  # right arrow key 
            self.theta_ref= self.theta_ref - 0.1
            
        if (data[0]==113 ):  # right arrow key 
            
            sim.setJointTargetVelocity(self.drive_motor, 0) 
            
            
	
def sysCall_cleanup():
    pass