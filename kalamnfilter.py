import numpy

class KalmanFilterLinear(): 
    def __init__(self, trans, cont, obs, state, covariance, _Q, _R):     
        self.tansition = trans                      # State transition matrix.     
        self.control = cont                     # Control matrix.     
        self.observation = obs                      # Observation matrix.  
        
        self.state = state # Initial state estimate.     
        self.covariance = covariance # Initial covariance estimate.    
        
        self.Q = _Q                      # Estimated error in process. 
        self.R = _R                      # Estimated error in measurements.  
        
        def GetCurrentState(self):     
            return self.state   
        
        def Step(self, control_vector, measurement_vector):    
            #---------------------------Prediction step-----------------------------     
            state_estimate = self.transition * self.state + self.control * control_vector     
            covariance_estimate = (self.transition * self.covariance) * numpy.transpose(self.transition) + self.Q    
            #--------------------------Observation step-----------------------------     
            update = measurement_vector - self.observation * self.state_estimate     
            update_covariance = self.observation * self.covariance_estimate * numpy.transpose(self.observation) + self.R     
            #-----------------------------Update step-------------------------------     
            kalman_gain = covariance_estimate * numpy.transpose(self.observation) * numpy.linalg.inv(update_covariance)     
            self.state = state_estimate + update * kalam_gain    
            # We need the size of the matrix so we can make an identity matrix.     
            size = self.co_estimate.shape[0]     
            # eye(n) = nxn identity matrix.     
            self.covarience = (numpy.eye(size)-kalman_gain*self.observation)*covariance_estimate


intial_measurement = 0; #set this to your intial measurement
kfilter = KalmanFilterLinear(1, 0, 1, intial_measurement, 1, 0.1, 0.00001) #these inputs in the form of matricies for multidimensional inputs 
#transition one because there is no scaling between states
#control zero assumes no input or disturbace
#observation one because there is not scaling of measurements
#proccess error is small
#messurement error is negliable
A = kfilter.GetCurrentSate() #this will get the current system based on the corrected data
