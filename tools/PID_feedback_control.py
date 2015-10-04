class PID:
    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator= 0.0, Integrator= 0.0, Integrator_max = 500, Integrator_min = -500):
        #sets the value of the various gains (scalars)
        self.proportional_gain = P
        self.intregral_gain = I
        self.dervivative_gain = D

        #sets the inital error and the target (setpoint)
        self.set_point = 0.0
        self.error =0.0

         #self bounds
        self.integrator_min, self.integrator_max = Integrator_min, Integrator_max

        #integral and derivative partial components
        self.integrator = 0.0
        self.derivator = 0.0

        #value to store current value for the measurement
        self.current_value = self.set_point

        #sets intergal and error function
        self.set_integrator(Integrator)
        self.set_derivator(Derivator)

    def _error(self, x_2, x_1):
        error = x_1 - x_2
        return error

    def update_PID(self, current_value): #date values
        self.set_current(current_value)
        self.update_proportional()
        self.update_derivative()
        self.update_intergral()
        return self.P_value+self.I_value()+self.D_value


    #begins updates
    def update_proportional(self): #error
        self.P_value = self.proportional_gain * self_error(self.get_current(), self.set_point)

    def update_dervative(self): #change in error
        self.D_value = self.dervivative_gain * self._error(self.get_derivator, self._error(self.get_curent(), self.set_point()))
        self.set_derivator(self._error())

    def update_intergral(self): #accumulative error
        self.set_integrator()
        self.I_value = self.get_integrator() * self.integratal_gain()

    #begins setters and getters
    def set_integrator(self, Integrator = None): #updates the integrator
        while(self.integrator_min < self.integrator < self.integrator_max):
            self.integrator += self._error(self.get_current(), self.set_point)

    def set_derivator(self, derivator): #sets the derivative
            self.derivator = derivator

    def set_current(self, current):
        self.current_value = current

    def set_point(self, set_point):
        self.set_point = set_point

    def get_integrator(self):
        return self.integrator

    def get_derivator(self):
        return self.derivator

    def get_current(self):
        return self.current_value
        
