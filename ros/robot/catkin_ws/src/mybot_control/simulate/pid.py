import time,math
class PID(object):

    def __init__(self, p_gain, i_gain, d_gain, i_max, i_min):
        self.set_gains(p_gain, i_gain, d_gain, i_max, i_min)
        self.reset()

    def reset(self):
        self._p_error_last = 0.0 # Save position state for derivative
        self._p_error = 0.0 # Position error.
        self._d_error = 0.0 # Derivative error.
        self._i_error = 0.0 # Integator error.
        self._cmd = 0.0 # Command to send.
        self._last_time = None # Used for automatic calculation of dt.
        
    def set_gains(self, p_gain, i_gain, d_gain, i_max, i_min): 
        self._p_gain = p_gain
        self._i_gain = i_gain
        self._d_gain = d_gain
        self._i_max = i_max
        self._i_min = i_min

    @property
    def p_gain(self):
        return self._p_gain

    @property
    def i_gain(self):
        return self._i_gain

    @property
    def d_gain(self):
        return self._d_gain

    # @property_cmd
    @property
    def i_max(self):
        return self._i_max

    @property
    def i_min(self):
        return self._i_min

    @property
    def p_error(self):
        return self._p_error

    @property
    def i_error(self):
        return self._i_error

    @property
    def d_error(self):
        return self._d_error

    @property
    def cmd(self):
        return self._cmd

    def __str__(self):
        result = ""
        result += "p_gain:  " + str(self.p_gain) + "\n"
        result += "i_gain:  " + str(self.i_gain) + "\n"
        result += "d_gain:  " + str(self.d_gain) + "\n"
        result += "i_max:   " + str(self.i_max) + "\n"
        result += "i_min:   " + str(self.i_min) + "\n"
        result += "p_error: " + str(self.p_error) + "\n"
        result += "i_error: " + str(self.i_error) + "\n"
        result += "d_error: " + str(self.d_error) + "\n"
        result += "cmd:     " + str(self.cmd) + "\n"
        return result
        
    def update_PID(self, p_error, dt=None):
        if dt == None:
            cur_time = time.time()
            if self._last_time is None:
                self._last_time = cur_time 
            dt = cur_time - self._last_time
            self._last_time = cur_time
            
        self._p_error = p_error # this is pError = pState-pTarget
        if dt == 0 or math.isnan(dt) or math.isinf(dt):
            return 0.0
        p_term = self._p_gain * self._p_error
        self._i_error += dt * self._p_error    
        i_term = self._i_gain * self._i_error
        if i_term > self._i_max and self._i_gain != 0:
            i_term = self._i_max
            self._i_error = i_term / self._i_gain
        elif i_term < self._i_min and self._i_gain != 0:
            i_term = self._i_min
            self._i_error = i_term / self._i_gain
        self._d_error = (self._p_error - self._p_error_last) / dt
        self._p_error_last = self._p_error 
        d_term = self._d_gain * self._d_error
        self._cmd = -p_term - i_term - d_term
        return -self._cmd