import sympy
from sympy import symbols, Eq, Function, solve, sqrt
import matplotlib.pyplot as plt
import math
import numpy as np
from numpy import sin, cos

def gener_traj(Tval):
    T, t = symbols(r'T, t')
    v = Function(r'v')(t)
    x = Function(r'x')(t)
    y = Function(r'y')(t)
    
    # function to get particular trajectory and follow
    
    y = 1.5*sympy.sin(t/T) 		# a function for the component x(t) of the ellipse trajectory
    x = -4.016 + 4*sympy.cos(t/T) 	# a function for the component y(t) of the ellipse trajectory
    

    # taking derivatives for getting velocity in both(x, y) direction
    
    xder = x.diff(t)
    yder = y.diff(t)
    
    xdder = xder.diff(t)
    ydder = yder.diff(t)

    # from kinetic equation point of view we get term similar to xdot = v*cos(theta) and ydot = v*sin(theta)
    
    v = sympy.sqrt(xder**2 + yder**2) 

    theta = sympy.atan2(yder, xder)

    thetader = theta.diff(t)  # derivative d/dt of theta(t)
    omega = thetader 	      # defining omega


    v = sympy.simplify(v.subs([(T, Tval)]))
    omega = sympy.simplify(omega.subs([(T, Tval)]))
    x = sympy.simplify(x.subs([(T, Tval)]))
    xder = sympy.simplify(xder.subs([(T, Tval)]))
    xdder = sympy.simplify(xdder.subs([(T, Tval)]))
    y = sympy.simplify(y.subs([(T, Tval)]))
    yder = sympy.simplify(yder.subs([(T, Tval)]))
    ydder = sympy.simplify(ydder.subs([(T, Tval)]))

    return x, y, xder, yder, xdder, ydder, v, omega

class oval():
    def __init__(self, Tval):
        # inti
        self._x, self._y, self._xdot, self._ydot, self._xddot, self._yddot, self._v, self._omega = gener_traj(Tval)
        self._t = symbols(r't')
    def velocity(self, time_step):
        """ 
            This function receives the result of the calculation and give outputs the position 	     (x, y) and the linear and angular velocity (v, w). 
        """
        x = self._x.subs(self._t, time_step)
        y = self._y.subs(self._t, time_step)
        v = self._v.subs(self._t, time_step)
        omega = self._omega.subs(self._t, time_step)
        return x, y, v, omega

    def test(self, time_step):
        """ 
            A function to test the calculation by mapping  
        """
        x = self._x.subs(self._t, time_step)
        y = self._y.subs(self._t, time_step)
        xdot = self._xdot.subs(self._t, time_step)
        ydot = self._ydot.subs(self._t, time_step)
        xddot = self._xddot.subs(self._t, time_step)
        yddot = self._yddot.subs(self._t, time_step)
        v = self._v.subs(self._t, time_step)
        omega = self._omega.subs(self._t, time_step)
        return x, y, xdot, ydot, xddot, yddot, v, omega
    
    def map(self):
        """
            Mapping generated point using matplotlib
        """
        t = np.linspace(0, 2*np.pi, 360)
        x, y, v, o = self.velocity(t)
        plt.plot(x, y)
        plt.grid(color='lightgray',linestyle='--')
        plt.show()


