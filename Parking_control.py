import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2
        # self.beta_t = 0
        # Reference or set point the controller will achieve.
        self.reference1 = [10, 10, 0] # first goal point
        self.reference2 = [10, 2, 3 * 3.14/2] # second goal point

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        # steering_velocity = steering
        beta_t = steering
        # self.beta_t += self.beta_t + steering_velocity * dt
        a_t = pedal
        wheel_base = 2.5 # meter
        
        #update
        x_t_1 = x_t + v_t*np.cos(psi_t)*dt
        y_t_1 = y_t + v_t*np.sin(psi_t)*dt
        psi_t_1 = psi_t + v_t*(np.tan(beta_t)/wheel_base)*dt
        v_t_1 = v_t + a_t*dt - v_t/25 # last term is drag

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self, u, *args):
        state = args[0]
        ref = args[1] #self.reference, (x,y,theta)
        cost = 0.0
        steering_prev = 0
        for k in range(self.horizon):
            v_prev = state[3] #previous velocity
            state = self.plant_model(state, self.dt, u[2*k], u[2*k+1]) #u[2*k]: pedal
                                                                       #u[2*k+1]:steering
            x = state[0]
            y = state[1]
            psi = state[2]
            v = state[3]

            #position cost
            cost += abs((x - ref[0])) + abs((y - ref[1])) #linear cost func
            #angle cost
            cost += (psi - ref[2])**2
            #acceleration cost
            cost += (v - v_prev)**2
            #steering input cost
            if (k == 0):
                steering_prev = u[2*k+1]
                
            cost += (u[2*k+1] - steering_prev)**2
            steering_prev = u[2*k+1]
        return cost

sim_run(options, ModelPredictiveControl)
