import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 0, 0]
        self.reference2 = None

        self.x_obs = 5
        self.y_obs = 0.1

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        beta_t = steering
        a_t = pedal
        wheel_base = 2.5 # meter
        
        #update
        x_t_1 = x_t + v_t*np.cos(psi_t)*dt
        y_t_1 = y_t + v_t*np.sin(psi_t)*dt
        psi_t_1 = psi_t + v_t*(np.tan(beta_t)/wheel_base)*dt
        v_t_1 = v_t + a_t*dt - v_t/25 # last term is drag

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
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
            cost += abs((x - ref[0]))**2 + abs((y - ref[1]))**2 #linear cost func
            #angle cost
            cost += (psi - ref[2])**2
            #acceleration cost
            cost += (v - v_prev)**2
            #steering input cost
            if (k == 0):
                steering_prev = u[2*k+1]
                
            cost += (u[2*k+1] - steering_prev)**2
            steering_prev = u[2*k+1]

            # obstacle cost
            cost += self.obstacle_cost(x, y)
            # cost += (1/ ((self.x_obs - x)**2 + (self.y_obs - y)**2)) * 30
        return cost

    def obstacle_cost (self, x, y):
        distance = np.sqrt((x - self.x_obs)**2 + (y - self.y_obs)**2)
        # in this way, no repulsion force will be applied to the car when it is far away 
        # from the obstacle
        if (distance > 2):
            cost = 0
        else:
            cost = (1 / distance)*30

        return cost

sim_run(options, ModelPredictiveControl)
