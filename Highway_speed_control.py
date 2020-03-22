import numpy as np
from sim.sim1d import sim_run
# This scenario shows the car goes straight only,
# no steering needed.
# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8] # simulator size
options['FULL_RECALCULATE'] = False
# if true, use the previous result as starting point, however, may in local minima
# if false,

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20 #horizon steps
        self.dt = 0.2 #delta time

        # Reference or set point the controller will achieve.
        # goal position (x, y, theta)
        self.reference = [50, 0, 0] 

    def plant_model(self, prev_state, dt, pedal, steering):
        #prev_state: x, y, theta, velocity
        x_t = prev_state[0]
        v_t = prev_state[3] # m/s
        a_t = pedal #direct map
        
        x_t_1 = x_t + v_t * dt
        v_t_1 = v_t + a_t * dt - v_t/25 # last term represents drag
        
        return [x_t_1, 0, 0, v_t_1] #current state

    def cost_function(self, u, *args):
        #u is a sequence of control input，= np.zeros(mpc.horizon*num_inputs)
        #u = [p_t0, s_t0, p_t1, s_t1, ..., p_tn, s_tn] p:pedal, s:steering

        state = args[0]
        ref = args[1]
        cost = 0.0
        for i in range(self.horizon):
            state = self.plant_model(state, self.dt, u[i*2], u[i*2+1])

            #position error cost
            cost += (state[0] - ref[0])**2
            velocity_kph = state[3] *3.6 # km / h

            # speed constraint cost, soft constraint
            if velocity_kph > 10:
                cost += velocity_kph * 100 # just a large penalty

        return cost

sim_run(options, ModelPredictiveControl)
