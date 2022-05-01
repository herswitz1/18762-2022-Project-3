import numpy as np

def initialize(size_Y, bus, generator, slack, flat_start=False):
    V_init = np.zeros(size_Y, dtype=np.float)
    if flat_start:
        for ele in bus:
            V_init[ele.node_Vr] = 1
            V_init[ele.node_Vi] = 0
            # TODO: You'll want to initialize all the lambda values as well...
            # HINT: A very small, positive number usually works well
            V_init[ele.lambda_r_node] = .000001
            V_init[ele.lambda_i_node] = .000001
            V_init[ele.node_Ifr] = 1
            V_init[ele.node_Ifi]= 1
        for ele in generator:
            V_init[ele.Q_node] += 1#(ele.Qmax+ele.Qmin)/2
            # TODO: initialize the lambda associated with the Vset equation
            V_init[ele.lambda_q_node] += .00001
        for ele in slack:
            # TODO: Initialize all the slack current injections
            #V_init[ele.]
            pass
    else:
        for ele in bus:
            V_init[ele.node_Vr] = ele.Vr_init
            V_init[ele.node_Vi] = ele.Vi_init
            # TODO: You'll want to initialize all the lambda values as well...
            # HINT: A very small, positive number usually works well
            V_init[ele.lambda_r_node] = .000001
            V_init[ele.lambda_i_node] = .000001
            V_init[ele.node_Ifr] = 1#.000001
            V_init[ele.node_Ifi]= 1#.000001
        for ele in generator:
            V_init[ele.Q_node] += -ele.Qinit#If i make this one it converges in 2 iteratarions with
            # TODO: initialize the lambda associated with the Vset equation
            V_init[ele.lambda_q_node] += .000001
        for ele in slack:
            V_init[ele.Slack_Ir_node] = ele.Ir_init
            V_init[ele.Slack_Ii_node] = ele.Ii_init
            V_init[ele.lambda_slack_r_node] = .000001
            V_init[ele.lambda_slack_i_node] = .000001
        for ele in slack:
            # TODO: Initialize all the slack current injections(are these infeasability currents)
            pass

    return V_init