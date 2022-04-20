from __future__ import division
from itertools import count
from scripts.global_vars import global_vars
from models.Buses import Buses
from scripts.stamp_helpers import *
from models.global_vars import global_vars
#THERE ARE 8 ROWS IN TOTAL FOR EACH GENERATOR
#3FOR POWER FLOW(DONE BY STAMP FUNCTION)
#3FOR LAMBDA (DONE BY STAMP DUAL)
#2 FOR IN FEASABILITY(THESE HAVE BOTH LINEAR AND NONLINEAR AND DONE BY INFEASABILITY SOURCES)

class Generators:
    _ids = count(0)
    RemoteBusGens = dict()
    RemoteBusRMPCT = dict()
    gen_bus_key_ = {}
    total_P = 0

    def __init__(self,
                 Bus,
                 P,
                 Vset,
                 Qmax,
                 Qmin,
                 Pmax,
                 Pmin,
                 Qinit,
                 RemoteBus,
                 RMPCT,
                 gen_type):
        """Initialize an instance of a generator in the power grid.

        Args:
            Bus (int): the bus number where the generator is located.
            P (float): the current amount of active power the generator is providing.
            Vset (float): the voltage setpoint that the generator must remain fixed at.
            Qmax (float): maximum reactive power
            Qmin (float): minimum reactive power
            Pmax (float): maximum active power
            Pmin (float): minimum active power
            Qinit (float): the initial amount of reactive power that the generator is supplying or absorbing.
            RemoteBus (int): the remote bus that the generator is controlling
            RMPCT (float): the percent of total MVAR required to hand the voltage at the controlled bus
            gen_type (str): the type of generator
        """

        self.Bus = Bus
        self.P_MW = P
        self.Vset = Vset
        self.Qmax_MVAR = Qmax
        self.Qmin_MVAR = Qmin
        self.Pmax_MW = Pmax
        self.Pmin_MW = Pmin
        self.Qinit_MVAR = Qinit
        self.RemoteBus = RemoteBus
        self.RMPCT = RMPCT
        self.gen_type = gen_type
        # convert P/Q to pu
        self.P = P/global_vars.base_MVA
        self.Vset = Vset
        self.Qmax = Qmax/global_vars.base_MVA
        self.Qmin = Qmin/global_vars.base_MVA
        self.Qinit = Qinit/global_vars.base_MVA
        self.Pmax = Pmax/global_vars.base_MVA
        self.Pmin = Pmin/global_vars.base_MVA

        self.id = self._ids.__next__()

    def assign_indexes(self, bus):
        # Nodes shared by generators on the same bus
        self.Vr_node = bus[Buses.bus_key_[self.Bus]].node_Vr
        self.Vi_node = bus[Buses.bus_key_[self.Bus]].node_Vi
        # run check to make sure the bus actually has a Q node
        self.Q_node = bus[Buses.bus_key_[self.Bus]].node_Q
        # check something about gen_type?? 
        self.Ifr_node = bus[Buses.bus_key_[self.Bus]].node_Ifr
        self.Ifi_node = bus[Buses.bus_key_[self.Bus]].node_Ifi
        #self.Ifq_node = bus[Buses.bus_key_[self.Bus]].node_Ifq

        ##ASSIGNING THE LAMBDA
        self.lambda_r_node = bus[Buses.bus_key_[self.Bus]].lambda_r_node
        self.lambda_i_node = bus[Buses.bus_key_[self.Bus]].lambda_i_node
        # run check to make sure the bus actually has a Q node
        self.lambda_q_node = bus[Buses.bus_key_[self.Bus]].lambda_q_node
    
    def stamp(self, V, Y_val, Y_row, Y_col, J_val, J_row, idx_Y, idx_J):
        P = -self.P
        Vr = V[self.Vr_node]
        Vi = V[self.Vi_node]
        Q = V[self.Q_node]
        ###REALLY NOT SURE HOW TO HANDLE THESE INFEASIABLITY
        Ifr = 1# V[self.Ifr_node]
        Ifi = 1#V[self.Ifi_node]
        #Ifq = V[self.Ifq_node]

        Irg_hist = (P*Vr+Q*Vi)/(Vr**2+Vi**2) ##(1)   + Ifr #()
        dIrgdVr = (P*(Vi**2-Vr**2) - 2*Q*Vr*Vi)/(Vr**2+Vi**2)**2
        dIrgdVi = (Q*(Vr**2-Vi**2) - 2*P*Vr*Vi)/(Vr**2+Vi**2)**2
        dIrgdQ = (Vi)/(Vr**2+Vi**2)
        Vr_J_stamp = -Irg_hist + dIrgdVr*Vr + dIrgdVi*Vi + dIrgdQ*Q

        idx_Y = stampY(self.Vr_node, self.Vr_node, dIrgdVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vr_node, self.Vi_node, dIrgdVi, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vr_node, self.Q_node, dIrgdQ, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.Vr_node, Vr_J_stamp, J_val, J_row, idx_J)

        Iig_hist = (P*Vi-Q*Vr)/(Vr**2+Vi**2)#(2)
        dIigdVi = -dIrgdVr
        dIigdVr = dIrgdVi
        dIigdQ = -(Vr)/(Vr**2+Vi**2)
        Vi_J_stamp = -Iig_hist + dIigdVr*Vr + dIigdVi*Vi + dIigdQ*Q

        idx_Y = stampY(self.Vi_node, self.Vr_node, dIigdVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vi_node, self.Vi_node, dIigdVi, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vi_node, self.Q_node, dIigdQ, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.Vi_node, Vi_J_stamp, J_val, J_row, idx_J)

        Vset_hist = self.Vset**2 - Vr**2 - Vi**2#(3)
        dVset_dVr = -2*Vr
        dVset_dVi = -2*Vi
        Vset_J_stamp = -Vset_hist + dVset_dVr*Vr + dVset_dVi*Vi

        idx_Y = stampY(self.Q_node, self.Vr_node, dVset_dVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Q_node, self.Vi_node, dVset_dVi, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.Q_node, Vset_J_stamp, J_val, J_row, idx_J)

        return (idx_Y, idx_J)

    def stamp_dual(self,V, Y_val, Y_row, Y_col, J_val, J_row, idx_Y, idx_J):
        # You need to implement this.
        #note there are 8 rows in the pv stamp anywhere witha #() gives what row
        P = -self.P
        Vr = V[self.Vr_node]
        Vi = V[self.Vi_node]
        Q = V[self.Q_node]
        Lrg = V[self.lambda_r_node]
        Lig = V[self.lambda_i_node]
        Lqg = V[self.lambda_q_node]
        ###REALLY NOT SURE HOW TO HANDLE THESE INFEASIABLITY
        Ifr = V[self.Ifr_node]
        Ifi = V[self.Ifi_node]
        #Ifq = V[self.Ifq_node]

        ###D LAMBDAS

        ###VRG
        ####dL/dlambda_r_node ######
        #Irg_hist = (P*Vr+Q*Vi)/(Vr**2+Vi**2)#same power flow stamps
        dIrgdVr = (P*(Vi**2-Vr**2) - 2*Q*Vr*Vi)/(Vr**2+Vi**2)**2#d2L/dVrg_dLrg possibly needs a 
        dIrgdVi = (Q*(Vr**2-Vi**2) - 2*P*Vr*Vi)/(Vr**2+Vi**2)**2#d2L/dVrg_dLig
        dIrgdQ = (Vi)/(Vr**2+Vi**2)#d2L/dVrg_dLqg 
        IGD_hist = Lrg*dIrgdVr + Lig*dIrgdVi + Lqg*dIrgdQ #(4)
        # Vr_J_stamp = -IGD_hist + dIrgdVr*Vr + dIrgdVi*Vi + dIrgdQ*Q

        ##d2L/d2Vrg:
        LBR_vr1 = (Lig*(-2*Q*Vr-2*Vi*P))/(Vr**2+Vi**2)**2
        LBR_vr2 = ((4*Vr*Lig)*(Q*(Vi**2-Vr**2)-2*Vr*Vi*P))/(Vr**2+Vi**2)**3
        LBR_vr3 = (Lrg*(2*Q*Vi+2*Vr*P))/(Vr**2+Vi**2)**2
        LBR_vr4 = ((4*Vr*Lrg)*(P*(Vr**2-Vi**2)+2*Vr*Vi*Q))/(Vr**2+Vi**2)**3
        LBR_vr5 = 2*Lqg
        d2L_d2vrl = LBR_vr1 - LBR_vr2 + LBR_vr3 - LBR_vr4 + LBR_vr5 #(4)

        ##d2L/dvrg_dvig
        LBR_vi1 = (Lig*(2*Q*Vi-2*Vr*P))/(Vr**2+Vi**2)**2
        LBR_vi2 = ((4*Vi*Lig)*(Q*(Vi**2-Vr**2)-2*Vr*Vi*P))/(Vr**2+Vi**2)**3
        LBR_vi3 = (Lrg*(2*Q*Vr-2*Vi*P))/(Vr**2+Vi**2)**2
        LBR_vi4 = ((4*Vi*Lrg)*(P*(Vr**2-Vi**2)+2*Vr*Vi*Q))/(Vr**2+Vi**2)**3
        d2L_dvrldvil = LBR_vi1 - LBR_vi2 + LBR_vi3 - LBR_vi4

        ##d2L/dvrg_dqg
        LBR_q1 = (Lig*(Vi**2-Vr**2))/(Vr**2+Vi**2)**2
        LBR_q2 = (2*Lrg*Vr*Vi)/(Vr**2+Vi**2)**2
        d2L_dvrldq = LBR_q1 + LBR_q2
        #LAG_RL_hist = Lrg*(dIrgdVr) + Lig*(dIrgdVi) +Lqg*(2*Vr)

        #FEEL LIKE MAYBE I NEED TO ADD IFR,IFI AND IFQ TO CORRESPONDING TERMS bot here and for the stamps
        LAG_RG_J_stamp = -IGD_hist + Lrg*dIrgdVr + Lig*dIrgdVi + Vi*d2L_dvrldvil + Vr*d2L_d2vrl +Lqg*d2L_dvrldq

        idx_Y = stampY(self.lambda_r_node, self.Vr_node, d2L_d2vrl, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_r_node, self.Vi_node, d2L_dvrldvil, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_r_node, self.Q_node, d2L_dvrldq, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_r_node, self.lambda_r_node, dIrgdVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_r_node, self.lambda_i_node, dIrgdVi, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_r_node, self.lambda_q_node, dIrgdQ, Y_val, Y_row, Y_col, idx_Y)
        
        idx_J = stampJ(self.lambda_r_node, LAG_RG_J_stamp, J_val, J_row, idx_J)

        ###VIG############
        ####dL/dlambda_i_node ######
        dIigdVi = -dIrgdVr
        dIigdVr = dIrgdVi
        dIigdQ = -(Vr)/(Vr**2+Vi**2)
        IGDI_hist = Lrg*dIigdVi + Lig*dIigdVi + Lqg*dIigdQ#(5)

        ##d2L/d2Vig:
        LBI_vi1 = (Lig*(2*P*Vi-2*Vr*Q))/(Vr**2+Vi**2)**2
        LBI_vi2 = ((4*Vi*Lig)*(P*(Vi**2-Vr**2)-2*Vr*Vi*Q))/(Vr**2+Vi**2)**3
        LBI_vi3 = (Lrg*(2*Q*Vi+2*Vr*P))/(Vr**2+Vi**2)**2
        LBI_vi4 = ((4*Vi*Lrg)*(Q*(Vi**2-Vr**2)+2*Vr*Vi*P))/(Vr**2+Vi**2)**3
        LBI_vi5 = 2*Lqg
        d2L_d2vil = LBI_vi1 - LBI_vi2 + LBI_vi3 - LBI_vi4 + LBI_vi5

        ##d2L/dvig_dvrg
        LBI_vr1 = (Lig*(2*P*Vi-2*Vr*Q))/(Vr**2+Vi**2)**2
        LBI_vr2 = ((4*Vi*Lig)*(P*(Vi**2-Vr**2)-2*Vr*Vi*Q))/(Vr**2+Vi**2)**3
        LBI_vr3 = (Lrg*(2*Q*Vi-2*Vr*P))/(Vr**2+Vi**2)**2
        LBI_vr4 = ((4*Vi*Lrg)*(P*(Vi**2-Vr**2)+2*Vr*Vi*P))/(Vr**2+Vi**2)**3
        d2L_dvildvrl = LBI_vr1 - LBI_vr2 + LBI_vr3 - LBI_vr4

        ##d2L/dvig_dqg
        LBI_q1 = (Lrg*(Vi**2-Vr**2))/(Vr**2+Vi**2)**2
        LBI_q2 = (2*Lig*Vr*Vi)/(Vr**2+Vi**2)**2
        d2L_dvildq = LBI_q1 + LBI_q2
        #LAG_IL_hist = Lrl*(dIigdVr) + Lil*(dIigdVi) +Lql*(2*Vi)

        #FEEL LIKE MAYBE I NEED TO ADD IFR,IFI AND IFQ TO CORRESPONDING TERMS bot here and for the stamps
        LAG_RG_J_stamp = -IGDI_hist + Lrg*dIigdVr + Lig*dIigdVi + Vr*d2L_dvildvrl + Vi*d2L_d2vil +Lqg*d2L_dvildq

        idx_Y = stampY(self.lambda_i_node, self.Vr_node, d2L_dvildvrl, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_i_node, self.Vi_node, d2L_d2vil, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_i_node, self.Q_node, d2L_dvildq, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_i_node, self.lambda_r_node, dIigdVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_i_node, self.lambda_i_node, dIigdVi, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_i_node, self.lambda_q_node, dIigdQ, Y_val, Y_row, Y_col, idx_Y)
        
        idx_J = stampJ(self.lambda_r_node, LAG_RG_J_stamp, J_val, J_row, idx_J)

        ###dQ###NO CLUE WAHT TO DO
        ####dL/dlambda_q_node ######
        LBDQ_hist = Lrg*(-Vi/(Vr**2+Vi**2)**2) + Lig*(Vr/(Vr**2+Vi**2)**2)#(6)
        #Vset_J_stamp = -Vset_hist + dVset_dVr*Vr + dVset_dVi*Vi

        d2L_dqdvr = -(2*Vr**2*Lig)/(Vr**2+Vi**2)**2 + Lig/(Vr**2+Vi**2) + (2*Lrg*Vi*Vr)/(Vr**2+Vi**2)**2
        d2L_dqdvi = -(2*Vr*Vi*Lig)/(Vr**2+Vi**2)**2 - Lrg/(Vr**2+Vi**2) + (2*Lrg*Vi**2)/(Vr**2+Vi**2)**2
        
        LAG_Qg_history = -LBDQ_hist +Vr*d2L_dqdvr + Vi*d2L_dqdvi
        idx_Y = stampY(self.lambda_q_node, self.lambda_i_node, dIigdVi, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_q_node, self.lambda_q_node, dIigdQ, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.lambda_q_node, LAG_Qg_history, J_val, J_row, idx_J)
        #########INFISABLITY STAMPS NOT SURE IF I DO THEM HERE OR IN THE CLASS
        pass

    def calc_residuals(self, resid, V):
        P = -self.P
        Vr = V[self.Vr_node]
        Vi = V[self.Vi_node]
        Q = V[self.Q_node]
        resid[self.Vr_node] += (P*Vr+Q*Vi)/(Vr**2+Vi**2)
        resid[self.Vi_node] += (P*Vi-Q*Vr)/(Vr**2+Vi**2)
        resid[self.Q_node] += self.Vset**2 - Vr**2 - Vi**2

