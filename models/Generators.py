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
        #self.Ifr_node = bus[Buses.bus_key_[self.Bus]].node_Ifr
        #self.Ifi_node = bus[Buses.bus_key_[self.Bus]].node_Ifi
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
        #Ifr = 1# V[self.Ifr_node]
        #Ifi = 1#V[self.Ifi_node]
        #Ifq = V[self.Ifq_node]

        Irg_hist = (-P*Vr-Q*Vi)/(Vr**2+Vi**2)    
        dL2_dlambda_r_dVr = (P*(Vr**2-Vi**2) + 2*Q*Vr*Vi)/(Vr**2+Vi**2)**2
        dL2_dlambda_r_dVi = (Q*(Vi**2-Vr**2) + 2*P*Vr*Vi)/(Vr**2+Vi**2)**2
        dL2_dlambda_r_dQ = (-Vi)/(Vr**2+Vi**2)
        Vr_J_stamp = -Irg_hist+ dL2_dlambda_r_dVr*Vr + dL2_dlambda_r_dVi*Vi + dL2_dlambda_r_dQ*Q

        #trying changeing Vr_node to lambda_r_node
        idx_Y = stampY(self.lambda_r_node, self.Vr_node, dL2_dlambda_r_dVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_r_node, self.Vi_node, dL2_dlambda_r_dVi, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_r_node, self.Q_node, dL2_dlambda_r_dQ, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.lambda_r_node, Vr_J_stamp, J_val, J_row, idx_J)

        Iig_hist = (-P*Vi+Q*Vr)/(Vr**2+Vi**2)
        dL2_dlambda_i_dVr = (Q*(Vi**2-Vr**2) + 2*P*Vr*Vi)/(Vr**2+Vi**2)**2
        dL2_dlambda_i_dVi = (P*(Vi**2-Vr**2) + 2*Q*Vr*Vi)/(Vr**2+Vi**2)**2
        dL2_dlambda_i_dQ = (Vr)/(Vr**2+Vi**2)
        Vi_J_stamp = -Iig_hist + dL2_dlambda_i_dVr*Vr + dL2_dlambda_i_dVi*Vi + dL2_dlambda_i_dQ*Q

        #Trying changing Vi_node to lambda_i_node
        idx_Y = stampY(self.lambda_i_node, self.Vr_node, dL2_dlambda_i_dVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_i_node, self.Vi_node, dL2_dlambda_i_dVi, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_i_node, self.Q_node, dL2_dlambda_i_dQ, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.lambda_i_node, Vi_J_stamp, J_val, J_row, idx_J)

        Vset_hist =  Vr**2 + Vi**2 -self.Vset**2
        dVset_dVr = 2*Vr
        dVset_dVi = 2*Vi
        Vset_J_stamp = -Vset_hist + dVset_dVr*Vr + dVset_dVi*Vi

        #trying to change Q_node to lambda_q_node
        idx_Y = stampY(self.lambda_q_node, self.Vr_node, dVset_dVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_q_node, self.Vi_node, dVset_dVi, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.lambda_q_node, Vset_J_stamp, J_val, J_row, idx_J)

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
        ###REPEATING THE PRIMAL BUT NOW NEED TO CHANGE STAMP LOCATIONS
       #############VARIABLES USED IN WOLFRAM SO THAT I CAN IMPLEMET DIRECTLY 
        x = P
        b = Q
        c=Vr
        d=Vi
        e=Lrg
        f=Lig
        g = Lqg
        ###############################################
        #VR row(changed minus to pluses for 153 and 154)
        dIrgdVr = (P*(Vr**2-Vi**2) - 2*Q*Vr*Vi)/(Vr**2+Vi**2)**2#d2L/dVrg_dLrg 
        dIrgdVi = (Q*(Vi**2-Vr**2) - 2*P*Vr*Vi)/(Vr**2+Vi**2)**2#d2L/dVrg_dLig 
        dIrgdQ = 2*Vr#(Vi)/(Vr**2+Vi**2)#d2L/dVrg_dLqg #(3)
        IGD_hist = Lrg*dIrgdVr + Lig*dIrgdVi + Lqg*dIrgdQ 

        ###ALL PARTIALS COME DIRECTLY FROM WOLFRAM
        dL2_dVr_dVr=(2*(b*(c**3*f - 3*e*c**2*d - 3*c*d**2*f + e*d**3) + c**6*g + 3*c**4*d**2*g - e*c**3*x + 3*c**2*(d**4*g - d*f*x) + 3*e*c*d**2*x + d**6*g + d**3*f*x))/(c**2 + d**2)**3
        dL2_dVr_dVi = (2*(b*(e*c**3 + 3*c**2*d*f - 3*e*c*d**2 - d**3*f) + x*(c**3*f - 3*e*c**2*d - 3*c*d**2*f + e*d**3)))/(c**2 + d**2)**3
        dL2_dVr_dQ = (c**2*(-f) + 2*e*c*d + d**2*f)/(c**2 + d**2)**2
        dL2_dVr_dlambda_r = (2*b*c*d + x*(c**2 - d**2))/(c**2 + d**2)**2
        dL2_dVr_dlambda_i =(b*(d**2 - c**2) + 2*c*d*x)/(c**2 + d**2)**2
        dL2_dVr_dlambda_q =2*c

        LAG_RG_J_stamp = (-IGD_hist+ Vi*dL2_dVr_dVi + Vr*dL2_dVr_dVr + Lrg*(dL2_dVr_dlambda_r) + Lig*(dL2_dVr_dlambda_i) +b*dL2_dVr_dQ +g*dL2_dVr_dlambda_q)

        idx_Y = stampY(self.Vr_node, self.Vr_node, dL2_dVr_dVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vr_node, self.Vi_node, dL2_dVr_dVi, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vr_node, self.Q_node, dL2_dVr_dQ, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vr_node, self.lambda_r_node, dL2_dVr_dlambda_r, Y_val, Y_row, Y_col, idx_Y)#transpose
        idx_Y = stampY(self.Vr_node, self.lambda_i_node, dL2_dVr_dlambda_i, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vr_node, self.lambda_q_node,  dL2_dVr_dlambda_q, Y_val, Y_row, Y_col, idx_Y)
        
        idx_J = stampJ(self.Vr_node, LAG_RG_J_stamp, J_val, J_row, idx_J)

        #VI row
        dIigdVi = (Q*(Vi**2-Vr**2) + 2*P*Vr*Vi)/(Vr**2+Vi**2)**2
        dIigdVr = (P*(Vi**2-Vr**2) - 2*Q*Vr*Vi)/(Vr**2+Vi**2)**2
        dIigdQ = 2*Vi#-(Vr)/(Vr**2+Vi**2)
        IGDI_hist = Lrg*dIigdVr + Lig*dIigdVi + Lqg*dIigdQ
        ###ALL PARTIALS COME DIRECTLY FROM WOLFRAM
        dL2_dVi_dVr=(2*(b*(e*c**3*+ 3*c**2*d*f - 3*e*c*d**2 - d**3*f) + x*(c**3*f - 3*e*c**2*d - 3*c*d**2*f + e*d**3)))/(c**2 + d**2)**3
        dL2_dVi_dVi = (2*(-b*(c**3*f - 3*e*c**2*d - 3*c*d**2*f + e*d**3) + c**6*g + 3*c**4*d**2*g + e*c**3*x + 3*c**2*(d**4*g + d*f*x) - 3*e*c*d**2*x + d**6*g - d**3*f*x))/(c**2 + d**2)**3
        dL2_dVi_dQ = (-e*c**2 - 2*c*d*f + e*d**2)/(c**2 + d**2)**2
        dL2_dVi_dlambda_r = (b*(d**2 - c**2) + 2*c*d*x)/(c**2 + d**2)**2
        dL2_dVi_dlambda_i =(x*(d**2 - c**2) - 2*b*c*d)/(c**2 + d**2)**2
        dL2_dVi_dlambda_q =2*d

        LAG_RG_J_stamp = -IGDI_hist + Lrg*dL2_dVi_dlambda_r + Lig*dL2_dVi_dlambda_i + Vr*dL2_dVi_dVr + Vi*dL2_dVi_dVi +Lqg*dL2_dVi_dlambda_q +b*dL2_dVi_dQ
        ##Trying changing lambda_i_node to Vi_node
        idx_Y = stampY(self.Vi_node, self.Vr_node, dL2_dVi_dVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vi_node, self.Vi_node, dL2_dVi_dVi, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vi_node, self.Q_node, dL2_dVi_dQ, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vi_node, self.lambda_r_node, dL2_dVi_dlambda_r, Y_val, Y_row, Y_col, idx_Y)#transpose
        idx_Y = stampY(self.Vi_node, self.lambda_i_node, dL2_dVi_dlambda_i, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vi_node, self.lambda_q_node, dL2_dVi_dlambda_q, Y_val, Y_row, Y_col, idx_Y)
        
        idx_J = stampJ(self.Vi_node, LAG_RG_J_stamp, J_val, J_row, idx_J)

        #Q ROW
        #Vset_hist = self.Vset**2 - Vr**2 - Vi**2
        LBDQ_hist = Lrg*(-Vi/(Vr**2+Vi**2)**2) + Lig*(Vr/(Vr**2+Vi**2)**2)
        ###ALL PARTIALS COME DIRECTLY FROM WOLFRAM
        dL2_dQ_dVr=(c**2*(-f) + 2*e*c*d + d**2*f)/(c**2 + d**2)**2
        dL2_dQ_dVi = (-e*c**2 - 2*c*d*f + e*d**2)/(c**2 + d**2)**2
        dL2_dQ_dlambda_r = -d/(c**2 + d**2)
        dL2_dQ_dlambda_i =c/(c**2 + d**2)
        LAG_Qg_history = -LBDQ_hist +Vr*dL2_dQ_dVr + Vi*dL2_dQ_dVi +Lrg*dL2_dQ_dlambda_r +Lig*dL2_dQ_dlambda_i
        idx_Y = stampY(self.Q_node, self.Vr_node, dL2_dQ_dVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Q_node, self.Vi_node, dL2_dQ_dVi, Y_val, Y_row, Y_col, idx_Y) 
        idx_Y = stampY(self.Q_node, self.lambda_r_node, dL2_dQ_dlambda_r, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Q_node, self.lambda_i_node, dL2_dQ_dlambda_i, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.Q_node, LAG_Qg_history, J_val, J_row, idx_J)



        # ### FIRST ORIGNAL ATTEMPTED METHODE
        # ###VRG
        # ####dL/dlambda_r_node ######
        # #Irg_hist = (-P*Vr+Q*Vi)/(Vr**2+Vi**2)#same power flow stamps
        # dIrgdVr = (P*(Vi**2-Vr**2) - 2*Q*Vr*Vi)/(Vr**2+Vi**2)**2#d2L/dVrg_dLrg 
        # dIrgdVi = (Q*(Vr**2-Vi**2) - 2*P*Vr*Vi)/(Vr**2+Vi**2)**2#d2L/dVrg_dLig 
        # dIrgdQ = 2*Vr#(Vi)/(Vr**2+Vi**2)#d2L/dVrg_dLqg #(3)
        # IGD_hist = Lrg*dIrgdVr + Lig*dIrgdVi + Lqg*dIrgdQ 
        # # Vr_J_stamp = -IGD_hist + dIrgdVr*Vr + dIrgdVi*Vi + dIrgdQ*Q

        # ##d2L/d2Vrg:
        # LBR_vr1 = (Lig*(-2*Q*Vr-2*Vi*P))/(Vr**2+Vi**2)**2
        # LBR_vr2 = ((4*Vr*Lig)*(Q*(Vi**2-Vr**2)-2*Vr*Vi*P))/(Vr**2+Vi**2)**3
        # LBR_vr3 = (Lrg*(2*Q*Vi+2*Vr*P))/(Vr**2+Vi**2)**2
        # LBR_vr4 = ((4*Vr*Lrg)*(P*(Vr**2-Vi**2)+2*Vr*Vi*Q))/(Vr**2+Vi**2)**3
        # LBR_vr5 = 2*Lqg
        # d2L_d2vrl = LBR_vr1 - LBR_vr2 + LBR_vr3 - LBR_vr4 + LBR_vr5 

        # ##d2L/dvrg_dvig
        # LBR_vi1 = (Lig*(2*Q*Vi-2*Vr*P))/(Vr**2+Vi**2)**2
        # LBR_vi2 = ((4*Vi*Lig)*(Q*(Vi**2-Vr**2)-2*Vr*Vi*P))/(Vr**2+Vi**2)**3
        # LBR_vi3 = (Lrg*(2*Q*Vr-2*Vi*P))/(Vr**2+Vi**2)**2
        # LBR_vi4 = ((4*Vi*Lrg)*(P*(Vr**2-Vi**2)+2*Vr*Vi*Q))/(Vr**2+Vi**2)**3
        # d2L_dvrldvil = LBR_vi1 - LBR_vi2 + LBR_vi3 - LBR_vi4

        # ##d2L/dvrg_dqg
        # LBR_q1 = (Lig*(Vi**2-Vr**2))/(Vr**2+Vi**2)**2
        # LBR_q2 = (2*Lrg*Vr*Vi)/(Vr**2+Vi**2)**2
        # d2L_dvrldq = LBR_q1 + LBR_q2
        # #LAG_RL_hist = Lrg*(dIrgdVr) + Lig*(dIrgdVi) +Lqg*(2*Vr)

        
        # LAG_RG_J_stamp = IGD_hist - Vi*d2L_dvrldvil - Vr*d2L_d2vrl -Lqg*d2L_dvrldq - Lrg*dIrgdVr - Lig*dIrgdVi

        # #trying changing lambda_r_node to Vr_node
        # idx_Y += stampY(self.Vr_node, self.Vr_node, d2L_d2vrl, Y_val, Y_row, Y_col, idx_Y)
        # idx_Y += stampY(self.Vr_node, self.Vi_node, d2L_dvrldvil, Y_val, Y_row, Y_col, idx_Y)
        # idx_Y += stampY(self.Vr_node, self.Q_node, d2L_dvrldq, Y_val, Y_row, Y_col, idx_Y)
        # idx_Y += stampY(self.Vr_node, self.lambda_r_node, dIrgdVr, Y_val, Y_row, Y_col, idx_Y)#transpose
        # idx_Y += stampY(self.Vr_node, self.lambda_i_node, dIrgdVi, Y_val, Y_row, Y_col, idx_Y)
        # idx_Y += stampY(self.Vr_node, self.lambda_q_node, dIrgdQ, Y_val, Y_row, Y_col, idx_Y)
        
        # idx_J += stampJ(self.Vr_node, LAG_RG_J_stamp, J_val, J_row, idx_J)

        # ###VIG############
        # ####dL/dlambda_i_node ######
        # #Iig_hist = (P*Vi-Q*Vr)/(Vr**2+Vi**2)
        # dIigdVi = (Q*(Vi**2-Vr**2) + 2*P*Vr*Vi)/(Vr**2+Vi**2)**2
        # dIigdVr = (P*(Vi**2-Vr**2) - 2*Q*Vr*Vi)/(Vr**2+Vi**2)**2
        # dIigdQ = 2*Vi#-(Vr)/(Vr**2+Vi**2)
        # IGDI_hist = Lrg*dIigdVi + Lig*dIigdVi + Lqg*dIigdQ

        # ##d2L/d2Vig:
        # LBI_vi1 = (Lig*(2*P*Vi-2*Vr*Q))/(Vr**2+Vi**2)**2
        # LBI_vi2 = ((4*Vi*Lig)*(P*(Vi**2-Vr**2)-2*Vr*Vi*Q))/(Vr**2+Vi**2)**3
        # LBI_vi3 = (Lrg*(2*Q*Vi+2*Vr*P))/(Vr**2+Vi**2)**2
        # LBI_vi4 = ((4*Vi*Lrg)*(Q*(Vi**2-Vr**2)+2*Vr*Vi*P))/(Vr**2+Vi**2)**3
        # LBI_vi5 = 2*Lqg
        # d2L_d2vil = LBI_vi1 - LBI_vi2 + LBI_vi3 - LBI_vi4 + LBI_vi5

        # ##d2L/dvig_dvrg
        # LBI_vr1 = (Lig*(-2*Q*Vi-2*Vr*P))/(Vr**2+Vi**2)**2
        # LBI_vr2 = ((4*Vr*Lig)*(P*(Vi**2-Vr**2)-2*Vr*Vi*Q))/(Vr**2+Vi**2)**3
        # LBI_vr3 = (Lrg*(2*P*Vi-2*Vr*Q))/(Vr**2+Vi**2)**2
        # LBI_vr4 = ((4*Vr*Lrg)*(Q*(Vi**2-Vr**2)+2*Vr*Vi*P))/(Vr**2+Vi**2)**3
        # d2L_dvildvrl = LBI_vr1 - LBI_vr2 + LBI_vr3 - LBI_vr4

        # ##d2L/dvig_dqg
        # LBI_q1 = (Lrg*(Vi**2-Vr**2))/(Vr**2+Vi**2)**2
        # LBI_q2 = (2*Lig*Vr*Vi)/(Vr**2+Vi**2)**2
        # d2L_dvildq = LBI_q1 - LBI_q2
        # #LAG_IL_hist = Lrl*(dIigdVr) + Lil*(dIigdVi) +Lql*(2*Vi)

        # #FEEL LIKE MAYBE I NEED TO ADD IFR,IFI AND IFQ TO CORRESPONDING TERMS bot here and for the stamps
        # LAG_RG_J_stamp = IGDI_hist - Lrg*dIigdVr - Lig*dIigdVi - Vr*d2L_dvildvrl - Vi*d2L_d2vil -Lqg*d2L_dvildq
        # ##Trying changing lambda_i_node to Vi_node
        # idx_Y += stampY(self.Vi_node, self.Vr_node, d2L_dvildvrl, Y_val, Y_row, Y_col, idx_Y)
        # idx_Y += stampY(self.Vi_node, self.Vi_node, d2L_d2vil, Y_val, Y_row, Y_col, idx_Y)
        # idx_Y += stampY(self.Vi_node, self.Q_node, d2L_dvildq, Y_val, Y_row, Y_col, idx_Y)
        # idx_Y += stampY(self.Vi_node, self.lambda_r_node, dIigdVr, Y_val, Y_row, Y_col, idx_Y)#transpose
        # idx_Y += stampY(self.Vi_node, self.lambda_i_node, dIigdVi, Y_val, Y_row, Y_col, idx_Y)
        # idx_Y += stampY(self.Vi_node, self.lambda_q_node, dIigdQ, Y_val, Y_row, Y_col, idx_Y)
        
        # idx_J += stampJ(self.Vi_node, LAG_RG_J_stamp, J_val, J_row, idx_J)

        # ###dQ###NO CLUE WAHT TO DO
        # ####dL/dlambda_q_node ######
        # #Vset_hist = self.Vset**2 - Vr**2 - Vi**2
        # LBDQ_hist = Lrg*(-Vi/(Vr**2+Vi**2)**2) + Lig*(Vr/(Vr**2+Vi**2)**2)
        # #Vset_J_stamp = -Vset_hist + dVset_dVr*Vr + dVset_dVi*Vi

        # #d2L_dqdvr = -(2*(Vr**2)*Lig)/((Vr**2+Vi**2)**2) + Lig/(Vr**2+Vi**2) + (2*Lrg*Vi*Vr)/((Vr**2+Vi**2)**2)
        # #d2L_dqdvi = -(2*Vr*Vi*Lig)/(Vr**2+Vi**2)**2 - Lrg/(Vr**2+Vi**2) + (2*Lrg*Vi**2)/(Vr**2+Vi**2)**2
        # d2L_dqdvr =((-Lrg*Vr**2)+(2*Vr*Vi*Lrg)+(Lig*Vi**2))/((Vr**2+Vi**2)**2)
        # d2L_dqdvi = ((-Lrg*Vr**2)-(2*Vr*Vi*Lig)+(Lrg*Vi**2))/((Vr**2+Vi**2)**2)

        # LAG_Qg_history = LBDQ_hist -Vr*d2L_dqdvr - Vi*d2L_dqdvi
        # #Trying changing lambda_q_node to Q_node
        # idx_Y += stampY(self.Q_node, self.lambda_r_node, d2L_dqdvr, Y_val, Y_row, Y_col, idx_Y)
        # idx_Y += stampY(self.Q_node, self.lambda_i_node, d2L_dqdvi, Y_val, Y_row, Y_col, idx_Y)
        # idx_J += stampJ(self.Q_node, LAG_Qg_history, J_val, J_row, idx_J)
        

        
        return (idx_Y, idx_J)

    def calc_residuals(self, resid, V):
        P = -self.P
        Vr = V[self.Vr_node]
        Vi = V[self.Vi_node]
        Q = V[self.Q_node]
        resid[self.Vr_node] += (P*Vr+Q*Vi)/(Vr**2+Vi**2)
        resid[self.Vi_node] += (P*Vi-Q*Vr)/(Vr**2+Vi**2)
        resid[self.Q_node] += self.Vset**2 - Vr**2 - Vi**2

