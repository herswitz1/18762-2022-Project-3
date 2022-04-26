from __future__ import division
from itertools import count
from models.Buses import Buses
from scripts.stamp_helpers import *
from models.global_vars import global_vars

class Loads:
    _ids = count(0)

    def __init__(self,
                 Bus,
                 P,
                 Q,
                 IP,
                 IQ,
                 ZP,
                 ZQ,
                 area,
                 status):
        """Initialize an instance of a PQ or ZIP load in the power grid.

        Args:
            Bus (int): the bus where the load is located
            self.P (float): the active power of a constant power (PQ) load.
            Q (float): the reactive power of a constant power (PQ) load.
            IP (float): the active power component of a constant current load.
            IQ (float): the reactive power component of a constant current load.
            ZP (float): the active power component of a constant admittance load.
            ZQ (float): the reactive power component of a constant admittance load.
            area (int): location where the load is assigned to.
            status (bool): indicates if the load is in-service or out-of-service.
        """
        self.Bus = Bus
        self.P_MW = P
        self.Q_MVA = Q
        self.IP_MW = IP
        self.IQ_MVA = IQ
        self.ZP_MW = ZP
        self.ZQ_MVA = ZQ
        self.area = area
        self.status = status
        self.id = Loads._ids.__next__()

        self.P = P/global_vars.base_MVA
        self.Q = Q/global_vars.base_MVA
        self.IP = IP/global_vars.base_MVA
        self.IQ = IQ/global_vars.base_MVA
        self.ZP = ZP/global_vars.base_MVA
        self.ZQ = ZQ/global_vars.base_MVA
    
    def assign_indexes(self, bus):
        # Nodes shared by generators on the same bus
        self.Vr_node = bus[Buses.bus_key_[self.Bus]].node_Vr
        self.Vi_node = bus[Buses.bus_key_[self.Bus]].node_Vi
        # check something about gen_type??
        #self.Ifr_node = bus[Buses.bus_key_[self.Bus]].node_Ifr
        #self.Ifi_node = bus[Buses.bus_key_[self.Bus]].node_Ifi
        #DUEL NODES FOR LAMBDA
        self.lambda_r_node = bus[Buses.bus_key_[self.Bus]].lambda_r_node
        self.lambda_i_node = bus[Buses.bus_key_[self.Bus]].lambda_i_node

    
    def stamp(self, V, Y_val, Y_row, Y_col, J_val, J_row, idx_Y, idx_J):
        Vr = V[self.Vr_node]
        Vi = V[self.Vi_node]
        #############VARIABLES USED IN WOLFRAM SO THAT I CAN IMPLEMET DIRECTLY 
        x = self.P
        b = self.Q
        c=Vr
        d=Vi
        
        ###############################################

        Irg_hist = (self.P*Vr+self.Q*Vi)/(Vr**2+Vi**2) 
        dL2_dlambda_r_dVr = ((self.P*(Vi**2-Vr**2) - 2*self.Q*Vr*Vi)/(Vr**2+Vi**2)**2) 
        dL2_dlambda_r_dVi = ((self.Q*(Vr**2-Vi**2) - 2*self.P*Vr*Vi)/(Vr**2+Vi**2)**2) 
        
        Vr_J_stamp = -Irg_hist + (dL2_dlambda_r_dVr)*Vr + (dL2_dlambda_r_dVi)*Vi #+Ifr*dIrldIfr
        
        #LAMBA_R ROW
        #Trying changing Vr_node to lambda_r_node
        idx_Y = stampY(self.lambda_r_node, self.Vr_node, dL2_dlambda_r_dVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_r_node, self.Vi_node, dL2_dlambda_r_dVi, Y_val, Y_row, Y_col, idx_Y)
        #idx_Y = stampY(self.Vr_node, self.Ifr_node, dIrldIfr, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.lambda_r_node, Vr_J_stamp, J_val, J_row, idx_J)

        #LAMBDA I ROW
        Iig_hist = (self.P*Vi-self.Q*Vr)/(Vr**2+Vi**2) #+Ifi
        dL2_dlambda_i_dVr = (b*(c**2 - d**2) - 2*c*d*x)/(c**2 + d**2)**2#directly from wolfram
        dL2_dlambda_i_dVi = (2*b*c*d + x*(c**2 - d**2))/(c**2 + d**2)**2
        #dIildIfi = 1
        Vi_J_stamp = -Iig_hist + (dL2_dlambda_i_dVr)*Vr + (dL2_dlambda_i_dVi)*Vi #+Ifi*dIildIfi
        #Trying changing Vi_node to lambda_i_node
        idx_Y = stampY(self.lambda_i_node, self.Vr_node, dL2_dlambda_i_dVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_i_node, self.Vi_node, dL2_dlambda_i_dVi, Y_val, Y_row, Y_col, idx_Y)
        #idx_Y = stampY(self.Vi_node, self.Ifi_node, dIildIfi, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.lambda_i_node, Vi_J_stamp, J_val, J_row, idx_J)

        return (idx_Y, idx_J)

    def stamp_dual(self,V, Y_val, Y_row, Y_col, J_val, J_row, idx_Y, idx_J):
        # You need to implement this.
        Vr = V[self.Vr_node]
        Vi = V[self.Vi_node]
        Lrl = V[self.lambda_r_node]
        Lil = V[self.lambda_i_node]
        #############VARIABLES USED IN WOLFRAM SO THAT I CAN IMPLEMET DIRECTLY 
        x = self.P
        b = self.Q
        c=Vr
        d=Vi
        e=Lrl
        f=Lil
        ###############################################

        #Irg_hist_1 = (self.P*Vr+self.Q*Vi)/(Vr**2+Vi**2) #+ Ifr
        #Irg_hist_2 = (self.P*Vi-self.Q*Vr)/(Vr**2+Vi**2)
        #VR ROW
        dIrldVr = ((self.P*(Vi**2-Vr**2) - 2*self.Q*Vr*Vi)/(Vr**2+Vi**2)**2) #this is d^2L/dVrl_dLrl(1_)
        dIrldVi = ((self.Q*(Vr**2-Vi**2) - 2*self.P*Vr*Vi)/(Vr**2+Vi**2)**2) #this is d^2L/dVrl_dLil(2)
        LAG_RL_hist = Lrl*(dIrldVr) + Lil*(dIrldVi)
        ###ALL PARTIALS COME DIRECTLY FROM WOLFRAM
        dL2_dVr_dVr=-(2*(b*(c**3*f - 3*e*c**2*d - 3*c*d**2*f + e*d**3) + x*(-e*c**3 - 3*c**2*d*f + 3*e*c*d**2 + d**3*f)))/(c**2 + d**2)**3
        dL2_dVr_dVi = -(2*(b*(e*c**3 + 3*c**2*d*f - 3*e*c*d**2 - d**3*f) + x*(c**3*f - 3*e*c**2*d - 3*c*d**2*f + e*d**3)))/(c**2 + d**2)**3
        dL2_dVr_dlambda_r = (x*(d**2 - c**2) - 2*b*c*d)/(c**2 + d**2)**2
        dL2_dVr_dlambda_i =(b*(c**2 - d**2) - 2*c*d*x)/(c**2 + d**2)**2

        LAG_RL_J_stamp = (-LAG_RL_hist+ Vi*dL2_dVr_dVi + Vr*dL2_dVr_dVr + Lrl*(dL2_dVr_dlambda_r) + Lil*(dL2_dVr_dlambda_i) )
        #Trying changing lambda_r_node to Vr_node
        idx_Y = stampY(self.Vr_node, self.Vr_node, dL2_dVr_dVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vr_node, self.Vi_node, dL2_dVr_dVi, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vr_node, self.lambda_r_node, dL2_dVr_dlambda_r, Y_val, Y_row, Y_col, idx_Y)#POWER FLOW TRANSPOSE
        idx_Y = stampY(self.Vr_node, self.lambda_i_node, dL2_dVr_dlambda_i, Y_val, Y_row, Y_col, idx_Y)#POWER FLOW TRANSPOSE
        idx_J = stampJ(self.Vr_node, LAG_RL_J_stamp, J_val, J_row, idx_J)

        #VI ROW
        dIildVi = (self.Q*(Vr**2-Vi**2)-2*self.P*Vr*Vi)/(Vr**2+Vi**2)**2#-dIrldVr#d2L/dvil_dlrl#these may need to be chaged
        dIildVr = (self.P*(Vr**2-Vi**2)-2*self.Q*Vi*Vr)/(Vr**2+Vi**2)**2#dIrldVi#d2L/dvil_dIil
        LAG_IL_hist = Lrl*(dIildVi) + Lil*(dIildVr)
        ###ALL PARTIALS COME DIRECTLY FROM WOLFRAM
        dL2_dVi_dVr =-(2*(b*(e*c**3 + 3*c**2*d*f - 3*e*c*d**2 - d**3*f) + x*(c**3*f - 3*e*c**2*d - 3*c*d**2*f + e*d**3)))/(c**2 + d**2)**3
        dL2_dVi_dVi = (2*(b*(c**3*f - 3*e*c**2*d - 3*c*d**2*f + e*d**3) + x*(-e*c**3 - 3*c**2*d*f + 3*e*c*d**2 + d**3*f)))/(c**2 + d**2)**3
        dL2_dVi_dlambda_r =  (b*(c**2 - d**2) - 2*c*d*x)/(c**2 + d**2)**2
        dL2_dVi_dlambda_i = (2*b*c*d + x*(c**2 - d**2))/(c**2 + d**2)**2
        
        LAG_IL_J_stamp = (-LAG_IL_hist  + Vr*dL2_dVi_dVr + Vi*dL2_dVi_dVi + Lrl*(dL2_dVi_dlambda_r) + Lil*(dL2_dVi_dlambda_i))
        #Trying changing lambda_i_node to Vi_node
        idx_Y = stampY(self.Vi_node, self.Vr_node,dL2_dVi_dVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vi_node, self.Vi_node,dL2_dVi_dVi, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vi_node, self.lambda_r_node, dL2_dVi_dlambda_r, Y_val, Y_row, Y_col, idx_Y)#power flow transpose
        idx_Y = stampY(self.Vi_node, self.lambda_i_node, dL2_dVi_dlambda_i, Y_val, Y_row, Y_col, idx_Y)#power flow transpose
        idx_J = stampJ(self.Vi_node, LAG_IL_J_stamp, J_val, J_row, idx_J)
        
        
        
        #####ORIGANAL MANUALE APPROACH FOR IMPLEMENTING
        # ##breaking up the very long equations
        # LBR_vr1 = (Lil*(2*self.Q*Vr-2*Vi*self.P))/(Vr**2+Vi**2)**2
        # LBR_vr2 = ((4*Vr*Lil)*(self.Q*(Vr**2-Vi**2)-2*Vr*Vi*self.P))/(Vr**2+Vi**2)**3##SEEM TO BE PROBLEM HERE
        # LBR_vr3 = (Lrl*(-2*self.Q*Vi-2*Vr*self.P))/(Vr**2+Vi**2)**2
        # LBR_vr4 = ((4*Vr*Lrl)*(self.P*(Vi**2-Vr**2)-2*Vr*Vi*self.Q))/(Vr**2+Vi**2)**3#SEEMS TO BE PROBLEM HERE
        # d2L_d2vrl = LBR_vr1 - LBR_vr2 + LBR_vr3 - LBR_vr4 #(3)

        # LBR_vi1 = (Lil*(-2*self.Q*Vi-2*Vr*self.P))/(Vr**2+Vi**2)**2
        # LBR_vi2 = ((4*Vi*Lil)*(self.Q*(Vr**2-Vi**2)-2*Vr*Vi*self.P))/(Vr**2+Vi**2)**3
        # LBR_vi3 = (Lrl*(2*self.P*Vi-2*Vr*self.Q))/(Vr**2+Vi**2)**2
        # LBR_vi4 = ((4*Vi*Lrl)*(self.P*(Vi**2-Vr**2)-2*Vr*Vi*self.Q))/(Vr**2+Vi**2)**3
        # d2L_dvrldvil = LBR_vi1 - LBR_vi2 + LBR_vi3 - LBR_vi4#(4)

        # LAG_RL_J_stamp = (LAG_RL_hist- Vi*d2L_dvrldvil - Vr*d2L_d2vrl - Lrl*(dIrldVr) - Lil*(dIrldVi) )
        # #Trying changing lambda_r_node to Vr_node
        # idx_Y += stampY(self.Vr_node, self.Vr_node, d2L_d2vrl, Y_val, Y_row, Y_col, idx_Y)
        # idx_Y += stampY(self.Vr_node, self.Vi_node, d2L_dvrldvil, Y_val, Y_row, Y_col, idx_Y)
        # idx_Y += stampY(self.Vr_node, self.lambda_r_node, dIrldVr, Y_val, Y_row, Y_col, idx_Y)#POWER FLOW TRANSPOSE
        # idx_Y += stampY(self.Vr_node, self.lambda_i_node, dIrldVi, Y_val, Y_row, Y_col, idx_Y)#POWER FLOW TRANSPOSE
        # idx_J += stampJ(self.Vr_node, LAG_RL_J_stamp, J_val, J_row, idx_J)
        #########
        ##NOW STAMPING LAMBDA_IL ROW
        # Iig_hist = (self.P*Vi-self.Q*Vr)/(Vr**2+Vi**2)
        # dIildVi = (self.Q*(Vr**2-Vi**2)-2*self.P*Vr*Vi)/(Vr**2+Vi**2)**2#-dIrldVr#d2L/dvil_dlrl#these may need to be chaged
        # dIildVr = (self.P*(Vr**2-Vi**2)-2*self.Q*Vi*Vr)/(Vr**2+Vi**2)**2#dIrldVi#d2L/dvil_dIil

        # ##breaking up the very long equations
        # LBI_vi1 = (Lil*(-2*self.Q*Vr-2*Vi*self.P))/(Vr**2+Vi**2)**2
        # LBI_vi2 = ((4*Vi*Lil)*(self.P*(Vr**2-Vi**2)-2*Vr*Vi*self.Q))/(Vr**2+Vi**2)**3
        # LBI_vi3 = (Lrl*(-2*self.Q*Vi-2*Vr*self.P))/(Vr**2+Vi**2)**2
        # LBI_vi4 = ((4*Vi*Lrl)*(self.Q*(Vr**2-Vi**2)-2*Vr*Vi*self.P))/(Vr**2+Vi**2)**3
        # d2L_d2vil = LBI_vi1 - LBI_vi2 + LBI_vi3 - LBI_vi4

        # LBI_vr1 = (Lil*(2*self.P*Vr-2*Vi*self.Q))/(Vr**2+Vi**2)**2
        # LBI_vr2 = ((4*Vr*Lil)*(self.P*(Vr**2-Vi**2)-2*Vr*Vi*self.Q))/(Vr**2+Vi**2)**3
        # LBI_vr3 = (Lrl*(2*self.Q*Vr-2*Vi*self.P))/(Vr**2+Vi**2)**2
        # LBI_vr4 = ((4*Vr*Lrl)*(self.Q*(Vr**2-Vi**2)-2*Vr*Vi*self.P))/(Vr**2+Vi**2)**3
        # d2L_dvildvrl = LBI_vr1 - LBI_vr2 + LBI_vr3 - LBI_vr4

        # LAG_IL_hist = Lrl*(dIildVi) + Lil*(dIildVr)

        # LAG_IL_J_stamp = (LAG_IL_hist  - Vr*d2L_dvildvrl - Vi*d2L_d2vil - Lrl*(dIildVr) - Lil*(dIildVi))
        # #Trying changing lambda_i_node to Vi_node
        # idx_Y += stampY(self.Vi_node, self.Vr_node,d2L_dvildvrl, Y_val, Y_row, Y_col, idx_Y)
        # idx_Y += stampY(self.Vi_node, self.Vi_node,d2L_d2vil , Y_val, Y_row, Y_col, idx_Y)
        # idx_Y += stampY(self.Vi_node, self.lambda_r_node, dIildVi, Y_val, Y_row, Y_col, idx_Y)#power flow transpose
        # idx_Y += stampY(self.Vi_node, self.lambda_i_node, dIildVr, Y_val, Y_row, Y_col, idx_Y)#power flow transpose
        # idx_J += stampJ(self.Vi_node, LAG_IL_J_stamp, J_val, J_row, idx_J)
        

        return (idx_Y, idx_J)

    def calc_residuals(self, resid, V):
        P = self.P
        Vr = V[self.Vr_node]
        Vi = V[self.Vi_node]
        Q = self.Q
        resid[self.Vr_node] += (P*Vr+Q*Vi)/(Vr**2+Vi**2)
        resid[self.Vi_node] += (P*Vi-Q*Vr)/(Vr**2+Vi**2)
