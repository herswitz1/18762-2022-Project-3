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
        # Ifr = V[self.Ifr_node]
        # Ifi = V[self.Ifi_node]

        Irg_hist = (self.P*Vr+self.Q*Vi)/(Vr**2+Vi**2) 
        dIrldVr = ((self.P*(Vi**2-Vr**2) - 2*self.Q*Vr*Vi)/(Vr**2+Vi**2)**2) 
        dIrldVi = ((self.Q*(Vr**2-Vi**2) - 2*self.P*Vr*Vi)/(Vr**2+Vi**2)**2) 
        
        Vr_J_stamp = -Irg_hist + (dIrldVr)*Vr + (dIrldVi)*Vi #+Ifr*dIrldIfr
        
        idx_Y = stampY(self.Vr_node, self.Vr_node, dIrldVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vr_node, self.Vi_node, dIrldVi, Y_val, Y_row, Y_col, idx_Y)
        #idx_Y = stampY(self.Vr_node, self.Ifr_node, dIrldIfr, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.Vr_node, Vr_J_stamp, J_val, J_row, idx_J)

        Iig_hist = (self.P*Vi-self.Q*Vr)/(Vr**2+Vi**2) #+Ifi
        dIildVi = -dIrldVr
        dIildVr = dIrldVi
        #dIildIfi = 1
        Vi_J_stamp = -Iig_hist + (dIildVr)*Vr + (dIildVi)*Vi #+Ifi*dIildIfi

        idx_Y = stampY(self.Vi_node, self.Vr_node, dIildVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.Vi_node, self.Vi_node, dIildVi, Y_val, Y_row, Y_col, idx_Y)
        #idx_Y = stampY(self.Vi_node, self.Ifi_node, dIildIfi, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.Vi_node, Vi_J_stamp, J_val, J_row, idx_J)

        return (idx_Y, idx_J)

    def stamp_dual(self,V, Y_val, Y_row, Y_col, J_val, J_row, idx_Y, idx_J):
        # You need to implement this.
        Vr = V[self.Vr_node]
        Vi = V[self.Vi_node]
        Lrl = V[self.lambda_r_node]
        Lil = V[self.lambda_i_node]
        #NOT SURE IF I AM USING THESE RIGHT
        #Ifr = V[self.Ifr_node]
        #Ifi = V[self.Ifi_node]

        Irg_hist = (self.P*Vr+self.Q*Vi)/(Vr**2+Vi**2) #+ Ifr
        dIrldVr = ((self.P*(Vi**2-Vr**2) - 2*self.Q*Vr*Vi)/(Vr**2+Vi**2)**2) #this is d^2L/dVrl_dLrl(1_)
        dIrldVi = ((self.Q*(Vr**2-Vi**2) - 2*self.P*Vr*Vi)/(Vr**2+Vi**2)**2) #this is d^2L/dVrl_dLil(2)
        ##breaking up the very long equations
        LBR_vr1 = (Lil*(2*self.Q*Vr-2*Vi*self.P))/(Vr**2+Vi**2)**2
        LBR_vr2 = ((4*Vr*Lil)*(self.Q*(Vr**2-Vi**2)-2*Vr*Vi*self.P))/(Vr**2+Vi**2)**3
        LBR_vr3 = (Lrl*(-2*self.Q*Vi-2*Vr*self.P))/(Vr**2+Vi**2)**2
        LBR_vr4 = ((4*Vr*Lrl)*(self.P*(Vi**2-Vr**2)-2*Vr*Vi*self.Q))/(Vr**2+Vi**2)**3
        d2L_d2vrl = LBR_vr1 - LBR_vr2 + LBR_vr3 - LBR_vr4 #(3)

        LBR_vi1 = (Lil*(2*self.Q*Vi-2*Vr*self.P))/(Vr**2+Vi**2)**2
        LBR_vi2 = ((4*Vi*Lil)*(self.Q*(Vr**2-Vi**2)-2*Vr*Vi*self.P))/(Vr**2+Vi**2)**3
        LBR_vi3 = (Lrl*(2*self.P*Vi-2*Vr*self.Q))/(Vr**2+Vi**2)**2
        LBR_vi4 = ((4*Vi*Lrl)*(self.P*(Vi**2-Vr**2)-2*Vr*Vi*self.Q))/(Vr**2+Vi**2)**3
        d2L_dvrldvil = LBR_vi1 - LBR_vi2 + LBR_vi3 - LBR_vi4#(4)

        LAG_RL_hist = Lrl*(dIrldVr) + Lil*(dIrldVi)

        LAG_RL_J_stamp = LAG_RL_hist - Lrl*(dIrldVr) - Lil*(dIrldVi) - Vi*d2L_dvrldvil - Vr*d2L_d2vrl

        idx_Y = stampY(self.lambda_r_node, self.Vr_node, d2L_d2vrl, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_r_node, self.Vi_node, d2L_dvrldvil, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_r_node, self.lambda_r_node, dIrldVr, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_r_node, self.lambda_i_node, dIrldVi, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.lambda_r_node, LAG_RL_J_stamp, J_val, J_row, idx_J)
        #########
        ##NOW STAMPING LAMBDA_IL ROW
        dIildVi = -dIrldVr#d2L/dvil_dlrl
        dIildVr = dIrldVi#d2L/dvil_dIil

        ##breaking up the very long equations
        LBI_vi1 = (Lil*(-2*self.Q*Vr-2*Vi*self.P))/(Vr**2+Vi**2)**2
        LBI_vi2 = ((4*Vi*Lil)*(self.P*(Vr**2-Vi**2)-2*Vr*Vi*self.Q))/(Vr**2+Vi**2)**3
        LBI_vi3 = (Lrl*(-2*self.Q*Vi-2*Vr*self.P))/(Vr**2+Vi**2)**2
        LBI_vi4 = ((4*Vi*Lrl)*(self.Q*(Vr**2-Vi**2)-2*Vr*Vi*self.P))/(Vr**2+Vi**2)**3
        d2L_d2vil = LBI_vi1 - LBI_vi2 + LBI_vi3 - LBI_vi4

        LBI_vr1 = (Lil*(2*self.P*Vr-2*Vi*self.Q))/(Vr**2+Vi**2)**2
        LBI_vr2 = ((4*Vr*Lil)*(self.P*(Vr**2-Vi**2)-2*Vr*Vi*self.Q))/(Vr**2+Vi**2)**3
        LBI_vr3 = (Lrl*(2*self.Q*Vr-2*Vi*self.P))/(Vr**2+Vi**2)**2
        LBI_vr4 = ((4*Vr*Lrl)*(self.Q*(Vr**2-Vi**2)-2*Vr*Vi*self.P))/(Vr**2+Vi**2)**3
        d2L_dvildvrl = LBI_vr1 - LBI_vr2 + LBI_vr3 - LBI_vr4

        LAG_IL_hist = Lrl*(dIildVr) + Lil*(dIildVi)

        LAG_IL_J_stamp = LAG_IL_hist - Lrl*(dIildVr) - Lil*(dIildVi) - Vr*d2L_dvildvrl - Vi*d2L_d2vil

        idx_Y = stampY(self.lambda_i_node, self.Vr_node,d2L_dvildvrl, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_i_node, self.Vi_node,d2L_d2vil , Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_i_node, self.lambda_r_node, dIildVi, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_i_node, self.lambda_i_node, dIildVr, Y_val, Y_row, Y_col, idx_Y)
        idx_J = stampJ(self.lambda_i_node, LAG_IL_J_stamp, J_val, J_row, idx_J)
        

        return (idx_Y, idx_J)

    def calc_residuals(self, resid, V):
        P = self.P
        Vr = V[self.Vr_node]
        Vi = V[self.Vi_node]
        Q = self.Q
        resid[self.Vr_node] += (P*Vr+Q*Vi)/(Vr**2+Vi**2)
        resid[self.Vi_node] += (P*Vi-Q*Vr)/(Vr**2+Vi**2)
