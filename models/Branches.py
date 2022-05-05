from __future__ import division
from itertools import count
from scripts.stamp_helpers import *
from models.Buses import Buses

class Branches:
    _ids = count(0)

    def __init__(self,
                 from_bus,
                 to_bus,
                 r,
                 x,
                 b,
                 status,
                 rateA,
                 rateB,
                 rateC):
        """Initialize a branch in the power grid.

        Args:
            from_bus (int): the bus number at the sending end of the branch.
            to_bus (int): the bus number at the receiving end of the branch.
            r (float): the branch resistance
            x (float): the branch reactance
            b (float): the branch susceptance
            status (bool): indicates if the branch is online or offline
            rateA (float): The 1st rating of the line.
            rateB (float): The 2nd rating of the line
            rateC (float): The 3rd rating of the line.
        """
        self.from_bus = from_bus
        self.to_bus = to_bus
        self.r = r
        self.x = x
        self.b = b
        self.status = bool(status)
        self.rateA = rateA
        self.rateB = rateB
        self.rateC = rateC

        if abs(self.x) < 1e-6:
            if self.x < 0:
                self.x = -1e-6
            else:
                self.x = 1e-6

        # convert to G and B
        self.G_pu = self.r/(self.r**2+self.x**2)
        self.B_pu= -self.x/(self.r**2+self.x**2)


        self.id = self._ids.__next__()

    def assign_indexes(self, bus):
        self.Vr_from_node = bus[Buses.bus_key_[self.from_bus]].node_Vr
        self.Vi_from_node = bus[Buses.bus_key_[self.from_bus]].node_Vi
        self.Vr_to_node = bus[Buses.bus_key_[self.to_bus]].node_Vr
        self.Vi_to_node = bus[Buses.bus_key_[self.to_bus]].node_Vi

        #duals for lambda
        self.lambda_r_from = bus[Buses.bus_key_[self.from_bus]].lambda_r_node
        self.lambda_i_from = bus[Buses.bus_key_[self.from_bus]].lambda_i_node
        self.lambda_r_to = bus[Buses.bus_key_[self.to_bus]].lambda_r_node
        self.lambda_i_to = bus[Buses.bus_key_[self.to_bus]].lambda_i_node         

        #NEED TO TRY SWAPPING X AND LAMBDA ROWS FOR PRIMAL AND DUAL STAMPS
    def stamp(self, V, Ylin_val, Ylin_row, Ylin_col, Jlin_val, Jlin_row, idx_Y, idx_J,Tx):
        if not self.status:
            return (idx_Y, idx_J)
        Homoto_G = self.G_pu*(20*(1-Tx)) + self.G_pu*Tx#ATTEMPT AT IMPLEMENTING TX STEPPING: IF Tx IS SET TO 1 THEN FUNCTION LIKE NORMAL
        Homoto_B = self.B_pu*(20*(1-Tx)) +self.B_pu*Tx
        Homoto_SH = self.b*(20*(1-Tx)) +self.b*Tx
        # Line Bs
        idx_Y = stampY(self.lambda_r_from, self.Vi_from_node, -Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_r_from, self.Vi_to_node, Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_i_from, self.Vr_from_node, Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_i_from, self.Vr_to_node, -Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_r_to, self.Vi_to_node, -Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_r_to, self.Vi_from_node, Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_i_to, self.Vr_to_node, Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_i_to, self.Vr_from_node, -Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        
        # Line Shunts
        idx_Y = stampY(self.lambda_r_from, self.Vi_from_node, -Homoto_SH/2, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_i_from, self.Vr_from_node, Homoto_SH/2, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_r_to, self.Vi_to_node, -Homoto_SH/2, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_i_to, self.Vr_to_node, Homoto_SH/2, Ylin_val, Ylin_row, Ylin_col, idx_Y)

        if self.r == 0:
            return (idx_Y, idx_J)

        # Line Gs
        idx_Y = stampY(self.lambda_r_from, self.Vr_from_node, Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_i_from, self.Vi_from_node, Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_r_to, self.Vr_to_node, Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_i_to, self.Vi_to_node, Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_r_from, self.Vr_to_node, -Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_i_from, self.Vi_to_node, -Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_r_to, self.Vr_from_node, -Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.lambda_i_to, self.Vi_from_node, -Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        return (idx_Y, idx_J)

    def stamp_dual(self,V, Ylin_val, Ylin_row, Ylin_col, Jlin_val, Jlin_row, idx_Y, idx_J,Tx):
        # You need to implement this.(IMPLEMENTING TRANSPOSE OF THE ABOVE)
        if not self.status:
            return (idx_Y, idx_J)
        Homoto_G = self.G_pu*(20*(1-Tx)) + self.G_pu*Tx #ATTEMPT AT IMPLEMENTING TX STEPPING: IF Tx IS SET TO 1 THEN FUNCTION LIKE NORMAL
        Homoto_B = self.B_pu*(20*(1-Tx)) +self.B_pu*Tx
        Homoto_SH = self.b*(20*(1-Tx)) +self.b*Tx
        # Line Bs lambda
        idx_Y = stampY(self.Vr_from_node, self.lambda_i_from, Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vr_from_node, self.lambda_i_to, -Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vi_from_node, self.lambda_r_from, -Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vi_from_node, self.lambda_r_to, Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vr_to_node, self.lambda_i_to, Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vr_to_node, self.lambda_i_from, -Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vi_to_node, self.lambda_r_to, -Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vi_to_node, self.lambda_r_from, Homoto_B, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        
        # Line Shunts lambda
        idx_Y = stampY(self.Vr_from_node, self.lambda_i_from, Homoto_SH/2, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vi_from_node, self.lambda_r_from, -Homoto_SH/2, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vr_to_node, self.lambda_i_to, Homoto_SH/2, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vi_to_node, self.lambda_r_to, -Homoto_SH/2, Ylin_val, Ylin_row, Ylin_col, idx_Y)

        if self.r == 0:
            return (idx_Y, idx_J)

        # Line Gs lambda
        idx_Y = stampY(self.Vr_from_node, self.lambda_r_from, Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vi_from_node, self.lambda_i_from, Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vr_to_node, self.lambda_r_to, Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vi_to_node, self.lambda_i_to, Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vr_from_node, self.lambda_r_to, -Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vi_from_node, self.lambda_i_to, -Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vr_to_node, self.lambda_r_from, -Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
        idx_Y = stampY(self.Vi_to_node, self.lambda_i_from, -Homoto_G, Ylin_val, Ylin_row, Ylin_col, idx_Y)
    
        return (idx_Y, idx_J)
        

    def calc_residuals(self, resid, V):
        Vr_from = V[self.Vr_from_node]
        Vr_to = V[self.Vr_to_node]
        Vi_from = V[self.Vi_from_node]
        Vi_to = V[self.Vi_to_node]
        
        resid[self.Vr_from_node] += (Vr_from-Vr_to)*self.G_pu - (Vi_from-Vi_to)*self.B_pu - Vi_from*self.b/2
        resid[self.Vr_to_node] += (Vr_to-Vr_from)*self.G_pu - (Vi_to-Vi_from)*self.B_pu - Vi_to*self.b/2
        resid[self.Vi_from_node] += (Vr_from-Vr_to)*self.B_pu + (Vi_from-Vi_to)*self.G_pu + Vr_from*self.b/2
        resid[self.Vi_to_node] += (Vr_to-Vr_from)*self.B_pu + (Vi_to-Vi_from)*self.G_pu + Vr_to*self.b/2
