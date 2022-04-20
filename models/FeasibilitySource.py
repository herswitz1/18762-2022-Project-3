from __future__ import division
import numpy as np
from models.Buses import Buses
from models.Buses import Buses
from scripts.stamp_helpers import *
from models.global_vars import global_vars

class FeasibilitySource:

    def __init__(self,
                 Bus):
        """Initialize slack bus in the power grid.

        Args:
            Bus (int): the bus number corresponding to this set of feasibility currents
        """
        self.Bus = Bus
        
        self.Ir_init = 0
        self.Ii_init = 0
        

    def assign_nodes(self,bus):
        """Assign the additional slack bus nodes for a slack bus.
        Args:
            You decide :)
        Returns:
            None
        """
        # TODO: You decide how to implement variables for the feasibility injections
        #I thiink there should be 9 
        ###SOMETHING FEELS OFF HERE not sure if I am assigning nodes or adding more
        self.Ifr_node = bus[Buses.bus_key_[self.Bus]].node_Vr
        self.Ifi_node = bus[Buses.bus_key_[self.Bus]].node_Vi
        #self.Ifq_PV = bus[Buses.bus_key_[self.Bus]].node_Vi
        ###START OFF WITH ASSUMING SLACK DOES NOT HAVE INFEASABLITILY
        self.lambda_Ifr_node = Buses._node_index.__next__()
        self.lambda_Ifi_node = Buses._node_index.__next__()
        pass

    def stamp(self, V, Y_val, Y_row, Y_col, J_val, J_row, idx_Y, idx_J):
        # You need to implement this.
        #needs to add a one stamp for each row with
        idx_Y = stampY(self.Ifr_node, self.lambda_Ifr_node, 1, Y_val, Y_row, Y_col, idx_Y)#power flow
        idx_Y = stampY(self.Ifi_node, self.lambda_Ifi_node, 1, Y_val, Y_row, Y_col, idx_Y)#power flow
        idx_Y = stampY(self.lambda_Ifr_node, self.Ifr_node, 1, Y_val, Y_row, Y_col, idx_Y)#hessen
        idx_Y = stampY(self.lambda_Ifi_node, self.Ifi_node, 1, Y_val, Y_row, Y_col, idx_Y)#hessen
        

        return (idx_Y, idx_J)

    def stamp_dual(self,V, Y_val, Y_row, Y_col, J_val, J_row, idx_Y, idx_J):
        # You need to implement this.
        idx_Y = stampY(self.lambda_Ifr_node, self.lambda_Ifr_node, 2, Y_val, Y_row, Y_col, idx_Y)
        idx_Y = stampY(self.lambda_Ifi_node, self.lambda_Ifi_node, 2, Y_val, Y_row, Y_col, idx_Y)
        return (idx_Y, idx_J)
        #pass
