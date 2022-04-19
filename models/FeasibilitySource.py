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
        

    def assign_nodes(self,bus,slack):
        """Assign the additional slack bus nodes for a slack bus.
        Args:
            You decide :)
        Returns:
            None
        """
        # TODO: You decide how to implement variables for the feasibility injections
        #I thiink there should be 9 
        ###SOMETHING FEELS OFF HERE not sure if I am assigning nodes or adding more
        self.Ifr_PQ = bus[Buses.bus_key_[self.Bus]].node_Vr
        self.Ifi_PQ = bus[Buses.bus_key_[self.Bus]].node_Vi
        self.Ifr_PV = bus[Buses.bus_key_[self.Bus]].node_Vr
        self.Ifi_PV = bus[Buses.bus_key_[self.Bus]].node_Vi
        self.Ifq_PV = bus[Buses.bus_key_[self.Bus]].node_Vi
        self.Ifr_slack_Vr = bus[Buses.bus_key_[self.Bus]].node_Vr
        self.Ifi_slack_Vi = bus[Buses.bus_key_[self.Bus]].node_Vi
        self.Ifr_slack_Ir = bus[Buses.bus_key_[self.Bus]].Slack_Ir_node
        self.Ifi_slack_Ii = bus[Buses.bus_key_[self.Bus]].Slack_Ii_node
        pass

    def stamp(self, V, Y_val, Y_row, Y_col, J_val, J_row, idx_Y, idx_J):
        # You need to implement this.
        #needs to add a one stamp for each row with 
        return (idx_Y, idx_J)

    def stamp_dual(self):
        # You need to implement this.
        pass
