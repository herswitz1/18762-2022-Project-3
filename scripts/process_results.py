import numpy as np

def process_results(v, bus, slack, generator):
    print("BUS VOLTAGES:")
    feasability_sources_val = []
    feasability_sources_loc = []
    for ele in bus:
        # PQ bus
        if ele.Type == 1:
            Vr = v[ele.node_Vr]
            Vi = v[ele.node_Vi]
            Vmag = np.sqrt(Vr**2 + Vi**2)
            Vth = np.arctan2(Vi, Vr) * 180/np.pi
            print("%d, Vmag: %.3f p.u., Vth: %.3f deg" % (ele.Bus, Vmag, Vth))
            # feasability_sources_val.append(v[ele.Ifr_node])
            # feasability_sources_loc.append(ele.Ifr_node)
            # feasability_sources_val.append(v[ele.Ifi_node])
            # feasability_sources_loc.append(ele.Ifi_node)
            # feasability_sources_val.append(v[ele.lambda_Ifr_node])
            # feasability_sources_loc.append(ele.lambda_Ifr_node)
            # feasability_sources_val.append(v[ele.lambda_Ifi_node])
            # feasability_sources_loc.append(ele.lambda_Ifi_node)
        # PV bus
        elif ele.Type == 2:
            Vr = v[ele.node_Vr]
            Vi = v[ele.node_Vi]
            Vmag = np.sqrt(Vr**2 + Vi**2)
            Vth = np.arctan2(Vi, Vr) * 180.0/np.pi
            Qg = v[ele.node_Q]*100
            print("%d: Vmag: %.3f p.u., Vth: %.3f deg, Qg: %.3f MVAr" % (ele.Bus, Vmag, Vth, Qg))
            # feasability_sources_val.append(v[ele.Ifr_node])
            # feasability_sources_loc.append(ele.Ifr_node)
            # feasability_sources_val.append(v[ele.Ifi_node])
            # feasability_sources_loc.append(ele.Ifi_node)
            # feasability_sources_val.append(v[ele.lambda_Ifr_node])
            # feasability_sources_loc.append(ele.lambda_Ifr_node)
            # feasability_sources_val.append(v[ele.lambda_Ifi_node])
            # feasability_sources_loc.append(ele.lambda_Ifi_node)
        elif ele.Type == 3:
            Vr = v[ele.node_Vr]
            Vi = v[ele.node_Vi]
            Vmag = np.sqrt(Vr**2 + Vi**2)
            Vth = np.arctan2(Vi, Vr) * 180/np.pi
            Pg, Qg = slack[0].calc_slack_PQ(v)
            print("SLACK: %d, Vmag: %.3f p.u., Vth: %.3f deg, Pg: %.3f MW, Qg: %.3f MVar" % (ele.Bus, Vmag, Vth, Pg*100, Qg*100))
            # feasability_sources_val.append(v[ele.Ifr_node])
            # feasability_sources_loc.append(ele.Ifr_node)
            # feasability_sources_val.append(v[ele.Ifi_node])
            # feasability_sources_loc.append(ele.Ifi_node)
            # feasability_sources_val.append(v[ele.lambda_Ifr_node])
            # feasability_sources_loc.append(ele.lambda_Ifr_node)
            # feasability_sources_val.append(v[ele.lambda_Ifi_node])
            # feasability_sources_loc.append(ele.lambda_Ifi_node)
    for ele in bus:
        if ele.Type == 1:
            Ifr = v[ele.Ifr_node]
            Ifi = v[ele.Ifi_node]
            lambda_Ifr = v[ele.lambda_Ifr_node]
            lambda_Ifi = v[ele.lambda_Ifr_node]
