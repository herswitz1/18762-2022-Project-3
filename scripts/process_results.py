import numpy as np

def process_results(v, bus, slack, generator):
    print("BUS VOLTAGES:")
    feasability_sources_real = {}
    feasability_sources_imag = {}
    for ele in bus:
        # PQ bus
        if ele.Type == 1:
            Vr = v[ele.node_Vr]
            Vi = v[ele.node_Vi]
            Vmag = np.sqrt(Vr**2 + Vi**2)
            Vth = np.arctan2(Vi, Vr) * 180/np.pi
            print("%d, Vmag: %.3f p.u., Vth: %.3f deg" % (ele.Bus, Vmag, Vth))
            
        # PV bus
        elif ele.Type == 2:
            Vr = v[ele.node_Vr]
            Vi = v[ele.node_Vi]
            Vmag = np.sqrt(Vr**2 + Vi**2)
            Vth = np.arctan2(Vi, Vr) * 180.0/np.pi
            Qg = v[ele.node_Q]*100
            print("%d: Vmag: %.3f p.u., Vth: %.3f deg, Qg: %.3f MVAr" % (ele.Bus, Vmag, Vth, Qg))
            
        elif ele.Type == 3:
            Vr = v[ele.node_Vr]
            Vi = v[ele.node_Vi]
            Vmag = np.sqrt(Vr**2 + Vi**2)
            Vth = np.arctan2(Vi, Vr) * 180/np.pi
            Pg, Qg = slack[0].calc_slack_PQ(v)
            print("SLACK: %d, Vmag: %.3f p.u., Vth: %.3f deg, Pg: %.3f MW, Qg: %.3f MVar" % (ele.Bus, Vmag, Vth, Pg*100, Qg*100))
            
    print("INFEASABILITY CURRENTS:")
    for ele in bus:
        if ele.Type == 1:
            Ifr = v[ele.lambda_r_node]
            Ifi = v[ele.lambda_i_node]
            #print("%d, Real injection: %.3f, Imaginary Injection: %.3f" % (ele.Bus, Ifr, Ifi))
            feasability_sources_real[np.abs(Ifr)] = ele.Bus
            feasability_sources_imag[np.abs(Ifi)] =ele.Bus
        if ele.Type == 2:
            Ifr = v[ele.lambda_r_node]
            Ifi = v[ele.lambda_i_node]
            Ifq = v[ele.lambda_q_node]
            #print("%d, Real injection: %.3f, Imaginary Injection: %.3f, Q injection: %.3f" % (ele.Bus, Ifr, Ifi,Ifq))
            feasability_sources_real[np.abs(Ifr)] = ele.Bus
            feasability_sources_imag[np.abs(Ifi)] =ele.Bus
        if ele.Type == 3:
            Ifr = v[ele.lambda_r_node]
            Ifi = v[ele.lambda_i_node]
            #print("Slack: %d, Real injection: %.3f, Imaginary Injection: %.3f" % (ele.Bus, Ifr, Ifi))
            feasability_sources_real[np.abs(Ifr)] = ele.Bus
            feasability_sources_imag[np.abs(Ifi)] =ele.Bus
    FS_r = sorted(feasability_sources_real, reverse = True)
    FS_i = sorted(feasability_sources_imag, reverse = True)
    #print(FS_r)
    #print(FS_i)
    print("Three largest real current injections")
    for i in range(3):
        print("Bus: %d, largest real current inject: %.3f" %(feasability_sources_real[FS_r[i]], FS_r[i]))
    print("Three largest imaginary current injections")
    for i in range(3):
        print("Bus: %d, largest image current inject: %.3f" %(feasability_sources_imag[FS_i[i]], FS_i[i]))

