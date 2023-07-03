class ert_frame:
    """
    a structure for holding raw ERT voltage data containing useful information from an EIT scan.
    class structure:
    eit_frame------[filename]
                L--[v_data_V]
                L--[i_data_A]
                L--[t_data_s] 
                L--[i_src_A]
                L--[v_max_V]
                L--[num_elecs]
                L--[r_data_ohm(v_data_V, i_src_A)]
                
    """
    def __init__(self,
                 filename,
                 raw_data,# incl. v, i & t data.
                 i_src_A,
                 v_max_V,
                 num_elecs):
        self.filename = filename
        self.raw_data = raw_data
        self.i_src_A = i_src_A
        self.v_max_V = v_max_V
        self.num_elecs = num_elecs
        self.r_adj = self.get_r_adj_data()

    def get_r_adj_data(self):
        # Iterates through voltage data and determines all adjacent electrode resistances
        num_cycles = len(ert_v_data_V)//(num_elecs**2)
        R_elec_arr = np.zeros(((num_elecs*num_cycles),1))
        cycle = 0
        for i in range(len(ert_v_data_V)):
            if not (i % (num_elecs+1)):
                cycle = i // num_elecs**2
                R_elec_arr[i//(num_elecs)] = abs(ert_v_data_V[i+cycle])/i_src_A
        R_elec_arr_ = np.reshape(R_elec_arr,(num_cycles,num_elecs))
        R_elec_arr_[1][0] = R_elec_arr_[0][0]
        for cycle in range(2,num_cycles):
            R_elec_arr_[cycle][1:cycle] = R_elec_arr_[cycle][0:cycle-1]
            R_elec_arr_[cycle][0] = R_elec_arr_[0][0]
        R_elec_arr_[-1][-1] = R_elec_arr_[-2][-1]
        return R_elec_arr_