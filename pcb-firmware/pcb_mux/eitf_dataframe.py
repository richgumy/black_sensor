class EITFDataFrame:
    """
    A structure containing ALL useful information from a force compression test sequence on a piezoresistive disk domain for use with EIT
    """
    def __init__(self,
                filename,
                time_date,
                v_data_V,
                i_data_A,
                t_data_s,
                i_src_A,
                v_max_V,
                nplc,
                eit_cycles,
                r_adj_ohm,
                PiezoResSample,
                num_elecs=16,
                strain=None, # %
                t_hold_s=None,
                v_push=None,
                f_data_N=[],
                x_data_mm=[],
                y_data_mm=[],
                z_data_mm=[],
                z_mesh=[],
                z_mesh_locs=[],
                z_mesh_datetime=None,
                r_adj_error=None,
                v_max_error=None):
        # inputs
        self.filename = filename # filename e.g. filename = sample_name
        self.time_date = time_date # UTC time and date for taken just before data gathering
        self.v_data_V = v_data_V # voltage data
        self.i_data_A = i_data_A # current data (actual current driven)
        self.t_data_s = t_data_s # time data
        self.i_src_A = i_src_A # constant current source driven for EIT
        self.v_max_V = v_max_V # max drive voltage from the SMU Isrc
        self.nplc = nplc # integration time of the SMU in numbe rof power line cycles
        self.cycles = cycles # i.e. number of frames that can be reconstructed from 
        self.num_elecs = num_elecs # number of electrodes
        self.r_adj_ohm = r_adj_ohm # inter-electrode resistances
        self.PiezoResSample = PiezoResSample # must be a PiezoResSample class
        # optional inputs 
        self.f_data_N = f_data_N # force data
        self.xyz_data_mm = xyz_data_mm # arrays for tool center point x,y,z coordinates referenced from the center surface of the DUT
        self.r_adj_error = r_adj_error
        self.v_max_error = v_max_error

    def __str__(self):
        return f"Class for EIT scan data for '{self.PiezoResSample.sample_name}' \n *use .disp_all() to display all data"
    
    def disp_attr(self):
        print(self.PiezoResSample.__dict__)
        for attr in self.__dict__:
            print(f"{attr} : {self.__dict__[attr]}")
        

class PiezoResSample:
    """
    A class structure for holding piezoresistive disk sample characterisitics
    COULD DO: generalise for more shapes 
    """
    def __init__(self,
                sample_name,
                th_dim_mm,
                dia_dim_mm,
                fab_date='unknown'):

        self.sample_name = sample_name # e.g. CBSR_9p_1 = carbon black 9wt% silicone sample run #1 
        self.th_dimension = th_dim_mm # disk thickness
        self.dia_dimension = dia_dim_mm # disk diameter
        self.fab_date = fab_date # date fabricated




