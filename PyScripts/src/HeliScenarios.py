"""
DEPRECATED - TO BE MODIFIED
"""

import pandas as pd
import numpy as np
from dymola.dymola_interface import DymolaInterface # type: ignore
from dymola.dymola_exception import DymolaException # type: ignore

dymola_exe_directory = "C:\\Program Files\\Dymola 2018 FD01\\bin64\\Dymola.exe"
dymola_package_directory="C:\\Users\kikow\\Dropbox\\Thesis - Kiko Guimaraes\\ModelicaCodeLocalChanges\\DynTherM\\package.mo"

class FuselagePanelTest: 
    """Model of Fuselage for testing"""
    def __init__(self, ext_sw, case_dir="DynTherM.Systems.Helicopter.Tests.Fuselage",
                 dymola_exe_dir=dymola_exe_directory,
                 package_dir=dymola_package_directory,
                 excel_file='HeliTestCases.xlsx'):
        # Instantiate the Dymola interface and start Dymola
        self.dymola = DymolaInterface(dymola_exe_dir)
        self.dymola.openModel(path=package_dir)
        self.ext_sw = ext_sw
        self.case_dir = case_dir
        # Instantiate the pandas dataframe to write results
        self.df = pd.read_excel(excel_file, sheet_name='Template')
        self.excel_file = excel_file
        # Initialize result vectors
        self.Q = np.array([])

    def run(self):
        input_keys = ["radiationInput.E","radiationInput.theta"]
        output_keys = ["fuselagePanel.thermalPort.Q_flow"]
        input_values = [self.ext_sw.E[0], self.ext_sw.theta[0]]
        result = self.dymola.simulateExtendedModel(self.case_dir, 0.0, 10.0, 0, 0.0, "Dassl", 0.0001, 0.0, "attempt", input_keys, input_values, output_keys, True)
        if result[0]:
            self.Q = result[1][0]
        else:
            log = self.dymola.getLastError()
            print(log)
            raise ValueError("Dymola simulation failed")
            
        return

    def writeResultsExcel(self, case_name, case_idx):

        with pd.ExcelWriter(self.excel_file, mode='a') as writer:
            self.df.to_excel(writer, sheet_name=case_name)

        return

class PressuredCabin: 
    """Model of PressuredCabin for testing of a single cabin segment"""
    def __init__(self, ext_sw, case_dir="DynTherM.Systems.Helicopter.Tests.PressuredCabin",
                 dymola_exe_dir=dymola_exe_directory,
                 package_dir=dymola_package_directory,
                 excel_file='HeliTestCases.xlsx'):
        # Instantiate the Dymola interface and start Dymola
        self.dymola = DymolaInterface(dymola_exe_dir)
        self.dymola.openModel(path=package_dir)
        self.ext_sw = ext_sw
        self.case_dir = case_dir
        # Instantiate the pandas dataframe to write results
        #self.df = pd.read_excel(excel_file, sheet_name='Template')
        
        #self.df.set_index("TICKERS", inplace=True)
        self.excel_file = excel_file
        # Initialize result vectors
        self.Q = np.array([])
        # Model characteristics
        self.p = ["TR","BR","BL","TL"]

    def run(self):
        input_E = ["cabinSegment.radiation_%s.E" % str(self.p[i]) for i in range(len(self.p))]
        input_theta = ["cabinSegment.radiation_%s.theta" % str(self.p[i]) for i in range(len(self.p))]
        input_keys = input_E + input_theta
        output_keys = ["cabinSegment.CabinTemperature"]
        output_1 = ["cabinSegment.radiation_%s.E" % str(self.p[i]) for i in range(len(self.p))]
        output_2 = ["cabinSegment.radiation_%s.theta" % str(self.p[i]) for i in range(len(self.p))]
        output_keys += output_1 + output_2
        input_values = []
        for ii in range(len(self.p)):
            input_values.append(self.ext_sw.E[ii])
        for ii in range(len(self.p)):
            input_values.append(self.ext_sw.theta[ii])

        result = self.dymola.simulateExtendedModel(self.case_dir, 0.0, 10.0, 0, 0.0, "Dassl", 0.0001, 0.0, "attempt", input_keys, input_values, output_keys, True)
        if result[0]:
            self.T = result[1][0]
            self.E = result[1][1:5]
            self.theta = result[1][5:9]
        else:
            log = self.dymola.getLastError()
            print(log)
            raise ValueError("Dymola simulation failed")
            
        return

    def writeResultsExcel(self, case_name):
        self.df = pd.DataFrame()
        self.df['SIDE'] = self.p
        self.df.set_index("SIDE", inplace=True)
        #self.df['TEMP'] = [self.T]
        self.df['E'] = self.E
        self.df['theta'] = self.theta
        with pd.ExcelWriter(self.excel_file, mode='a') as writer:
            self.df.to_excel(writer, sheet_name=case_name)

        return

class DynTherM:
    def __init__(self, ext_sw, n_secs, dymola_exe_dir="C:\\Program Files\\Dymola 2018 FD01\\bin64\\Dymola.exe",
                 package_dir="C:\\Users\\kikow\\OneDrive\\02 - TUDelft - AE\\Thesis\\ModelicaCode\\DynTherM",
                 excel_file='HeliTestCases.xlsx'):
        # Instantiate the Dymola interface and start Dymola
        self.dymola = DymolaInterface(dymola_exe_dir)
        self.dymola.openModel(path=package_dir)
        self.ext_sw = ext_sw
        # Instantiate the pandas dataframe to write results
        self.df = pd.read_excel(excel_file, sheet_name='Template')
        self.excel_file = excel_file
        # Initialize result vectors
        self.n_secs = n_secs
        self.T = np.array([])
        self.P = np.array([])
        self.m = np.array([])
        self.Q_rad_absorbed_cab = np.zeros(n_secs)
        self.Q_rad_emitted_cab = np.zeros(n_secs)
        self.Q_rad_transmitted_cab = np.zeros(2)
        self.Q_convection_ext_cab = np.zeros(n_secs)
        self.Q_convection_wallPl_cab = np.zeros(n_secs)
        self.Q_convection_intPl_cab = 0.0
        self.Q_cond_int_cab = np.zeros(n_secs)
        self.Q_cond_ext_cab = np.zeros(n_secs)
        self.Q_pax_sens_cab = np.zeros(3)
        self.Q_pax_lat_cab = np.zeros(3)
        self.Q_int_cab = 0.0
        self.T_fuselage = np.zeros((n_secs, 5))

    def run(self, case_dir, ECS_bc, rec_target):
        input_keys = ["m_ECS.k", "T_ECS.k", "Xw_ECS.k", "A320.rec_target"]
        E_input = ["A320.E_%s" % str(x) for x in range(1, self.n_secs + 1)]
        theta_input = ["A320.theta_%s" % str(x) for x in range(1, self.n_secs + 1)]
        input_keys += E_input + theta_input
        output_keys = ["A320.cabin.cabin.T", "A320.cabin.cabin.P", "A320.recirculationFan.inlet.P",
                       "A320.mixingManifold.inlet2.P", "A320.distributionPipe.inlet.P",
                       "A320.cabin.cabin.inlet.m_flow", "A320.recirculationFan.inlet.m_flow",
                       "A320.mixingManifold.inlet2.m_flow", "A320.distributionPipe.inlet.m_flow",
                       "A320.cabin.section_1.heatToSeats.Q_flow", "A320.cabin.section_5.heatToSeats.Q_flow",
                       "A320.cabin.cabin.Q_sens", "A320.cabin.cabin.Q_lat", "A320.cabin.cabin.Q_int",
                       "A320.cabin.convection_cabinInterior_plenum.Q_flow"]
        Q_rad_absorbed_cab = ["A320.cabin.section_%s.wallRadiation.Q_absorbed" % str(x) for x in range(1, self.n_secs + 1)]
        Q_rad_emitted_cab = ["A320.cabin.section_%s.wallRadiation.Q_emitted" % str(x) for x in range(1, self.n_secs + 1)]
        Q_conv_ext_cab = ["A320.cabin.section_%s.extConvection.inlet.Q_flow" % str(x) for x in range(1, self.n_secs + 1)]
        Q_conv_wallPl_cab = ["A320.cabin.section_%s.intConvection.inlet.Q_flow" % str(x) for x in range(1, self.n_secs + 1)]
        Q_cond_int_cab = ["A320.cabin.section_%s.composite.enclosedAirSpace.inlet_thermal.Q_flow" % str(x) for x in range(1, self.n_secs + 1)]
        Q_cond_ext_cab = ["A320.cabin.section_%s.composite.enclosedAirSpace.outlet_thermal.Q_flow" % str(x) for x in range(1, self.n_secs + 1)]
        T_composite_ext_cab = ["A320.cabin.section_%s.composite.ext.T" % str(x) for x in range(1, self.n_secs + 1)]
        T_skin_core_cab = ["A320.cabin.section_%s.composite.skin_core.T_vol" % str(x) for x in range(1, self.n_secs + 1)]
        T_insulation_cab = ["A320.cabin.section_%s.composite.insulationBlankets.T_vol" % str(x) for x in range(1, self.n_secs + 1)]
        T_intPanel_core_cab = ["A320.cabin.section_%s.composite.interiorPanel_core.T_vol" % str(x) for x in range(1, self.n_secs + 1)]
        T_int_cab = ["A320.cabin.section_%s.composite.int.T" % str(x) for x in range(1, self.n_secs + 1)]
        output_keys += Q_rad_absorbed_cab + Q_rad_emitted_cab + Q_conv_ext_cab + Q_conv_wallPl_cab + \
                       Q_cond_int_cab + Q_cond_ext_cab + \
                       T_int_cab + T_intPanel_core_cab + T_insulation_cab + T_skin_core_cab + T_composite_ext_cab
        input_values = ECS_bc
        input_values.append(rec_target)
        for ii in range(self.n_secs):
            input_values.append(self.ext_sw.E[ii])
        for ii in range(self.n_secs):
            input_values.append(self.ext_sw.theta[ii])

        result = \
            self.dymola.simulateExtendedModel(case_dir, 0.0, 1.0, 0, 0.0, "Dassl", 0.0001, 0.0, "prova",
                                              input_keys, input_values, output_keys, True)
        if result[0]:
            self.T = result[1][0]
            self.P = np.array([result[1][1], result[1][2], result[1][3], result[1][4]])
            self.m = np.array([result[1][5], result[1][6], result[1][7], result[1][8]])
            self.Q_rad_transmitted_cab = np.array([result[1][9], result[1][10]])
            self.Q_pax_sens_cab = np.asarray(result[1][11])
            self.Q_pax_lat_cab = np.asarray(result[1][12])
            self.Q_int_cab = result[1][13]
            self.Q_convection_intPl_cab = result[1][14]
            self.Q_rad_absorbed_cab = np.array([result[1][i] for i in range(15, self.n_secs + 15)])
            self.Q_rad_emitted_cab = np.array([result[1][i] for i in range(self.n_secs + 15, 2 * self.n_secs + 15)])
            self.Q_convection_ext_cab = np.array([result[1][i] for i in range(2 * self.n_secs + 15, 3 * self.n_secs + 15)])
            self.Q_convection_wallPl_cab = np.array([result[1][i] for i in range(3 * self.n_secs + 15, 4 * self.n_secs + 15)])
            self.Q_cond_int_cab = np.array([result[1][i] for i in range(4 * self.n_secs + 15, 5 * self.n_secs + 15)])
            self.Q_cond_ext_cab = - np.array([result[1][i] for i in range(5 * self.n_secs + 15, 6 * self.n_secs + 15)])
            self.T_fuselage[:, 0] = np.array([result[1][i] for i in range(6 * self.n_secs + 15, 7 * self.n_secs + 15)])
            self.T_fuselage[:, 1] = np.array([result[1][i] for i in range(7 * self.n_secs + 15, 8 * self.n_secs + 15)])
            self.T_fuselage[:, 2] = np.array([result[1][i] for i in range(8 * self.n_secs + 15, 9 * self.n_secs + 15)])
            self.T_fuselage[:, 3] = np.array([result[1][i] for i in range(9 * self.n_secs + 15, 10 * self.n_secs + 15)])
            self.T_fuselage[:, 4] = np.array([result[1][i] for i in range(10 * self.n_secs + 15, 11 * self.n_secs + 15)])
        else:
            raise ValueError("Dymola simulation failed")

        return

    def writeResultsExcel(self, case_name, case_idx):

        # self.df['Q_RAD_ABS_CAB'][case_idx] = np.sum(self.Q_rad_absorbed_cab)
        # self.df['Q_RAD_EM_CAB'][case_idx] = np.sum(self.Q_rad_emitted_cab)
        # self.df['Q_RAD_TRANS_CAB'][case_idx] = np.sum(self.Q_rad_transmitted_cab)
        # self.df['Q_CONV_EXT_CAB'][case_idx] = np.sum(self.Q_convection_ext_cab)
        # self.df['Q_CONV_WPL_CAB'][case_idx] = np.sum(self.)
        # self.df['Q_CONV_INTPL_CAB'][case_idx] = np.sum(self.)
        # self.df['Q_COND_EXT_CAB'][case_idx] = np.sum(self.)
        # self.df['Q_COND_INT_CAB'][case_idx] = np.sum(self.)
        # self.df['Q_RAD_ABS_FD'][case_idx] = np.sum(self.)
        # self.df['Q_RAD_EM_FD'][case_idx] = np.sum(self.)
        # self.df['Q_RAD_TRANS_FD'][case_idx] = np.sum(self.)
        # self.df['Q_CONV_EXT_FD'][case_idx] = np.sum(self.)
        # self.df['Q_CONV_WPL_FD'][case_idx] = np.sum(self.)
        # self.df['Q_CONV_INTPL_FD'][case_idx] = np.sum(self.)
        # self.df['Q_COND_EXT_FD'][case_idx] = np.sum(self.)
        # self.df['Q_COND_INT_FD'][case_idx] = np.sum(self.)
        # self.df['M_LEAK_CAB'][case_idx] = np.sum(self.)
        # self.df['M_LEAK_FD'][case_idx] = np.sum(self.)

        with pd.ExcelWriter(self.excel_file, mode='a') as writer:
            self.df.to_excel(writer, sheet_name=case_name)

        return

class HeliCombined: 
    """Model of HeliCombined for testing of a various"""
    def __init__(self, conditions, ext_sw, case_idx, case_dir="DynTherM.Systems.Helicopter.NH90.ComplexAirbusEES.HeliCombined",
                 dymola_exe_dir=dymola_exe_directory,
                 package_dir=dymola_package_directory,
                 excel_file='HeliTestCases.xlsx'):
        # Instantiate the Dymola interface and start Dymola
        self.dymola = DymolaInterface(dymola_exe_dir)
        self.dymola.openModel(path=package_dir)
        self.ext_sw = ext_sw
        self.case_dir = case_dir
        # Test Conditions
        self.conditions = conditions
        self.T_in = [x + 273.15 for x in self.conditions[0:2]]
        # Results Prep
        self.case_idx = case_idx
        self.excel_file = excel_file
        # Model characteristics
        self.p = ["TR","BR","BL","TL"] # input suffixes
        self.out = ["T_plenum", "Q_sens", "Q_lat", "Q", "T_evap", "H_rel_plenum"] # output suffixes

    def run(self):
        input_E_ck = ["heliCockpit.radiation_%s.E" % str(self.p[i]) for i in range(len(self.p))]
        input_theta_ck = ["heliCockpit.radiation_%s.theta" % str(self.p[i]) for i in range(len(self.p))]
        input_E_cb = ["heliCabin.radiation_%s.E" % str(self.p[i]) for i in range(len(self.p))]
        input_theta_cb = ["heliCabin.radiation_%s.theta" % str(self.p[i]) for i in range(len(self.p))]
        #input_temp = ["evaporatorCockpit.T_tgt", "evaporatorCabin.T_tgt"]
        input_temp = ["T_ck", "T_cb"]
        input_env = ["environment.Altitude", "environment.ISA_plus", "N_pax"]
        input_keys = input_E_ck + input_theta_ck + input_E_cb + input_theta_cb + input_temp + input_env
        
        input_values = []
        for ii in range(len(self.p)):
            input_values.append(self.ext_sw.E[ii])
        for ii in range(len(self.p)):
            input_values.append(self.ext_sw.theta[ii])
        input_values += input_values + self.T_in + self.conditions[2::]

        output_ck = ["evaporatorCockpit.%s" % str(self.out[i]) for i in range(len(self.out))]
        output_cb = ["evaporatorCabin.%s" % str(self.out[i]) for i in range(len(self.out))]
        output_keys = output_ck + output_cb
        result = self.dymola.simulateExtendedModel(self.case_dir, 0.0, 150.0, 0, 0.0, "Dassl", 0.0001, 0.0, "attempt", input_keys, input_values, output_keys, True)
        if result[0]:
            self.ck = result[1][0:6]
            self.cb = result[1][6::] # TODO: make this division based on length/2
        else:
            log = self.dymola.getLastError()
            print(log)
            raise ValueError("Dymola simulation failed")
            
        return

    def writeResultsDict(self):

        self.results = {'Case Number': self.case_idx, 'Alt': self.conditions[2], 'ISA': self.conditions[3], 'Npax': self.conditions[4],                      
                        'Ck T_tgt': self.conditions[0], 'Ck T': self.ck[0] - 273.15, 'Ck Q_sens':self.ck[1], 'Ck Q_lat':self.ck[2], 'Ck Q':self.ck[3], 
                        'Ck T_evap': self.ck[4], 'Ck H_rel': self.ck[5], 
                        'Cb T_tgt': self.conditions[1], 'Cb T': self.cb[0] - 273.15, 'Cb Q_sens':self.cb[1], 'Cb Q_lat':self.cb[2], 'Cb Q':self.cb[3],
                        'Cb T_evap': self.cb[4], 'Cb H_rel': self.cb[5] 
                        }
        
        return self.results

    def writeResultsExcel(self, model_name):
        self.df.loc[self.case_idx] = self.conditions[2::] + self.conditions[0:1] + [self.ck[0] -273.15] + self.ck[1::] + self.conditions[1:2] + [self.cb[0] -273.15] + self.cb[1::]
        if self.case_idx == 0:
            with pd.ExcelWriter(self.excel_file, mode='a') as writer:
                self.df.to_excel(writer, sheet_name=model_name)
        else:
            with pd.ExcelWriter(self.excel_file, mode='w') as writer:
                self.df.to_excel(writer, sheet_name=model_name)
        print(self.df)

        return