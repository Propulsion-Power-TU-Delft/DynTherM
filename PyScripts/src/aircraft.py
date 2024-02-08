# -*- coding: utf-8 -*-

########################################################################################################################
# DynTherM: Dynamic modeling and simulation of Thermal Management systems
# Author: Dr. ir. A. Giuffre'
# Content: run aircraft thermal model in Dymola
# 2024 - Delft University of Technology - All rights reserved
########################################################################################################################

import numpy as np
from dymola.dymola_interface import DymolaInterface
from dymola.dymola_exception import DymolaException


class Aircraft:
    def __init__(self, package_dir, n_cab_secs=1, n_fus_secs=8):
        """
        :param package_dir: absolute path to the DynTherM package.mo file
        :param n_cab_secs: number of ventilation zones in which the cabin is divided
        :param n_fus_secs: number of circumferential sections in which the fuselage is divided
        """
        self.package_dir = package_dir
        self.n_cab_secs = n_cab_secs
        self.n_fus_secs = n_fus_secs

        # Initialize result vectors
        self.T = np.array([])
        self.P = np.array([])
        self.phi = np.array([])
        self.mass = np.array([])
        self.Q = np.array([])
        self.T_fuselage_fd = np.zeros((self.n_fus_secs, 4))
        self.T_fuselage_cab = np.zeros((self.n_fus_secs, 4))

    def run(self, data, radiation, case_idx):
        """
        Run Dymola and get the results for post-processing
        :param data: object collecting the input values read from the configuration file
        :param radiation: object collecting the solar radiation information
        :param case_idx: index identifying the case study under analysis
        """
        # define input keys
        E_input = ["A320.E_tb_%s" % str(x) for x in range(1, self.n_fus_secs + 1)] + \
                  ["A320.E_td_%s" % str(x) for x in range(1, self.n_fus_secs + 1)] + \
                  ["A320.E_tr_%s" % str(x) for x in range(1, self.n_fus_secs + 1)]
        theta_input = ["A320.theta_%s" % str(x) for x in range(1, self.n_fus_secs + 1)]
        input_keys = ["m_ECS.k", "A320.rec_target", "T_target.k", "m_trim_cab.k", "m_trim_fd.k", "T_trim.k",
                      "environment.Mach_inf_di", "environment.altitude_di", "environment.ISA_plus",
                      "environment.phi_amb", "environment.phi_amb_ground", "environment.T_ground", "A320.N_pax",
                      "A320.N_crew", "A320.N_pilots", "A320.Q_el", "A320.Q_galley", "A320.Q_avionics",
                      "A320.cabinLights", "A320.inFlightEntertainment"] + E_input + theta_input + \
                     ["A320.E_tb_front", "A320.E_td_front", "A320.E_tr_front", "A320.theta_front"]

        # define input values
        input_values = [data['M_pack'][case_idx], data['Rec_ratio'][case_idx], data['T_target'][case_idx],
                        data['M_trim_cab'][case_idx], data['M_trim_fd'][case_idx], data['T_trim'][case_idx],
                        data['Mach_inf'][case_idx], data['Height'][case_idx], data['Delta_ISA'][case_idx],
                        data['Phi_amb'][case_idx], data['Phi_ground'][case_idx], data['T_ground'][case_idx],
                        data['N_pax'][case_idx], data['N_crew'][case_idx], data['N_pilots'][case_idx],
                        data['Q_electronics'][case_idx], data['Q_galley'][case_idx], data['Q_avionics'][case_idx],
                        data['Cabin_lights'][case_idx], data['IFE'][case_idx]] + \
                       [radiation.E_direct[i] for i in range(self.n_fus_secs)] + \
                       [radiation.E_diffuse[i] for i in range(self.n_fus_secs)] + \
                       [radiation.E_reflected[i] for i in range(self.n_fus_secs)] + \
                       [radiation.theta[i] for i in range(self.n_fus_secs)] + \
                       [radiation.E_direct[-1], radiation.E_diffuse[-1], radiation.E_reflected[-1], radiation.theta[-1]]

        # define output keys
        T_output = ["A320.T_ECS", "A320.cockpit.flightDeck.T", "A320.cabin.cabin.T",
                    "A320.cargo.cargo.T", "A320.EEbay.cargo.T", "A320.mixingManifold.T"]
        P_output = ["A320.packFlow.outlet.P", "A320.cockpit.flightDeck.P", "A320.cabin.cabin.P", "A320.cargo.cargo.P",
                    "A320.EEbay.cargo.P", "A320.recirculationFan.inlet.P", "A320.mixingManifold.P"]
        phi_output = ["A320.phi_cockpit", "A320.phi_cabin", "A320.phi_cargo", "A320.phi_EE_bay", "A320.phi_mix"]
        mass_output = ["A320.mixingManifold.outlet.m_flow", "A320.cockpit.cockpitInflow.m_flow",
                       "A320.cabin.cabinInflow.m_flow", "A320.cargo.cargoToCabin.m_flow",
                       "A320.EEbay.cargoToCabin.m_flow", "A320.recirculationFan.inlet.m_flow",
                       "A320.outflowValve.inlet.m_flow", "A320.flowSplit", "A320.cockpitTrimFlow.outlet.m_flow",
                       "A320.cabinTrimFlow.outlet.m_flow"]
        Q_output = ["A320.Q_tot", "A320.Q_tot_cab", "A320.Q_int_cab", "A320.Q_ext_cab",
                    "A320.Q_tot_fd", "A320.Q_int_fd", "A320.Q_ext_fd", "A320.Q_avionics"]
        T_fuselage_ext = ["A320.cockpit.section_%s.composite.ext.T" % str(x) for x in range(1, 6)] + \
                         ["A320.EEbay.section_%s.composite.ext.T" % str(x) for x in range(6, 9)] + \
                         ["A320.cabin.section_%s.composite.ext.T" % str(x) for x in range(1, 6)] + \
                         ["A320.cargo.section_%s.composite.ext.T" % str(x) for x in range(6, 9)]
        T_skin_core = ["A320.cockpit.section_%s.composite.skin_core.T_vol" % str(x) for x in range(1, 6)] + \
                      ["A320.EEbay.section_%s.composite.skin_core.T_vol" % str(x) for x in range(6, 9)] + \
                      ["A320.cabin.section_%s.composite.skin_core.T_vol" % str(x) for x in range(1, 6)] + \
                      ["A320.cargo.section_%s.composite.skin_core.T_vol" % str(x) for x in range(6, 9)]
        T_panel_core = ["A320.cockpit.section_%s.composite.interiorPanel_core.T_vol" % str(x) for x in range(1, 6)] + \
                       ["A320.EEbay.section_%s.composite.interiorPanel_core.T_vol" % str(x) for x in range(6, 9)] + \
                       ["A320.cabin.section_%s.composite.interiorPanel_core.T_vol" % str(x) for x in range(1, 6)] + \
                       ["A320.cargo.section_%s.composite.interiorPanel_core.T_vol" % str(x) for x in range(6, 9)]
        T_fuselage_int = ["A320.cockpit.section_%s.composite.int.T" % str(x) for x in range(1, 6)] + \
                         ["A320.EEbay.section_%s.composite.int.T" % str(x) for x in range(6, 9)] + \
                         ["A320.cabin.section_%s.composite.int.T" % str(x) for x in range(1, 6)] + \
                         ["A320.cargo.section_%s.composite.int.T" % str(x) for x in range(6, 9)]

        output_keys = T_fuselage_int + T_panel_core + T_skin_core + T_fuselage_ext + \
            Q_output + T_output + P_output + phi_output + mass_output

        dymola = None

        try:
            model_name = f"DynTherM.Examples.Aircraft.{data['Model_dir'][case_idx]}.{data['Case_name'][case_idx]}"
            print(f"\nRun name: {data['Run_name'][case_idx]}")
            print(f"Simulating model: {model_name}\n")

            # Instantiate the Dymola interface and start Dymola
            dymola = DymolaInterface()
            dymola.openModel(path=self.package_dir)

            # translate the prescribed model to avoid issues with non-evaluated parameters
            dymola.translateModel(model_name + "(environment.ISA_plus=%d)" % data['Delta_ISA'][case_idx])

            # run the prescribed model in Dymola
            result = dymola.simulateExtendedModel("", 0.0, 10.0, 0, 0.0, "Dassl", 0.0001, 0.0,
                                                  'Simulations/' + data['Run_name'][case_idx],
                                                  input_keys, input_values, output_keys, True)
            # get output values
            if result[0]:
                idx = 0

                # fuselage temperature
                for j in range(4):
                    self.T_fuselage_fd[0:8, j] = np.array([result[1][idx + i] for i in range(8)])
                    idx += 8
                    self.T_fuselage_cab[0:8, j] = np.array([result[1][idx + i] for i in range(8)])
                    idx += 8

                # heat flow rates
                for i in range(len(Q_output)):
                    self.Q = np.append(self.Q, result[1][idx])
                    idx += 1

                # temperature along the air distribution system
                for i in range(len(T_output)):
                    self.T = np.append(self.T, result[1][idx])
                    idx += 1

                # pressure along the air distribution system
                for i in range(len(P_output)):
                    self.P = np.append(self.P, result[1][idx])
                    idx += 1

                # relative humidity along the air distribution system
                for i in range(len(phi_output)):
                    self.phi = np.append(self.phi, result[1][idx])
                    idx += 1

                # mass flow rate along the air distribution system
                for i in range(len(mass_output)):
                    self.mass = np.append(self.mass, result[1][idx])
                    idx += 1

            else:
                print("\nDymola simulation did not converge")
                log = dymola.getLastErrorLog()
                print(log)

        # close Dymola
        except DymolaException as ex:
            print("Error: " + str(ex))

        finally:
            if dymola is not None:
                dymola.close()
