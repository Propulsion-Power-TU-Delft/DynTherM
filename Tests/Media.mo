within DynTherM.Tests;
package Media

  model WaterGlycolMixtures
    package EGW_20 = DynTherM.Media.IncompressibleTableBased.EGW(X = 0.2);
    package EGW_40 = DynTherM.Media.IncompressibleTableBased.EGW(X = 0.4);
    package EGW_60 = DynTherM.Media.IncompressibleTableBased.EGW(X = 0.6);
    package PGW_20 = DynTherM.Media.IncompressibleTableBased.PGW(X = 0.2);
    package PGW_40 = DynTherM.Media.IncompressibleTableBased.PGW(X = 0.4);
    package PGW_60 = DynTherM.Media.IncompressibleTableBased.PGW(X = 0.6);

    parameter Pressure P = 1e5;
    parameter Temperature T_vec[9] = {260, 265, 270, 275, 280, 285, 290, 295, 300};

    EGW_20.ThermodynamicState state_EGW_20[9];
    EGW_40.ThermodynamicState state_EGW_40[9];
    EGW_60.ThermodynamicState state_EGW_60[9];
    PGW_20.ThermodynamicState state_PGW_20[9];
    PGW_40.ThermodynamicState state_PGW_40[9];
    PGW_60.ThermodynamicState state_PGW_60[9];

    Density rho_EGW_20[9];
    Density rho_EGW_40[9];
    Density rho_EGW_60[9];
    Density rho_PGW_20[9];
    Density rho_PGW_40[9];
    Density rho_PGW_60[9];

    SpecificHeatCapacity cp_EGW_20[9];
    SpecificHeatCapacity cp_EGW_40[9];
    SpecificHeatCapacity cp_EGW_60[9];
    SpecificHeatCapacity cp_PGW_20[9];
    SpecificHeatCapacity cp_PGW_40[9];
    SpecificHeatCapacity cp_PGW_60[9];

    ThermalConductivity kt_EGW_20[9];
    ThermalConductivity kt_EGW_40[9];
    ThermalConductivity kt_EGW_60[9];
    ThermalConductivity kt_PGW_20[9];
    ThermalConductivity kt_PGW_40[9];
    ThermalConductivity kt_PGW_60[9];

    DynamicViscosity mu_EGW_20[9];
    DynamicViscosity mu_EGW_40[9];
    DynamicViscosity mu_EGW_60[9];
    DynamicViscosity mu_PGW_20[9];
    DynamicViscosity mu_PGW_40[9];
    DynamicViscosity mu_PGW_60[9];

  equation
    for i in 1:9 loop
      state_EGW_20[i] = EGW_20.setState_pT(P, T_vec[i]);
      state_EGW_40[i] = EGW_40.setState_pT(P, T_vec[i]);
      state_EGW_60[i] = EGW_60.setState_pT(P, T_vec[i]);
      state_PGW_20[i] = PGW_20.setState_pT(P, T_vec[i]);
      state_PGW_40[i] = PGW_40.setState_pT(P, T_vec[i]);
      state_PGW_60[i] = PGW_60.setState_pT(P, T_vec[i]);

      rho_EGW_20[i] = EGW_20.density(state_EGW_20[i]);
      rho_EGW_40[i] = EGW_40.density(state_EGW_40[i]);
      rho_EGW_60[i] = EGW_60.density(state_EGW_60[i]);
      rho_PGW_20[i] = PGW_20.density(state_PGW_20[i]);
      rho_PGW_40[i] = PGW_40.density(state_PGW_40[i]);
      rho_PGW_60[i] = PGW_60.density(state_PGW_60[i]);

      cp_EGW_20[i] = EGW_20.specificHeatCapacityCp(state_EGW_20[i]);
      cp_EGW_40[i] = EGW_40.specificHeatCapacityCp(state_EGW_40[i]);
      cp_EGW_60[i] = EGW_60.specificHeatCapacityCp(state_EGW_60[i]);
      cp_PGW_20[i] = PGW_20.specificHeatCapacityCp(state_PGW_20[i]);
      cp_PGW_40[i] = PGW_40.specificHeatCapacityCp(state_PGW_40[i]);
      cp_PGW_60[i] = PGW_60.specificHeatCapacityCp(state_PGW_60[i]);

      kt_EGW_20[i] = EGW_20.thermalConductivity(state_EGW_20[i]);
      kt_EGW_40[i] = EGW_40.thermalConductivity(state_EGW_40[i]);
      kt_EGW_60[i] = EGW_60.thermalConductivity(state_EGW_60[i]);
      kt_PGW_20[i] = PGW_20.thermalConductivity(state_PGW_20[i]);
      kt_PGW_40[i] = PGW_40.thermalConductivity(state_PGW_40[i]);
      kt_PGW_60[i] = PGW_60.thermalConductivity(state_PGW_60[i]);

      mu_EGW_20[i] = EGW_20.dynamicViscosity(state_EGW_20[i]);
      mu_EGW_40[i] = EGW_40.dynamicViscosity(state_EGW_40[i]);
      mu_EGW_60[i] = EGW_60.dynamicViscosity(state_EGW_60[i]);
      mu_PGW_20[i] = PGW_20.dynamicViscosity(state_PGW_20[i]);
      mu_PGW_40[i] = PGW_40.dynamicViscosity(state_PGW_40[i]);
      mu_PGW_60[i] = PGW_60.dynamicViscosity(state_PGW_60[i]);
    end for;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end WaterGlycolMixtures;
end Media;
