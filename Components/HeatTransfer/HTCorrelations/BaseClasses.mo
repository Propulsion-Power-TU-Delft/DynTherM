within DynTherM.Components.HeatTransfer.HTCorrelations;
package BaseClasses
  partial model BaseClassExternal
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
    outer DynTherM.Components.Environment environment "Environmental properties";

    input Temperature T_surf "Surface temperature" annotation (Dialog(enable=true));
    parameter CoefficientOfHeatTransfer ht_start=10
      "Heat transfer coefficient - starting value";

    CoefficientOfHeatTransfer ht(start=ht_start) "Heat transfer coefficient";
    Temperature T_out "Temperature used to compute heat flow rate";

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end BaseClassExternal;

  partial model BaseClassInternal
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

    parameter CoefficientOfHeatTransfer ht_start=10
      "Heat transfer coefficient - starting value";

    CoefficientOfHeatTransfer ht(start=ht_start)
      "Heat transfer coefficient";

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end BaseClassInternal;

  partial model BaseClassFreeConvectionInternal "Free convection between in a space confined by two surfaces"
    extends BaseClasses.BaseClassInternal;

    input Length x "Characteristic dimension" annotation (Dialog(enable=true));
    input Temperature T1 "Temperature of surface 1" annotation (Dialog(enable=true));
    input Temperature T2 "Temperature of surface 2" annotation (Dialog(enable=true));
    input Pressure P_film "Pressure of the fluid in the enclosure" annotation (Dialog(enable=true));
    input MassFraction X_film[Medium.nX]=Medium.reference_X "Mass fractions of the fluid in the enclosure" annotation (Dialog(enable=true));

    PrandtlNumber Pr "Prandtl number";
    GrashofNumberOfMassTransfer Gr "Grashof number";
    RayleighNumber Ra "Rayleigh number";
    NusseltNumber Nu "NusseltNumber";
    Real beta "Thermal expansion coefficient";
    Temperature T_film "Film temperature";
    Medium.ThermodynamicState state_film "Thermodynamic state of the film";

  equation
    state_film = Medium.setState_pTX(P_film, T_film, X_film);
    T_film = (T1 + T2)/2;
    beta = 1/T_film;
    Pr = Medium.specificHeatCapacityCp(state_film)*Medium.dynamicViscosity(state_film)/
      Medium.thermalConductivity(state_film);
    Gr = g_n*beta*abs(T1 - T2)*(x^3)/
      ((Medium.dynamicViscosity(state_film)/Medium.density(state_film))^2);
    Ra = Gr*Pr;
    Nu = max(1, ht*x/Medium.thermalConductivity(state_film));

  end BaseClassFreeConvectionInternal;

  partial model BaseClassFreeConvectionExternal "Free convection with external environment"
    extends BaseClasses.BaseClassExternal;

    input Length x "Characteristic dimension" annotation (Dialog(enable=true));
    input Angle theta "Inclination angle with respect to vertical direction" annotation (Dialog(enable=true));

    PrandtlNumber Pr "Prandtl number";
    GrashofNumberOfMassTransfer Gr "Grashof number";
    RayleighNumber Ra "Rayleigh number";
    NusseltNumber Nu "Nusselt number";
    Real beta "Thermal expansion coefficient";
    Temperature T_film "Film temperature";
    Medium.ThermodynamicState state_film "Thermodynamic state of the film";

  equation
    state_film = Medium.setState_pTX(environment.P_amb, T_film, environment.X_amb);
    T_film = (T_surf + environment.T_amb)/2;
    beta = 1/T_film;
    Pr = Medium.specificHeatCapacityCp(state_film)*Medium.dynamicViscosity(state_film)/
      Medium.thermalConductivity(state_film);
    Gr = g_n*cos(theta)*beta*abs(T_surf - environment.T_amb)*(x^3)/
      ((Medium.dynamicViscosity(state_film)/Medium.density(state_film))^2);
    Ra = Gr*Pr;
    Nu = max(1, ht*x/Medium.thermalConductivity(state_film));

    assert(theta <= Modelica.Units.Conversions.to_deg(60), "Inclination angle must be lower than 60 degrees");

  end BaseClassFreeConvectionExternal;

  partial model BaseClassForcedConvectionExternal
    "Forced convection with external environment"
    extends BaseClasses.BaseClassExternal;

    input Length x "Characteristic dimension" annotation (Dialog(enable=true));
    input Velocity V "Flow velocity" annotation (Dialog(enable=true));
    parameter Velocity V_min=1 "Minimum velocity used for model convergence";

    ReynoldsNumber Re "Reynolds number";
    PrandtlNumber Pr "Prandtl number";
    NusseltNumber Nu "Nusselt number";
    Temperature T_film "Film temperature";
    Medium.ThermodynamicState state_film "Thermodynamic state of the film";

  equation
    state_film = Medium.setState_pTX(environment.P_amb, T_film, environment.X_amb);
    T_film = (T_surf + environment.T_amb)/2;
    Pr = Medium.specificHeatCapacityCp(state_film)*Medium.dynamicViscosity(state_film)/
      Medium.thermalConductivity(state_film);
    Re = Medium.density(state_film)*max(V_min,V)*x/Medium.dynamicViscosity(state_film);
    Nu = max(1, ht*x/Medium.thermalConductivity(state_film));

  end BaseClassForcedConvectionExternal;
end BaseClasses;
