within DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection;
model Gnielinski
  "Gnielinski's Correlation for internal convection in pipes"

  extends BaseClassInternal;
  input Modelica.Units.SI.Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
  input Modelica.Units.SI.Temperature T_in "Inlet fluid temperature" annotation(Dialog(enable = true));
  input Modelica.Units.SI.Temperature T_out "Outlet fluid temperature" annotation(Dialog(enable = true));
  input Modelica.Units.SI.ReynoldsNumber Re "Reynolds number" annotation(Dialog(enable = true));
  input Modelica.Units.SI.PrandtlNumber Pr "Prandtl number" annotation(Dialog(enable = true));
  input Medium.ThermodynamicState state "Average thermodynamic state" annotation(Dialog(enable = true));
  input Modelica.Units.SI.ReynoldsNumber f "Friction Factor";


  Modelica.Units.SI.NusseltNumber Nu "Nusselt number";

equation

// Laminar Flow
  if Re<2300 then
    Nu = 4.36       "Constant heat flux";
// Turbulent Flow
  else
    Nu = ((f/8)*(Re-1000)*Pr)/(1+12.7*((f/8)^0.5)*((Pr^0.6667)-1));
  end if;

  // Sanity check
  assert(Re >= 5e6, "The Gnielinski's correlation is only valid only for Reynolds numbers greater than 5e6", AssertionLevel.warning);
  assert(Re <= 3e3, "The Gnielinski's correlation is only valid only for Reynolds numbers greater than 3e3", AssertionLevel.warning);

  Nu = ht*Dh/Medium.thermalConductivity(state);


  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Gnielinski;
