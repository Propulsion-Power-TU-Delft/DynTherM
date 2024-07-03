within DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection;
model Gnielinski
  "Gnielinski's correlation for internal convection in pipes"

  extends BaseClassInternal;
  input Length Dh "Hydraulic diameter" annotation(Dialog(enable = true));
  input Temperature T_in "Inlet fluid temperature" annotation(Dialog(enable = true));
  input Temperature T_out "Outlet fluid temperature" annotation(Dialog(enable = true));
  input ReynoldsNumber Re "Reynolds number" annotation(Dialog(enable = true));
  input PrandtlNumber Pr "Prandtl number" annotation(Dialog(enable = true));
  input Medium.ThermodynamicState state "Average thermodynamic state" annotation(Dialog(enable = true));
  input ReynoldsNumber f "Friction factor";

  NusseltNumber Nu "Nusselt number";

equation

  // Laminar Flow
  if Re < 2300 then
    Nu = 4.36       "Constant heat flux";
  // Turbulent Flow
  else
    Nu = ((f/8)*(Re - 1000)*Pr)/(1 + 12.7*((f/8)^0.5)*((Pr^0.6667) - 1));
  end if;

  Nu = ht*Dh/Medium.thermalConductivity(state);

  // Sanity check
  assert(Re >= 5e6, "The Gnielinski's correlation is only valid only for Reynolds numbers greater than 5e6", AssertionLevel.warning);
  assert(Re <= 3e3, "The Gnielinski's correlation is only valid only for Reynolds numbers greater than 3e3", AssertionLevel.warning);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Gnielinski;
