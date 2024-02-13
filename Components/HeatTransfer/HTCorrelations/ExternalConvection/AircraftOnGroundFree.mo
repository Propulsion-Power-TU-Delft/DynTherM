within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model AircraftOnGroundFree
  "Free convection along the fuselage of an aircraft on ground"
  extends BaseClassExternal;

  parameter Modelica.Units.SI.Length R_ext "Fuselage radius";
  Modelica.Units.SI.PrandtlNumber Pr "Prandtl number";
  Modelica.Units.SI.GrashofNumberOfMassTransfer Gr "Grashof number";
  Real beta "Expansion coefficient";

protected
  Modelica.Units.SI.Temperature Tf;
  Medium.ThermodynamicState state_f;

equation
  state_f = Medium.setState_pTX(environment.P_amb, Tf, environment.X_amb);
  Tf = (T_skin + environment.T_amb)/2;
  beta = 1/Tf;
  Pr = Medium.specificHeatCapacityCp(state_f)*Medium.dynamicViscosity(state_f)/
    Medium.thermalConductivity(state_f);
  Gr = environment.g*beta*abs(T_skin - environment.T_amb)*((2*R_ext)^3)/
    ((Medium.dynamicViscosity(state_f)/Medium.density(state_f))^2);
  ht = (0.13*Medium.thermalConductivity(state_f)*(Gr*Pr)^(1/3))/(2*R_ext);
  T_out = environment.T_amb;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><b>Reference:</b></p>
<p>[1] ASHRAE&nbsp;Handbook&nbsp;-&nbsp;HVAC&nbsp;Applications,&nbsp;chapter&nbsp;13</p>
</html>"));
end AircraftOnGroundFree;
