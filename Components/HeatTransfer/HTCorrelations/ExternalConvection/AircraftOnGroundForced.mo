within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model AircraftOnGroundForced
  "Forced convection along the fuselage of an aircraft on ground"
  extends BaseClassExternal;

  parameter Modelica.Units.SI.Length R_ext "Fuselage external radius";
  parameter Modelica.Units.SI.Velocity V_wind=6.7 "Wind speed on ground";

  Modelica.Units.SI.PrandtlNumber Pr "Prandtl number";
  Modelica.Units.SI.ReynoldsNumber Re "Reynolds number";

protected
  Medium.ThermodynamicState state_f;
  Modelica.Units.SI.Temperature Tf;

equation
  state_f = Medium.setState_pTX(environment.P_amb, Tf, environment.X_amb);
  Tf = (T_skin + environment.T_amb)/2;
  Pr = Medium.specificHeatCapacityCp(state_f)*Medium.dynamicViscosity(state_f)/
    Medium.thermalConductivity(state_f);
  Re = Medium.density(state_f)*V_wind*(2*R_ext)/Medium.dynamicViscosity(state_f);
  ht = 0.0266*Medium.thermalConductivity(state_f)*(Re)^0.805*Pr^(1/3)/(2*R_ext);
  T_out = environment.T_amb;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Reference:</p>
<p>[1] ASHRAE&nbsp;Handbook&nbsp;-&nbsp;HVAC&nbsp;Applications,&nbsp;chapter&nbsp;13</p>
</html>"));
end AircraftOnGroundForced;
