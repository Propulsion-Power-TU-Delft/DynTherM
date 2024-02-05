within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model AircraftOnGroundForced
  "Forced convection along the fuselage of an aircraft on ground"
  extends BaseClassExternal;

  parameter Modelica.Units.SI.Length R_ext "Fuselage external radius";
  parameter Modelica.Units.SI.Velocity V_wind=6.7 "Wind speed on ground";

  Modelica.Units.SI.PrandtlNumber Pr[Nx,Ny] "Prandtl number";
  Modelica.Units.SI.ReynoldsNumber Re[Nx,Ny] "Reynolds number";

protected
  Medium.ThermodynamicState state_f[Nx,Ny];
  Modelica.Units.SI.Temperature Tf[Nx,Ny];

equation
  T_out = environment.T_amb*ones(Nx,Ny);

  for i in 1:Nx loop
    for j in 1:Ny loop
      state_f[i,j] = Medium.setState_pTX(environment.P_amb, Tf[i,j], environment.X_amb);
      Tf[i,j] = (T_skin[i,j] + environment.T_amb)/2;
      Pr[i,j] = Medium.specificHeatCapacityCp(state_f[i,j])*
        Medium.dynamicViscosity(state_f[i,j])/
        Medium.thermalConductivity(state_f[i,j]);
      Re[i,j] = Medium.density(state_f[i,j])*V_wind*(2*R_ext)/
        Medium.dynamicViscosity(state_f[i,j]);
      ht[i,j] = 0.0266*Medium.thermalConductivity(state_f[i,j])*
        Re[i,j]^0.805*Pr[i,j]^(1/3)/(2*R_ext);
    end for;
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Reference:</p>
<p>[1] ASHRAE&nbsp;Handbook&nbsp;-&nbsp;HVAC&nbsp;Applications,&nbsp;chapter&nbsp;13</p>
</html>"));
end AircraftOnGroundForced;
