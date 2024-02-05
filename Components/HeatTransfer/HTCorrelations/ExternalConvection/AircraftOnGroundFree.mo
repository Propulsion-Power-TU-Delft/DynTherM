within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model AircraftOnGroundFree
  "Free convection along the fuselage of an aircraft on ground"
  extends BaseClassExternal;
  parameter Modelica.Units.SI.Length R_ext "Fuselage radius";

  Modelica.Units.SI.PrandtlNumber Pr[Nx,Ny] "Prandtl number";
  Modelica.Units.SI.GrashofNumberOfMassTransfer Gr[Nx,Ny] "Grashof number";
  Real beta[Nx,Ny] "Expansion coefficient";

protected
  Modelica.Units.SI.Temperature Tf[Nx,Ny];
  Medium.ThermodynamicState state_f[Nx,Ny];

equation
  T_out = environment.T_amb*ones(Nx,Ny);

  for i in 1:Nx loop
    for j in 1:Ny loop
      state_f[i,j] = Medium.setState_pTX(environment.P_amb, Tf[i,j], environment.X_amb);
      Tf[i,j] = (T_skin[i,j] + environment.T_amb)/2;
      beta[i,j] = 1/Tf[i,j];
      Pr[i,j] = Medium.specificHeatCapacityCp(state_f[i,j])*
        Medium.dynamicViscosity(state_f[i,j])/
        Medium.thermalConductivity(state_f[i,j]);
      Gr[i,j] = environment.g*beta[i,j]*
        abs(T_skin[i,j] - environment.T_amb)*((2*R_ext)^3)/
        ((Medium.dynamicViscosity(state_f[i,j])/Medium.density(state_f[i,j]))^2);
      ht[i,j] = (0.13*Medium.thermalConductivity(state_f[i,j])*
        (Gr[i,j]*Pr[i,j])^(1/3))/(2*R_ext);
    end for;
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Reference:</p>
<p>[1] ASHRAE&nbsp;Handbook&nbsp;-&nbsp;HVAC&nbsp;Applications,&nbsp;chapter&nbsp;13</p>
</html>"));
end AircraftOnGroundFree;
