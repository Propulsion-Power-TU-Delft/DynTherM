within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model AircraftFlying
  "Convection during flight (aircraft)"
  extends BaseClassExternal;

  parameter Real Cp[Nx,Ny]=zeros(Nx,Ny) "Pressure coefficient: approximately zero for passenger section around the fuselage";
  parameter Real coeff=1 "Fraction of cylinder with active heat transfer";
  parameter Modelica.Units.SI.Length L "Fuselage element length";
  parameter Modelica.Units.SI.Length R_ext "Fuselage radius";
  parameter Modelica.Units.SI.Length L_nose[Nx,Ny]
    "Distance between fuselage section and fuselage nose";

  Modelica.Units.SI.Pressure P_f[Nx,Ny] "Pressure surrounding the fuselage";
  Modelica.Units.SI.Temperature T_aw[Nx,Ny] "Recovery temperature";
  Real r[Nx,Ny] "Recovery factor for turbulent boundary layer";
  Modelica.Units.SI.ReynoldsNumber Re_star[Nx,Ny](start=1e8*ones(Nx,Ny))
    "Reynolds number used in the heat transfer correlation";
  Modelica.Units.SI.PrandtlNumber Pr_star[Nx,Ny]
    "Prandtl number used in the heat transfer correlation";

protected
  Medium.ThermodynamicState state_aw[Nx,Ny];
  Medium.ThermodynamicState state_star[Nx,Ny];
  Real gamma_amb;
  Modelica.Units.SI.PrandtlNumber Pr_aw[Nx,Ny](start=0.75*ones(Nx,Ny));
  Modelica.Units.SI.Temperature T_star[Nx,Ny];

equation
  gamma_amb = Medium.isentropicExponent(environment.state_amb);

  for i in 1:Nx loop
    for j in 1:Ny loop
      state_aw[i,j] = Medium.setState_pTX(P_f[i,j], T_aw[i,j], environment.X_amb);
      state_star[i,j] = Medium.setState_pTX(P_f[i,j], T_star[i,j], environment.X_amb);
      P_f[i,j] = environment.P_amb + Cp[i,j]*
        Medium.density(environment.state_amb)*environment.V_inf^2/2;
      T_aw[i,j] = environment.T_amb*(1 + r[i,j]*(gamma_amb - 1)/2*environment.Mach_inf^2);
      T_star[i,j] = (T_aw[i,j] + environment.T_amb)/2 + 0.22*(T_aw[i,j] - environment.T_amb);
      Pr_aw[i,j] = Medium.specificHeatCapacityCp(state_aw[i,j])*
        Medium.dynamicViscosity(state_aw[i,j])/
        Medium.thermalConductivity(state_aw[i,j]);
      r[i,j] = Pr_aw[i,j]^(1/3);
      Re_star[i,j] = Medium.density(state_star[i,j])*environment.V_inf*L_nose[i,j]/
        Medium.dynamicViscosity(state_star[i,j]);
      Pr_star[i,j] = Medium.specificHeatCapacityCp(state_star[i,j])*
        Medium.dynamicViscosity(state_star[i,j])/
        Medium.thermalConductivity(state_star[i,j]);
      ht[i,j] = Medium.density(state_star[i,j])*
        Medium.specificHeatCapacityCp(state_star[i,j])*
        environment.V_inf*0.185*
        ((Modelica.Math.log10(Re_star[i,j]))^(-2.584))*(Pr_star[i,j]^(-2/3));
      T_out[i,j] = T_aw[i,j];
    end for;
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Reference:</p>
<p>[1] ASHRAE&nbsp;Handbook&nbsp;-&nbsp;HVAC&nbsp;Applications,&nbsp;chapter&nbsp;13</p>
</html>"));
end AircraftFlying;
