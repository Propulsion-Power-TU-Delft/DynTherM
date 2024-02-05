within DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection;
model WingFlying
  "Forced convection along a wing section placed in the slipstream of a propeller (distributed propulsion) at cruise conditions"
  extends BaseClassExternal;

  input Length L_le[Nx,Ny] "Distance from leading edge" annotation(Dialog(enable=true));

  PrandtlNumber Pr[Nx,Ny] "Prandtl number";
  ReynoldsNumber Re[Nx,Ny] "Reynolds number";
  NusseltNumber Nu[Nx,Ny] "Nusselt number";

protected
  Medium.ThermodynamicState state_f[Nx,Ny];
  Temperature Tf[Nx,Ny] "Film temperature";

equation
  T_out = environment.T_amb*ones(Nx,Ny);

  for i in 1:Nx loop
    for j in 1:Ny loop
      state_f[i,j] = Medium.setState_pTX(environment.P_amb, Tf[i,j], environment.X_amb);
      Tf[i,j] = (T_skin[i,j] + environment.T_amb)/2;
      Pr[i,j] = Medium.specificHeatCapacityCp(state_f[i,j])*
        Medium.dynamicViscosity(state_f[i,j])/
        Medium.thermalConductivity(state_f[i,j]);
      Re[i,j] = Medium.density(state_f[i,j])*environment.V_inf*L_le[i,j]/
        Medium.dynamicViscosity(state_f[i,j]);
      Nu[i,j] = 0.385*Re[i,j]^0.755*Pr[i,j]^5.285*
        (environment.T_amb/T_skin[i,j])^0.044;
      Nu[i,j] = ht[i,j]*L_le[i,j]/Medium.thermalConductivity(state_f[i,j]);
    end for;
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Reference:</p>
<p>[1] A. L. Habermann, et al. &quot;Aerodynamic Effects of a Wing Surface Heat Exchanger&quot;, MDPI Aerospace, 2023.</p>
</html>"));
end WingFlying;
