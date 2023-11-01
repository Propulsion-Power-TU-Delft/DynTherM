within DynTherM.Components.Adaptors;
model irradianceMultiplier
  "Model used to convert from irradiance port to distributed irradiance port"
  parameter Integer Nx(min=1) "Number of ports in x-direction";
  parameter Integer Ny(min=1) "Number of ports in y-direction";

  CustomInterfaces.IrradiancePort single annotation (Placement(transformation(extent={{-14,-74},{14,-46}}),
        iconTransformation(extent={{-14,-74},{14,-46}})));
  CustomInterfaces.DistributedIrradiancePort distributed(Nx=Nx, Ny=Ny) annotation (Placement(transformation(extent={{-40,-10},{40,130}}),
        iconTransformation(extent={{-40,-10},{40,130}})));

equation
  for i in 1:Nx loop
    for j in 1:Ny loop
      distributed.ports[i,j].E_tb  = single.E_tb;
      distributed.ports[i,j].E_td  = single.E_td;
      distributed.ports[i,j].E_tr  = single.E_tr;
      distributed.ports[i,j].theta = single.theta;
    end for;
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{0,-34},{0,34}},     color={238,46,47}),
        Line(points={{0,34},{6,24}},      color={238,46,47}),
        Line(points={{0,34},{-6,24}},     color={238,46,47})}),  Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p>
</html>"));
end irradianceMultiplier;
