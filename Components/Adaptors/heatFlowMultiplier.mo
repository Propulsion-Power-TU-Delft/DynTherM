within DynTherM.Components.Adaptors;
model heatFlowMultiplier "Model used to convert from heat port to distributed heat port"
  parameter Integer Nx(min=1) "Number of ports in x-direction";
  parameter Integer Ny(min=1) "Number of ports in y-direction";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a single annotation (Placement(transformation(extent={{-14,-74},{14,-46}}),
       iconTransformation(extent={{-14,-74},{14,-46}})));
  CustomInterfaces.DistributedHeatPort_A distributed(Nx=Nx, Ny=Ny) annotation (Placement(transformation(extent={{-40,-10},{40,130}}),
       iconTransformation(extent={{-40,-10},{40,130}})));

equation
  sum(distributed.ports.Q_flow) + single.Q_flow = 0;

  for i in 1:Nx loop
    for j in 1:Ny loop
      distributed.ports[i,j].T  = single.T;
    end for;
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{0,-34},{0,34}},     color={238,46,47}),
        Line(points={{0,34},{6,24}},      color={238,46,47}),
        Line(points={{0,34},{-6,24}},     color={238,46,47})}),  Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p></html>"),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end heatFlowMultiplier;
