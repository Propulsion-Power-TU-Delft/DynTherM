within DynTherM.Components.Adaptors;
model heatFluxToHeatFlow
  "Model used to convert from heat flux port to heat port"

  parameter Modelica.Units.SI.Area A "Heat transfer surface";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet
    annotation (Placement(transformation(extent={{-14,46},{14,74}}),
        iconTransformation(extent={{-14,46},{14,74}})));
  CustomInterfaces.HeatFluxPort_A outlet
    annotation (Placement(transformation(extent={{-14,-74},{14,-46}}),
        iconTransformation(extent={{-14,-74},{14,-46}})));
equation
  inlet.Q_flow + A*outlet.phi = 0;
  inlet.T = outlet.T;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{0,-34},{0,34}},     color={238,46,47}),
        Line(points={{0,34},{6,24}},      color={238,46,47}),
        Line(points={{0,34},{-6,24}},     color={238,46,47})}),  Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p><img src=\"modelica://ThermalManagement/ThermalManagement/Figures/ThermalRadiationASHRAE.PNG\"/></p>
</html>"));
end heatFluxToHeatFlow;
