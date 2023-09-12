within DynTherM.Components.HeatTransfer;
model InternalConvection
  "0D model of internal convection with a cylinder"
  // Reference: SAE AIR 1168/3, section 3.2.2
  package Medium = Modelica.Media.Air.MoistAir;
  outer DynTherM.Components.Environment environment "Environmental properties";
  parameter Modelica.Units.SI.Area A "Heat transfer area";
  replaceable model HTC =
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    annotation (choicesAllMatching=true);
  HTC ht_correlation;
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet
    annotation (Placement(transformation(extent={{-14,20},{14,48}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outlet
    annotation (Placement(transformation(extent={{-14,-48},{14,-20}})));
equation
  inlet.Q_flow = ht_correlation.ht*A*(inlet.T - outlet.T);
  inlet.Q_flow = - outlet.Q_flow;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{90,8},{-90,8}},     color={0,127,255}),
        Line(points={{78,-16},{90,-10}},   color={0,127,255}),
        Line(points={{90,-10},{-90,-10}}, color={0,127,255}),
        Line(points={{78,-4},{90,-10}},    color={0,127,255}),
        Line(points={{78,2},{90,8}},       color={0,127,255}),
        Line(points={{78,14},{90,8}},      color={0,127,255})}), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end InternalConvection;
