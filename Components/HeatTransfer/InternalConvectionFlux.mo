within DynTherM.Components.HeatTransfer;
model InternalConvectionFlux "Model of internal convection per unit area"

  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

  replaceable model HTC =
    Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue
    constrainedby
    Components.HeatTransfer.HTCorrelations.BaseClassInternal
    annotation (choicesAllMatching=true);

  HTC ht_correlation;

  DynTherM.CustomInterfaces.ZeroDimensional.HeatFluxPort_A inlet
    annotation (Placement(transformation(extent={{-14,20},{14,48}})));
  DynTherM.CustomInterfaces.ZeroDimensional.HeatFluxPort_B outlet
    annotation (Placement(transformation(extent={{-14,-48},{14,-20}})));

equation
  inlet.phi = ht_correlation.ht*(inlet.T - outlet.T);
  inlet.phi + outlet.phi = 0;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{90,8},{-90,8}},     color={0,127,255}),
        Line(points={{78,-16},{90,-10}},   color={0,127,255}),
        Line(points={{90,-10},{-90,-10}}, color={0,127,255}),
        Line(points={{78,-4},{90,-10}},    color={0,127,255}),
        Line(points={{78,2},{90,8}},       color={0,127,255}),
        Line(points={{78,14},{90,8}},      color={0,127,255})}), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end InternalConvectionFlux;
