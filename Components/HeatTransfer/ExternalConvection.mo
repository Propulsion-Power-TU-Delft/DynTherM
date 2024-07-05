within DynTherM.Components.HeatTransfer;
model ExternalConvection "Model of external convection"

  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  outer Components.Environment environment "Environmental properties";

  input Area A "Heat transfer area" annotation (Dialog(enable=true));

  replaceable model HTC =
    Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue
    constrainedby
    Components.HeatTransfer.HTCorrelations.BaseClassExternal(
      T_skin=inlet.T,
      ht_fixed=1) annotation (choicesAllMatching=true);

  HTC ht_correlation;

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet
    annotation (Placement(transformation(extent={{-14,20},{14,48}})));

equation
  inlet.Q_flow = ht_correlation.ht*A*(inlet.T - ht_correlation.T_out);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,20},{100,0}},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Line(points={{90,-10},{-90,-10}}, color={0,127,255}),
        Line(points={{78,-16},{90,-10}},   color={0,127,255}),
        Line(points={{90,-30},{-90,-30}}, color={0,127,255}),
        Line(points={{78,-4},{90,-10}},    color={0,127,255}),
        Line(points={{78,-36},{90,-30}},   color={0,127,255}),
        Line(points={{78,-24},{90,-30}},   color={0,127,255})}), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ExternalConvection;
