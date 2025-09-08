within DynTherM.Components.HeatTransfer;
model VariableThermalConductor "Lumped thermal element transporting heat without storing it"

public
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inlet annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outlet annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Blocks.Interfaces.RealInput G "Variable thermal conductance" annotation (Placement(transformation(
        extent={{-14,-14},{14,14}},
        rotation=-90,
        origin={0,74}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,72})));

equation
  inlet.Q_flow = G*(inlet.T - outlet.T);
  inlet.Q_flow + outlet.Q_flow = 0;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-86,70},{86,-70}},
          pattern=LinePattern.None,
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Line(
          points={{-88,70},{-88,-70}},
          thickness=0.5),
        Line(
          points={{88,70},{88,-70}},
          thickness=0.5)}),                                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end VariableThermalConductor;
