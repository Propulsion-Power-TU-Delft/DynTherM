within DynTherM.Components.OneDimensional;
model ExternalConvection1D
  "External convection model implementing 1D spatial discretization"

  outer Components.Environment environment "Environmental properties";
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  model CV = DynTherM.Components.HeatTransfer.ExternalConvection "Control volume";

  replaceable model HTC =
    DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClasses.BaseClassExternal
    annotation (choicesAllMatching=true);

  input Area A "Heat transfer area (total)" annotation (Dialog(enable=true));

  // Discretization
  parameter Integer N(min=1) "Number of control volumes";

  CV cv[N](
    redeclare model HTC = HTC,
    redeclare package Medium = Medium,
    each A=A_cv);

  Area A_cv "Heat transfer area associated with one control volume";

  CustomInterfaces.OneDimensional.HeatPort1D_A inlet(Nx=N)
    annotation (Placement(transformation(extent={{-60,-20},{60,80}}),
        iconTransformation(extent={{-60,-20},{60,80}})));

equation
  A_cv = A/N;

  for i in 1:N loop
    connect(cv[i].inlet, inlet.ports[i]);
  end for;

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
        Line(points={{78,-24},{90,-30}},   color={0,127,255})}),
                                               Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end ExternalConvection1D;
