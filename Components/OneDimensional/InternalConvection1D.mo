within DynTherM.Components.OneDimensional;
model InternalConvection1D
  "Internal convection model implementing 1D spatial discretization"

  outer Components.Environment environment "Environmental properties";
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  model CV = DynTherM.Components.HeatTransfer.InternalConvectionFlux "Control volume";

  replaceable model HTC =
    DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection.FixedValue
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassInternal
    annotation (choicesAllMatching=true);

  // Discretization
  parameter Integer N(min=1) "Number of control volumes";

  CV cv[N](
    redeclare model HTC = HTC,
    redeclare package Medium = Medium);

  CustomInterfaces.DistributedHeatFluxPort_A inlet(
    Nx=N,
    Ny=1)
    annotation (
      Placement(transformation(extent={{-60,-20},{60,80}}), iconTransformation(
          extent={{-60,-20},{60,80}})));

  CustomInterfaces.DistributedHeatFluxPort_B outlet(
    Nx=N,
    Ny=1)
    annotation (
      Placement(transformation(extent={{-60,-20},{60,80}}), iconTransformation(
          extent={{-60,-80},{60,20}})));

equation
  for i in 1:N loop
    connect(outlet.ports[i,1], cv[i].inlet);
    connect(inlet.ports[i,1], cv[i].outlet);
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{92,10},{-88,10}},   color={0,127,255}),
        Line(points={{80,4},{92,10}},      color={0,127,255}),
        Line(points={{92,-10},{-88,-10}}, color={0,127,255}),
        Line(points={{80,16},{92,10}},     color={0,127,255}),
        Line(points={{80,-16},{92,-10}},   color={0,127,255}),
        Line(points={{80,-4},{92,-10}},    color={0,127,255})}),
                                               Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end InternalConvection1D;
