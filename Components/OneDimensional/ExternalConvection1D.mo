within DynTherM.Components.OneDimensional;
model ExternalConvection1D
  "External convection model implementing 1D spatial discretization"

  outer Components.Environment environment "Environmental properties";
  replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
    Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
  model CV = DynTherM.Components.HeatTransfer.ExternalConvectionFlux "Control volume";

  replaceable model HTC =
    DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue
    constrainedby
    DynTherM.Components.HeatTransfer.HTCorrelations.BaseClassExternal(
      T_skin=inlet.T) annotation (choicesAllMatching=true);

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
          extent={{-60,-60},{60,40}})));

equation
  for i in 1:N loop
    connect(outlet.ports[i,1], cv[i].inlet);
    connect(outlet.ports[i,1], inlet.ports[i,1]);
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,20},{100,0}},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Line(points={{90,-30},{-90,-30}}, color={0,127,255}),
        Line(points={{78,-36},{90,-30}},   color={0,127,255}),
        Line(points={{90,-50},{-90,-50}}, color={0,127,255}),
        Line(points={{78,-24},{90,-30}},   color={0,127,255}),
        Line(points={{78,-56},{90,-50}},   color={0,127,255}),
        Line(points={{78,-44},{90,-50}},   color={0,127,255})}),
                                               Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end ExternalConvection1D;
