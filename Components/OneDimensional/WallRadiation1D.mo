within DynTherM.Components.OneDimensional;
model WallRadiation1D
  "Wall radiation model implementing 1D spatial discretization in longitudinal direction"

  outer DynTherM.Components.Environment environment "Environmental properties";
  replaceable model Material =
    DynTherM.Materials.Paints.WhiteCoatings.CatalacWhitePaint
    constrainedby DynTherM.Materials.Paints.BasePaint "Material choice" annotation (choicesAllMatching=true);
  Material Mat;
  model CV = DynTherM.Components.HeatTransfer.WallRadiationFlux "Control volume";

  parameter Modelica.Units.SI.Angle csi
    "Tilt angle of the surface wrt horizontal";

  // Discretization
  parameter Integer N(min=1) "Number of control volumes";

  CV cv[N](each csi=csi);

  CustomInterfaces.DistributedHeatFluxPort_A inlet(Nx=N, Ny=1) annotation (
      Placement(transformation(extent={{-60,-20},{60,80}}), iconTransformation(
          extent={{-60,-20},{60,80}})));
  CustomInterfaces.DistributedIrradiancePort outlet(Nx=N, Ny=1) annotation (
      Placement(transformation(extent={{-60,-60},{60,40}}), iconTransformation(
          extent={{-60,-60},{60,40}})));

equation
  for i in 1:N loop
    connect(outlet.ports[i,1], cv[i].outlet);
    connect(inlet.ports[i,1], cv[i].inlet);
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,20},{100,0}},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Line(points={{-60,-94},{-60,-26}},color={238,46,47}),
        Line(points={{-30,-94},{-30,-26}},color={238,46,47}),
        Line(points={{-60,-26},{-66,-36}},color={238,46,47}),
        Line(points={{-60,-26},{-54,-36}},color={238,46,47}),
        Line(points={{-30,-26},{-24,-36}},color={238,46,47}),
        Line(points={{-30,-26},{-36,-36}},color={238,46,47}),
        Line(points={{30,-94},{30,-26}},  color={238,46,47}),
        Line(points={{60,-94},{60,-26}},  color={238,46,47}),
        Line(points={{54,-84},{60,-94}}, color={238,46,47}),
        Line(points={{24,-84},{30,-94}}, color={238,46,47}),
        Line(points={{66,-84},{60,-94}}, color={238,46,47}),
        Line(points={{36,-84},{30,-94}}, color={238,46,47})}),
                                               Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end WallRadiation1D;
