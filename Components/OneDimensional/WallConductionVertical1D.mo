within DynTherM.Components.OneDimensional;
model WallConductionVertical1D
  "Dynamic model of 1D heat conduction in a planar surface discretised along the vertical direction"

  replaceable model Mat=Materials.Aluminium constrainedby Materials.Properties
    "Material properties" annotation (choicesAllMatching=true);
  model CV = Components.HeatTransfer.WallConduction "Control volume";

  // Geometry
  input Length x "Horizontal length" annotation (Dialog(enable=true));
  input Length y "Vertical length" annotation (Dialog(enable=true));
  input Length z "Thickness, out of plane" annotation (Dialog(enable=true));

  // Initialization
  parameter Temperature Tstart=300 "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N(min=1) "Number of horizontal sections in which the wall is discretized";

  CV cv[N](
    redeclare model Mat=Mat,
    each t=x,
    each A=A/N,
    each N=1,
    each Tstart=Tstart,
    each initOpt=initOpt);

  Mass m "Mass";
  Volume V "Volume";
  Area A "Main heat transfer area";

  CustomInterfaces.OneDimensional.HeatPort1D_B West(Nx=N) annotation (
      Placement(transformation(
        extent={{-20,-10},{20,10}},
        rotation=-90,
        origin={20,3.55271e-15}), iconTransformation(
        extent={{-60,-48},{60,48}},
        rotation=90,
        origin={-30,0})));
  CustomInterfaces.OneDimensional.HeatPort1D_A East(Nx=N) annotation (
      Placement(transformation(
        extent={{-20,-10},{20,10}},
        rotation=-90,
        origin={-20,3.55271e-15}), iconTransformation(
        extent={{-60,-48},{60,48}},
        rotation=90,
        origin={30,0})));

equation
  m = sum(cv.m);
  A = y*z;
  V = x*y*z;

  // Port connections
  for i in 1:N loop
    connect(West.ports[i], cv[i].outlet);
    connect(East.ports[i], cv[i].inlet);
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                      Rectangle(
          extent={{-20,100},{20,-100}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Line(
          points={{2,-42},{2,-82}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-62,58},
          rotation=90),
        Line(
          points={{-20,44},{-20,4}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={24,0},
          rotation=90),
        Line(
          points={{0,-36},{0,4}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-16,-60},
          rotation=90),
        Text(
          extent={{-32,32},{32,-32}},
          lineColor={255,255,255},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="WALL",
          rotation=-90),
        Line(
          points={{2,-42},{2,-82}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-62,20},
          rotation=90)}),                      Diagram(coordinateSystem(
          preserveAspectRatio=false)),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Heat transfer is modelled only in transversal direction, i.e., through the wall thickness.</p>
<p>Heat transfer in vertical direction, i.e., among adjacent wall control volumes, is neglected.</p>
</html>"));
end WallConductionVertical1D;
