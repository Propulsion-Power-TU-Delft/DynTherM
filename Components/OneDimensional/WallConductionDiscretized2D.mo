within DynTherM.Components.OneDimensional;
model WallConductionDiscretized2D
  "Dynamic model of conduction in a planar surface implementing 1D discretization in vertical direction"

  replaceable model Mat = Materials.Aluminium constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  model CV = DynTherM.Components.HeatTransfer.WallConduction2D "Control volume";

  // Geometry
  input Length t "Wall thickness" annotation (Dialog(enable=true));
  input Length h "Wall height, dimension perpendicular to thickness along heat transfer" annotation (Dialog(enable=true));
  input Area A "Wall surface" annotation (Dialog(enable=true));

  // Initialization
  parameter Temperature Tstart "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter DynTherM.Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N(min=1) "Number of vertical sections in which the wall is discretized";

  CV cv[N](
    redeclare model Mat=Mat,
    each t=t,
    each h=h,
    each A=A/N,
    each Tstart=Tstart,
    each initOpt=initOpt);

  CustomInterfaces.DistributedHeatPort_B outlet(Nx=N, Ny=1) annotation (
      Placement(transformation(
        extent={{-20,-10},{20,10}},
        rotation=-90,
        origin={20,3.55271e-15}),iconTransformation(
        extent={{-60,-48},{60,48}},
        rotation=180,
        origin={7.10543e-15,-30})));
  CustomInterfaces.DistributedHeatPort_A inlet(Nx=N, Ny=1) annotation (
      Placement(transformation(
        extent={{-20,-10},{20,10}},
        rotation=-90,
        origin={-20,3.55271e-15}),
                                 iconTransformation(
        extent={{-60,-48},{60,48}},
        rotation=180,
        origin={7.10543e-15,30})));

  CustomInterfaces.DistributedHeatPort_B outletH(Nx=N, Ny=1)
                                                            annotation (
      Placement(transformation(
        extent={{-20,-10},{20,10}},
        rotation=-90,
        origin={20,3.55271e-15}),iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-96,0})));
  CustomInterfaces.DistributedHeatPort_A inletH(Nx=N, Ny=1)
                                                           annotation (
      Placement(transformation(
        extent={{-20,-10},{20,10}},
        rotation=-90,
        origin={-20,3.55271e-15}),
                                 iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={96,0})));
equation
  for i in 1:N loop
    connect(outlet.ports[i,1], cv[i].outlet);
    connect(inlet.ports[i,1], cv[i].inlet);
  end for;
  for i in 1:N loop
    connect(outletH.ports[i,1], cv[i].outletH);
    connect(inletH.ports[i,1], cv[i].inletH);
  end for;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                      Rectangle(
          extent={{-100,20},{98,-20}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward),
        Line(
          points={{2,-42},{-38,-42}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-22,18},
          rotation=90),
        Line(
          points={{-20,44},{-60,44}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={24,40},
          rotation=90),
        Line(
          points={{-40,4},{-2.28848e-17,4}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-56,20},
          rotation=90),
        Text(
          extent={{-32,32},{32,-32}},
          lineColor={255,255,255},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="WALL"),
        Line(
          points={{-40,4},{-2.28848e-17,4}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={64,20},
          rotation=90)}),                      Diagram(coordinateSystem(
          preserveAspectRatio=false)),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Heat transfer is modelled only in transversal direction, i.e., through the wall thickness.</p>
<p>Heat transfer in vertical direction, i.e., among adjacent wall control volumes, is neglected.</p>
</html>"));
end WallConductionDiscretized2D;
