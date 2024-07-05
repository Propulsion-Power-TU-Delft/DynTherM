within DynTherM.Components.OneDimensional;
model PouchCellThermal1D
  "Thermal model of a pouch cell implementing 1D discretization in vertical direction"

  replaceable model Mat = Materials.PolestarCellInPlane
                                                 constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  model CV = Components.OneDimensional.PouchCellThermalCV
    "Control volume for cell";

  // Geometry
  parameter Area A "Base surface area" annotation (Dialog(tab="Geometry"));
  parameter Length H "Height" annotation (Dialog(tab="Geometry"));

  // Initialization
  parameter Temperature Tstart "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter DynTherM.Choices.InitOpt initOpt  "Initialization option" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N(min=1) "Number of vertical sections in which the cell is discretized";

  CV cv[N](
    redeclare model Mat=Mat,
    each H=H/N,
    each A=A,
    each Tstart=Tstart,
    each initOpt=initOpt);

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b Top annotation (Placement(
        transformation(extent={{-10,70},{10,90}}),iconTransformation(extent={{-10,
            54},{6,70}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a Average annotation (
      Placement(transformation(extent={{-10,-10},{10,10}}),iconTransformation(
          extent={{-10,-8},{6,8}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b Bottom annotation (
      Placement(transformation(extent={{-10,-90},{10,-70}}),
                                                           iconTransformation(
          extent={{-10,-70},{6,-54}})));
  CustomInterfaces.DistributedHeatPort_B Distributed(Nx=N, Ny=1) annotation (
      Placement(transformation(
        extent={{-20,-10},{20,10}},
        rotation=-90,
        origin={-20,3.55271e-15}), iconTransformation(
        extent={{-40,-13},{40,13}},
        rotation=-90,
        origin={-85,0})));

equation

  // Port connections
  Average.T = sum(cv.T_vol)/N;

  for i in 1:N loop
    cv[i].Q_gen = Average.Q_flow/N;
    cv[i].T_vol = Distributed.ports[i,1].T;
  end for;

  // Internal connections
  for i in 1:(N-1) loop
    connect(cv[i].outlet, cv[i+1].inlet);
  end for;

  // Boundary connections
  connect(cv[1].inlet, Bottom);
  connect(cv[end].outlet, Top);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Bitmap(
          extent={{-88,-88},{88,88}},
          fileName="modelica://DynTherM/Figures/PouchCell.PNG",
          origin={6,0},
          rotation=-90),
        Line(
          points={{0,24},{0,-100}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-52,-26},
          rotation=90),
        Line(
          points={{0,24},{0,-100}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-52,0},
          rotation=90),
        Line(
          points={{0,24},{0,-100}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-52,26},
          rotation=90)}),                      Diagram(coordinateSystem(
          preserveAspectRatio=false)),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Heat generation is assumed uniform over the vertical control volumes.</p>
<p>Heat conduction is modelled only in-plane, whereas cross-plane heat conduction is disregarded.</p>
</html>"));
end PouchCellThermal1D;
