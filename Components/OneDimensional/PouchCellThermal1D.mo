within DynTherM.Components.OneDimensional;
model PouchCellThermal1D
  "Thermal model of a pouch cell featuring 1D discretization"

  replaceable model Mat = Materials.PolestarCell constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  model CV_cell = Components.OneDimensional.PouchCellThermalCV
    "Control volume for cell";

  // Geometry
  parameter Area A "Base surface Area" annotation (Dialog(tab="Geometry"));
  parameter Length H "Cell Height" annotation (Dialog(tab="Geometry"));

  // Initialization
  parameter Temperature Tstart "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter DynTherM.Choices.InitOpt initOpt  "Initialization option" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N(min=1) "Number of vertical sections in which the cell is discretized";

  CV_cell Cell[N](
    redeclare model Mat = Mat,
    each H=H/N,
    each A=A,
    each Tstart=Tstart,
    each initOpt=initOpt,
    each Q_gen=Q/N);

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a Average annotation (
      Placement(transformation(extent={{-10,-6},{4,8}}), iconTransformation(
          extent={{-10,-6},{4,8}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b Top annotation (Placement(
        transformation(extent={{-10,54},{4,68}}), iconTransformation(extent={{-10,
            54},{4,68}})));

  Modelica.Blocks.Interfaces.RealInput Q annotation (Placement(transformation(
          extent={{-98,-8},{-82,8}}), iconTransformation(extent={{-98,-8},{-82,8}})));

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b Bottom annotation (
      Placement(transformation(extent={{-10,54},{4,68}}), iconTransformation(
          extent={{-10,-68},{4,-54}})));

equation
  Average.T = sum(Cell.T_vol)/N;

  // Connections for Side Conduction
  for i in 1:(N-1) loop
    connect(Cell[i].outlet, Cell[i+1].inlet);
  end for;

  // Convection Connection
  connect(Cell[1].inlet, Bottom);
  connect(Cell[end].outlet, Top);

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
        coordinateSystem(preserveAspectRatio=false)));
end PouchCellThermal1D;
