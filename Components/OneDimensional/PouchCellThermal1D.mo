within DynTherM.Components.OneDimensional;
model PouchCellThermal1D
  "Thermal model of a pouch cell featuring 1D discretization"

  replaceable model Mat = Materials.PolestarCell constrainedby
    Materials.Properties "Material choice" annotation (choicesAllMatching=true);
  model CV_cell = Components.OneDimensional.PouchCellThermalCV
    "Control volume for cell";

  // Geometry
  parameter Area A  "Base surface Area" annotation (Dialog(tab="Geometry"));
  parameter Length h "Cell Height" annotation (Dialog(tab="Geometry"));

  // Initialization
  parameter Temperature Tstart "Temperature start value" annotation (Dialog(tab="Initialization"));
  parameter DynTherM.Choices.InitOpt initOpt  "Initialization option" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N(min=1) "Number of vertical sections in which the cell is discretized";

  CV_cell Cell[N](
    redeclare model Mat = Mat,
    each t=h/N,
    each A=A,
    each Tstart=Tstart,
    each initOpt=initOpt,
    each Q_gen=Heat_gen/N);

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a Convection_Port
    annotation (Placement(transformation(extent={{-10,-90},{10,-70}}),
        iconTransformation(extent={{-10,-90},{10,-70}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b AvgT_port annotation (
      Placement(transformation(extent={{-68,-4},{-60,4}}), iconTransformation(
          extent={{-68,-4},{-60,4}})));

  Modelica.Blocks.Interfaces.RealInput Heat_gen annotation (Placement(
        transformation(extent={{74,-8},{60,6}}), iconTransformation(extent={{74,
            -8},{60,6}})));

equation
  AvgT_port.T =  sum(Cell.T_vol)/N;

  // Connections for Side Conduction
  for i in 1:(N-1) loop
    connect(Cell[i].outlet, Cell[i+1].inlet);
  end for;

  // Convection Connection
  connect(Cell[1].inlet, Convection_Port);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                      Rectangle(
          extent={{-90,5},{90,-5}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward,
          origin={55,10},
          rotation=90),
                      Rectangle(
          extent={{-90,5},{90,-5}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward,
          origin={-55,10},
          rotation=90),
        Line(
          points={{-60,20},{-60,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{0,22},{0,-78}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-28,70},
          rotation=90),
        Line(
          points={{60,20},{60,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash),
                      Rectangle(
          extent={{-60,5},{60,-5}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Backward,
          origin={0,95},
          rotation=180),
        Line(
          points={{0,22},{0,-78}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-28,50},
          rotation=90),
        Line(
          points={{0,22},{0,-78}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-28,30},
          rotation=90),
        Line(
          points={{0,22},{0,-78}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-28,10},
          rotation=90),
        Line(
          points={{0,22},{0,-78}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-28,-12},
          rotation=90),
        Line(
          points={{0,22},{0,-78}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-28,-30},
          rotation=90),
        Line(
          points={{0,22},{0,-78}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-28,-50},
          rotation=90),
        Line(
          points={{0,22},{0,-78}},
          color={0,0,0},
          pattern=LinePattern.Dash,
          origin={-28,-70},
          rotation=90),
        Text(
          extent={{-26,64},{26,-44}},
          textColor={0,0,0},
          fontName="Arial Black",
          textString="Cell")}),                Diagram(coordinateSystem(
          preserveAspectRatio=false)),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end PouchCellThermal1D;
