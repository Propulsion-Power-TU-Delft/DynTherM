within DynTherM.Systems.Battery;
model PouchModuleParallel "Battery module made of pouch cells"

  replaceable model InPlaneCellMat = Materials.PolestarCellInPlane constrainedby
    Materials.Properties "In-plane cell material properties" annotation (choicesAllMatching=true);

  replaceable model CrossPlaneCellMat = Materials.PolestarCellCrossPlane constrainedby
    Materials.Properties "Cross-plane cell material properties" annotation (choicesAllMatching=true);

  replaceable model FirewallMat = Materials.CompressionPadFoam constrainedby
    Materials.Properties "Firewall material" annotation (choicesAllMatching=true);

  replaceable model ResinMat = Materials.ThermalResin constrainedby
    Materials.Properties "Resin material" annotation (choicesAllMatching=true);

  replaceable model FrameMat = Materials.AluminiumFoil constrainedby
    Materials.Properties "Frame material" annotation (choicesAllMatching=true);

  model Cell = Components.Electrical.PouchCell "Cell";
  model Firewall = Components.TwoDimensional.WallConductionVertical2D
    "Firewall in between adjacent cells";

  // Geometry
  parameter Length W_cell "Cell width" annotation (Dialog(tab="Geometry"));
  parameter Length H_cell "Cell height" annotation (Dialog(tab="Geometry"));
  parameter Length t_cell "Cell thickness" annotation (Dialog(tab="Geometry"));
  parameter Length t_fw "Thickness of firewall between cells in parallel" annotation (Dialog(tab="Geometry"));
  parameter Length t_resin "Thickness of resin between cells and frame" annotation (Dialog(tab="Geometry"));
  parameter Length t_frame "Frame thickness" annotation (Dialog(tab="Geometry"));

  // Electrical parameters
  parameter Real eta=0.98 "Cell charging/discharging efficiency";
  parameter ElectricCharge C_nom "Nominal cell capacity";

  // Initialization
  parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
    "Initialization option" annotation (Dialog(tab="Initialization"));
  parameter Real SoC_start "State of charge - start value" annotation (Dialog(tab="Initialization"));
  parameter Temperature Tstart=298.15 "Temperature - start value" annotation (Dialog(tab="Initialization"));

  // Discretization
  parameter Integer N_cv(min=1)=10 "Number of vertical control volumes in which each cell is discretized";
  parameter Integer Ns(min=1) "Number of cells connected in series";
  parameter Integer Np(min=2) "Number of cells connected in parallel";

  Length W_module "Module width";
  Length H_module "Module height";
  Length t_module "Module thickness";
  Volume V_module "Volume of the battery module";
  Mass m_module "Mass of the battery module";
  Real V_overhead "Volume overhead [%]";
  Real m_overhead "Mass overhead [%]";
  Temperature T_avg "Average temperature of the moduel";

  Cell cell[Ns,Np](
    redeclare model InPlaneMat=InPlaneCellMat,
    redeclare model CrossPlaneMat=CrossPlaneCellMat,
    each W=W_cell,
    each H=H_cell,
    each t=t_cell,
    each eta=eta,
    each C_nom=C_nom,
    each SoC_start=SoC_start,
    each Tstart=Tstart,
    each initOpt=initOpt,
    each N=N_cv);

  Firewall firewall[Ns*Np - 1](
    redeclare model MatX=FirewallMat,
    redeclare model MatY=FirewallMat,
    each x=t_fw,
    each y=H_cell,
    each z=W_cell,
    each Tstart=Tstart,
    each initOpt=initOpt,
    each N=N_cv);

  CustomInterfaces.OneDimensional.HeatPort1D_A Bottom(Nx=Ns*Np) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={0,-80}), iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=0,
        origin={-20,-66})));
  CustomInterfaces.OneDimensional.HeatPort1D_A Top(Nx=Ns*Np) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={0,80}), iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=0,
        origin={-20,26})));
  Modelica.Electrical.Analog.Interfaces.PositivePin p annotation (Placement(
        transformation(extent={{-108,-8},{-92,8}}),    iconTransformation(
          extent={{-66,-26},{-54,-14}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n annotation (Placement(
        transformation(extent={{108,-8},{92,8}}),     iconTransformation(extent={{14,-26},
            {26,-14}})));
  Modelica.Electrical.Analog.Sensors.MultiSensor multiSensor annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Modelica.Electrical.Batteries.Interfaces.CellBus batteryBus
    "Battery bus (average / sum over all cells)" annotation (Placement(
        transformation(extent={{-68,-70},{-32,-34}}), iconTransformation(extent={{-10,30},
            {10,50}})));
  CustomInterfaces.OneDimensional.HeatPort1D_A Left(Nx=N_cv) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={-80,40}), iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=-90,
        origin={-76,-20})));
  CustomInterfaces.OneDimensional.HeatPort1D_A Right(Nx=N_cv) annotation (Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={80,40}), iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=-90,
        origin={36,-20})));
  Components.TwoDimensional.WallConductionHorizontal2D resin_bottom(
    redeclare model MatX = ResinMat,
    redeclare model MatY = ResinMat,
    x=t_module,
    y=t_resin,
    z=W_module,
    Tstart=Tstart,
    initOpt=initOpt,
    N=Ns*Np) "Layer of thermal resin applied on bottom surface"
    annotation (Placement(transformation(extent={{-14,-54},{14,-26}})));
  Components.TwoDimensional.WallConductionHorizontal2D frame_top(
    redeclare model MatX = FrameMat,
    redeclare model MatY = FrameMat,
    x=t_module,
    y=t_frame,
    z=W_module,
    Tstart=Tstart,
    initOpt=initOpt,
    N=Ns*Np) "Top portion of external frame"
    annotation (Placement(transformation(extent={{-14,46},{14,74}})));
  Components.TwoDimensional.WallConductionHorizontal2D frame_bottom(
    redeclare model MatX = FrameMat,
    redeclare model MatY = FrameMat,
    x=t_module,
    y=t_frame,
    z=W_module,
    Tstart=Tstart,
    initOpt=initOpt,
    N=Ns*Np) "Bottom portion of external frame"
    annotation (Placement(transformation(extent={{-14,-74},{14,-46}})));
  Components.TwoDimensional.WallConductionHorizontal2D resin_top(
    redeclare model MatX = ResinMat,
    redeclare model MatY = ResinMat,
    x=t_module,
    y=t_resin,
    z=W_module,
    Tstart=Tstart,
    initOpt=initOpt,
    N=Ns*Np) "Layer of thermal resin applied on top surface"
    annotation (Placement(transformation(extent={{-14,26},{14,54}})));
  Components.TwoDimensional.WallConductionVertical2D frame_left(
    redeclare model MatX=FrameMat,
    redeclare model MatY=FrameMat,
    x=t_frame,
    y=H_module,
    z=W_module,
    Tstart=Tstart,
    initOpt=initOpt,
    N=N_cv) "Left portion of external frame"
    annotation (Placement(
        transformation(
        extent={{14,14},{-14,-14}},
        rotation=180,
        origin={-60,40})));
  Components.TwoDimensional.WallConductionVertical2D frame_right(
    redeclare model MatX=FrameMat,
    redeclare model MatY=FrameMat,
    x=t_frame,
    y=H_module,
    z=W_module,
    Tstart=Tstart,
    initOpt=initOpt,
    N=N_cv) "Right portion of external frame"
    annotation (Placement(
        transformation(
        extent={{14,-14},{-14,14}},
        rotation=180,
        origin={60,40})));

equation

  T_avg = (sum(cell.thermal.Average.T))/(Ns*Np);

  // Geometry
  W_module = W_cell + 2*t_frame;
  H_module = H_cell + 2*t_resin + 2*t_frame;
  t_module = Np*Ns*t_cell + (Np*Ns - 1)*t_fw + 2*t_frame;
  V_module = W_module*H_module*t_module;
  m_module = sum(cell.m) + sum(firewall.m) +
    resin_top.m + resin_bottom.m + frame_top.m + frame_bottom.m +
    frame_left.m + frame_right.m;
  V_overhead = (V_module/sum(cell.V) - 1)*100;
  m_overhead = (m_module/sum(cell.m) - 1)*100;

  // ------------------------------- ELECTRICAL --------------------------------
  // External
  connect(multiSensor.nc, cell[1,1].p);
  connect(cell[Ns,1].n, n);

  // Internal
  // Parallel
  for ks in 1:Ns loop
    for kp in 1:(Np - 1) loop
      connect(cell[ks, kp].p, cell[ks, kp + 1].p);
      connect(cell[ks, kp].n, cell[ks, kp + 1].n);
    end for;
  end for;

  // Series
  for ks in 1:(Ns - 1) loop
    connect(cell[ks, 1].n, cell[ks + 1, 1].p);
  end for;

  // -------------------------------- THERMAL ----------------------------------
  // External
  for ks in 1:Ns loop
    for kp in 1:Np loop
      connect(cell[ks, kp].Top, resin_top.South.ports[Np*(ks - 1) + kp]);
      connect(cell[ks, kp].Bottom, resin_bottom.North.ports[Np*(ks - 1) + kp]);
    end for;
  end for;

  connect(frame_left.East, cell[1, 1].Left);
  connect(frame_right.West, cell[Ns, Np].Right);

  // Internal
  for ks in 1:Ns loop
    for kp in 1:(Np - 1) loop
      connect(cell[ks, kp].Right, firewall[Np*(ks - 1) + kp].West);
      connect(firewall[Np*(ks - 1) + kp].East, cell[ks, kp + 1].Left);
    end for;
  end for;

  for ks in 1:(Ns - 1) loop
    connect(cell[ks, Np].Right, firewall[Np*(ks - 1) + Np].West);
    connect(firewall[Np*(ks - 1) + Np].East, cell[ks + 1, 1].Left);
  end for;

  connect(p, multiSensor.pc) annotation (Line(points={{-100,0},{-60,0}}, color={0,0,255}));
  connect(multiSensor.pc, multiSensor.pv) annotation (Line(points={{-60,0},{-60,10},{-50,10}}, color={0,0,255}));
  connect(n, multiSensor.nv) annotation (Line(points={{100,0},{100,-20},{-50,
          -20},{-50,-10}},
                      color={0,0,255}));
  connect(multiSensor.v, batteryBus.v) annotation (Line(points={{-44,-11},{-44,
          -52},{-50,-52},{-50,-51.91},{-49.91,-51.91}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(multiSensor.i, batteryBus.i) annotation (Line(points={{-56,-11},{-56,
          -51.91},{-49.91,-51.91}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(multiSensor.power, batteryBus.power) annotation (Line(points={{-61,-6},
          {-70,-6},{-70,-51.91},{-49.91,-51.91}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(Top, frame_top.North)
    annotation (Line(points={{0,80},{0,64.2}}, color={191,0,0}));
  connect(frame_top.South, resin_top.North)
    annotation (Line(points={{0,55.8},{0,44.2}}, color={191,0,0}));
  connect(resin_bottom.South, frame_bottom.North)
    annotation (Line(points={{0,-44.2},{0,-55.8}}, color={191,0,0}));
  connect(frame_bottom.South, Bottom)
    annotation (Line(points={{0,-64.2},{0,-80}}, color={191,0,0}));
  connect(Left, frame_left.West)
    annotation (Line(points={{-80,40},{-64.2,40}}, color={191,0,0}));
  connect(frame_right.East, Right)
    annotation (Line(points={{64.2,40},{80,40}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-70,20},{-50,-60}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-64,-4},{-56,-36}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(points={{-70,20},{-30,60},{-10,60},{-50,20}}, color={0,0,0}),
        Line(points={{-10,60},{-10,-20},{-50,-60}}, color={0,0,0}),
        Rectangle(
          extent={{-30,20},{-10,-60}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-24,-4},{-16,-36}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(points={{-30,20},{10,60},{30,60},{-10,20}}, color={0,0,0}),
        Line(points={{30,60},{30,-20},{-10,-60}}, color={0,0,0}),
        Rectangle(
          extent={{10,20},{30,-60}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{16,-4},{24,-36}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(points={{10,20},{50,60},{70,60},{30,20}}, color={0,0,0}),
        Line(points={{70,60},{70,-20},{30,-60}}, color={0,0,0})}),
                                                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>Pouch cells are electrically connected in series/parallel, see image below.</p>
<p>However, they are physically stacked in parallel.</p>
<p><img src=\"modelica://DynTherM/Figures/pouch_cell_module.png\"/></p>
</html>"));
end PouchModuleParallel;
