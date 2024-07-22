within DynTherM.Systems.Battery;
model PouchModule "Battery module made of pouch cells"

  replaceable model InPlaneCellMat = Materials.PolestarCellInPlane constrainedby
    Materials.Properties "In-plane cell material properties" annotation (choicesAllMatching=true);

  replaceable model CrossPlaneCellMat = Materials.PolestarCellCrossPlane constrainedby
    Materials.Properties "Cross-plane cell material properties" annotation (choicesAllMatching=true);

  replaceable model FirewallMat = Materials.PolyurethaneFoam constrainedby
    Materials.Properties "Firewall material" annotation (choicesAllMatching=true);

  replaceable model ResinMat = Materials.ThermalResin constrainedby
    Materials.Properties "Resin material" annotation (choicesAllMatching=true);

  replaceable model FrameMat = Materials.AluminiumFoil constrainedby
    Materials.Properties "Frame material" annotation (choicesAllMatching=true);

  model Cell = Components.Electrical.PouchCell1D "Cell";
  model Firewall = Components.TwoDimensional.WallConductionDiscretized
    "Firewall in between adjacent cells";

  // Geometry
  parameter Length W_cell "Cell width" annotation (Dialog(tab="Geometry"));
  parameter Length H_cell "Cell height" annotation (Dialog(tab="Geometry"));
  parameter Length t_cell "Cell thickness" annotation (Dialog(tab="Geometry"));
  parameter Length t_fw "Firewall thickness between cells in parallel" annotation (Dialog(tab="Geometry"));
  parameter Length t_gap "Air gap thickness between cells in series" annotation (Dialog(tab="Geometry"));
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
  parameter Integer Np(min=1) "Number of cells connected in parallel";

  Length W_module "Module width";
  Length H_module "Module height";
  Length t_module "Module thickness";
  Volume V_module "Volume of the battery module";

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

  Firewall firewall[Ns,Np-1](
    redeclare model Mat=FirewallMat,
    each A=W_cell*H_cell,
    each t=t_fw,
    each Tstart=Tstart,
    each initOpt=initOpt,
    each Nx=N_cv,
    each Ny=1);

  CustomInterfaces.DistributedHeatPort_A Bottom(Nx=Ns, Ny=Np) annotation (
      Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={0,-80}), iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=0,
        origin={-20,-66})));
  CustomInterfaces.DistributedHeatPort_A Top(Nx=Ns, Ny=Np) annotation (Placement(
        transformation(
        extent={{-12,-12},{12,12}},
        rotation=180,
        origin={0,80}),     iconTransformation(
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
  CustomInterfaces.DistributedHeatPort_A Left(Nx=Ns, Ny=N_cv) annotation (Placement(
        transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={-80,40}),iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=-90,
        origin={-76,-20})));
  CustomInterfaces.DistributedHeatPort_A Right(Nx=Ns, Ny=N_cv) annotation (
      Placement(transformation(
        extent={{-12,-12},{12,12}},
        rotation=90,
        origin={80,40}),iconTransformation(
        extent={{-30,-16},{30,16}},
        rotation=-90,
        origin={36,-20})));

  Components.TwoDimensional.WallConductionDiscretized frame_top(
    redeclare model Mat = FrameMat,
    t=t_frame,
    A=W_cell*(t_cell + t_fw/2),
    Tstart=Tstart,
    initOpt=initOpt,
    Nx=Ns,
    Ny=Np) "Top portion of external frame"
    annotation (Placement(transformation(extent={{-12,76},{12,56}})));
  Components.TwoDimensional.WallConductionDiscretized resin_top(
    redeclare model Mat = ResinMat,
    t=t_resin,
    A=W_cell*(t_cell + t_fw/2),
    Tstart=Tstart,
    initOpt=initOpt,
    Nx=Ns,
    Ny=Np) "Layer of thermal resin applied on top surface"
    annotation (Placement(transformation(extent={{-12,60},{12,40}})));
  Components.TwoDimensional.WallConductionDiscretized frame_left(
    redeclare model Mat = FrameMat,
    t=t_frame,
    A=W_cell*H_cell,
    Tstart=Tstart,
    initOpt=initOpt,
    Nx=Ns,
    Ny=N_cv)
            "Left portion of external frame" annotation (Placement(
        transformation(
        extent={{-12,10},{12,-10}},
        rotation=90,
        origin={-60,40})));
  Components.TwoDimensional.WallConductionDiscretized frame_right(
    redeclare model Mat = FrameMat,
    t=t_frame,
    A=W_cell*H_cell,
    Tstart=Tstart,
    initOpt=initOpt,
    Nx=Ns,
    Ny=N_cv)
            "Right portion of external frame" annotation (Placement(
        transformation(
        extent={{-12,-10},{12,10}},
        rotation=90,
        origin={60,40})));
  Components.TwoDimensional.WallConductionDiscretized resin_bottom(
    redeclare model Mat = ResinMat,
    t=t_resin,
    A=W_cell*(t_cell + t_fw/2),
    Tstart=Tstart,
    initOpt=initOpt,
    Nx=Ns,
    Ny=Np) "Layer of thermal resin applied on bottom surface"
    annotation (Placement(transformation(extent={{-12,-60},{12,-40}})));
  Components.TwoDimensional.WallConductionDiscretized frame_bottom(
    redeclare model Mat = FrameMat,
    t=t_frame,
    A=W_cell*(t_cell + t_fw/2),
    Tstart=Tstart,
    initOpt=initOpt,
    Nx=Ns,
    Ny=Np) "Bottom portion of external frame"
    annotation (Placement(transformation(extent={{-12,-76},{12,-56}})));

equation
  // Geometry
  W_module = Ns*W_cell + (Ns - 1)*t_gap + 2*t_frame;
  H_module = H_cell + 2*t_resin + 2*t_frame;
  t_module = Np*t_cell + (Np - 1)*t_fw + 2*t_frame;
  V_module = W_module*H_module*t_module;

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
      connect(cell[ks, kp].Top, resin_top.inlet.ports[ks, kp]);
      connect(cell[ks, kp].Bottom, resin_bottom.inlet.ports[ks, kp]);
    end for;
  end for;

  for ks in 1:Ns loop
    for j in 1:N_cv loop
      connect(cell[ks, 1].Left.ports[j, 1], frame_left.inlet.ports[ks, j]);
      connect(cell[ks, Np].Right.ports[j, 1], frame_right.inlet.ports[ks, j]);
    end for;
  end for;

  // Internal
  for ks in 1:Ns loop
    for kp in 1:(Np - 1) loop
      connect(cell[ks, kp].Right, firewall[ks, kp].inlet);
      connect(firewall[ks, kp].outlet, cell[ks, kp + 1].Left);
    end for;
  end for;

  connect(p, multiSensor.pc)
    annotation (Line(points={{-100,0},{-60,0}}, color={0,0,255}));
  connect(multiSensor.pc, multiSensor.pv)
    annotation (Line(points={{-60,0},{-60,10},{-50,10}}, color={0,0,255}));
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
  connect(resin_top.outlet, frame_top.inlet) annotation (Line(points={{8.88178e-16,
          53},{8.88178e-16,63}}, color={191,0,0}));
  connect(frame_top.outlet, Top) annotation (Line(points={{8.88178e-16,69},{8.88178e-16,
          74.5},{0,74.5},{0,80}}, color={191,0,0}));
  connect(resin_bottom.outlet, frame_bottom.inlet) annotation (Line(points={{8.88178e-16,
          -53},{8.88178e-16,-63}}, color={191,0,0}));
  connect(frame_bottom.outlet, Bottom) annotation (Line(points={{8.88178e-16,-69},
          {8.88178e-16,-74.5},{0,-74.5},{0,-80}}, color={191,0,0}));
  connect(Left, frame_left.outlet)
    annotation (Line(points={{-80,40},{-63,40}}, color={191,0,0}));
  connect(frame_right.outlet, Right)
    annotation (Line(points={{63,40},{80,40}}, color={191,0,0}));
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
<p>Pouch cells are physically stacked and electrically connected in series/parallel, see image below.</p>
<p><img src=\"modelica://DynTherM/Figures/pouch_cell_module.png\"/></p>
</html>"));
end PouchModule;
