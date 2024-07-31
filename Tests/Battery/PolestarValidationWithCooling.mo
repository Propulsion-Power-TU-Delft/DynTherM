within DynTherM.Tests.Battery;
model PolestarValidationWithCooling
  "Validation of Polestar battery module with its cooling system"

package Coolant = DynTherM.Media.IncompressibleTableBased.MEG_Polestar(X=0.1);
// package Coolant = Modelica.Media.Water.ConstantPropertyLiquidWater;

// Cell Properties
  parameter Length W_cell = 0.35 "Cell width" annotation (Dialog(tab="Geometry"));
  parameter Length H_cell = 0.1 "Cell height" annotation (Dialog(tab="Geometry"));
  parameter Length t_cell = 0.01 "Cell thickness" annotation (Dialog(tab="Geometry"));
  parameter ElectricCharge C_nom = 66.4*3600 "Nominal cell capacity";

// Module
  parameter Integer N_cv = 10 "Number of vertical control volumes in which each cell is discretized";
  parameter Integer Ns = 4 "Number of cells connected in series";
  parameter Integer Np = 3 "Number of cells connected in parallel";
  parameter Length t_fw = 0.002 "Thickness of firewall between cells in parallel";
  parameter Length t_resin = 0.0001 "Thickness of resin between cells and frame";
  parameter Length t_frame = 0.005 "Frame thickness";

// Cold Plate
  parameter Length L = W_cell "Length of the channel" annotation (Dialog(tab="Geometry"));
  parameter Length t = 2*R_int + 0.002 "Thickness of the cold Plate" annotation (Dialog(tab="Geometry"));
  parameter Length d = 0.01 "Center to center distance between the Channels" annotation (Dialog(tab="Geometry"));
  parameter Length R_int = 0.0025  "Channel internal radius" annotation (Dialog(tab="Geometry"));

    parameter Temperature T_fluid=298.15;
    parameter MassFlowRate m_flow=0.04416;

// Initialization

  Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (
      Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=0,
        origin={-2,68})));
  Modelica.Blocks.Sources.TimeTable I_charging(table=[0,600; 200,600; 200,600; 500,
        600; 501,360; 700,360; 701,300; 900,300; 901,360; 1000,360; 1001,210; 1200,
        210])
             annotation (Placement(transformation(extent={{-94,60},{-82,72}})));
  Systems.Battery.PouchModuleParallel pouchModuleParallel(
    redeclare model InPlaneCellMat = DynTherM.Materials.PolestarCellInPlane,
    redeclare model CrossPlaneCellMat =
        DynTherM.Materials.PolestarCellCrossPlane,
    redeclare model FirewallMat = DynTherM.Materials.CompressionPadFoam,
    redeclare model ResinMat = DynTherM.Materials.ThermalResin,
    redeclare model FrameMat = DynTherM.Materials.AluminiumColdPlate,
    W_cell=W_cell,
    H_cell=H_cell,
    t_cell=t_cell,
    t_fw=t_fw,
    t_resin=t_resin,
    t_frame=t_frame,                                      C_nom(displayUnit="Ah")
       = C_nom,
    SoC_start=0.1,
    N_cv=N_cv,
    Ns=Ns,
    Np=Np)
    annotation (Placement(transformation(extent={{-36,-6},{36,66}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{52,-12},{68,4}})));
  Modelica.Blocks.Math.Gain gain(k=-1)
    annotation (Placement(transformation(extent={{-40,82},{-28,94}})));
  Systems.Battery.ColdPlatePolestar coldPlatePolestar(
    redeclare model Mat = DynTherM.Materials.AluminiumColdPlate,
    redeclare package Medium = Coolant,
    allowFlowReversal=true,
    DP_opt=DynTherM.Choices.PDropOpt.correlation,
    L=L,
    t=t,
    d=d,
    R_int=R_int,
    T_start_solid(displayUnit="degC") = 298.15,
    T_start_fluid=298.15,
    m_flow_start=0.04416,
    rho_start(displayUnit="kg/m3") = 1e3,
    u_start=2,
    dP_start=200000000,
    Re_start=3e3,
    Pr_start=25,
    N_cv=1,
    Nt=3,
    N_channels=1)
    annotation (Placement(transformation(extent={{-74,-134},{102,-8}})));
    BoundaryConditions.flow_source          flow_source1(
    redeclare package Medium = Coolant,
    T_nom=T_fluid,
    massFlow_nom=m_flow,
    allowFlowReversal=environment.allowFlowReversal,
    use_in_massFlow=false,
    use_in_T=false)
      annotation (Placement(transformation(extent={{-78,-50},{-60,-68}})));
    BoundaryConditions.pressure_sink          pressure_sink1(
    redeclare package Medium = Coolant,
    allowFlowReversal=environment.allowFlowReversal,
    use_ambient=false)
      annotation (Placement(transformation(extent={{-66,-88},{-78,-76}})));
  Components.HeatTransfer.WallConduction wallConduction(
    redeclare model Mat = DynTherM.Materials.AluminiumColdPlate,
    t=0.092,
    A=W_cell*(t_cell + t_fw)*Ns*Np)
    annotation (Placement(transformation(extent={{-12,-13},{12,13}},
        rotation=90,
        origin={-33,-26})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier(Nx=12, Ny=1)
    annotation (Placement(transformation(extent={{-13,-4},{13,4}},
        rotation=90,
        origin={-55,-26})));
  CustomInterfaces.Adaptors.heatFlowMultiplier heatFlowMultiplier1(Nx=1,  Ny=1)
    annotation (Placement(transformation(extent={{-13,4},{13,-4}},
        rotation=90,
        origin={-13,-26})));
equation
  connect(pouchModuleParallel.p, signalCurrent.p) annotation (Line(points={{-21.6,
          22.8},{-40,22.8},{-40,68},{-10,68}},color={0,0,255}));
  connect(pouchModuleParallel.n, signalCurrent.n) annotation (Line(points={{7.2,
          22.8},{40,22.8},{40,68},{6,68}},         color={0,0,255}));
  connect(pouchModuleParallel.n, ground.p)
    annotation (Line(points={{7.2,22.8},{60,22.8},{60,4}},  color={0,0,255}));
  connect(gain.u, I_charging.y)
    annotation (Line(points={{-41.2,88},{-70,88},{-70,66},{-81.4,66}},
                                                   color={0,0,127}));
  connect(gain.y, signalCurrent.i)
    annotation (Line(points={{-27.4,88},{-2,88},{-2,77.6}},
                                                         color={0,0,127}));
  connect(flow_source1.outlet, coldPlatePolestar.inlet) annotation (Line(points={{-60,-59},
          {-60,-60},{-50,-60},{-50,-65.96},{-42.5714,-65.96}},
        color={0,0,0}));
  connect(pressure_sink1.inlet, coldPlatePolestar.outlet) annotation (Line(
        points={{-66,-82},{-52,-82},{-52,-74.78},{-42.5714,-74.78}},
        color={0,0,0}));
  connect(heatFlowMultiplier.single, wallConduction.inlet)
    annotation (Line(points={{-52.6,-26},{-37.42,-26}},      color={191,0,0}));
  connect(heatFlowMultiplier.distributed, pouchModuleParallel.Bottom)
    annotation (Line(points={{-57.4,-26},{-74,-26},{-74,-4},{-8,-4},{-8,6.24},{-7.2,
          6.24}},                                                  color={191,0,
          0}));
  connect(wallConduction.outlet, heatFlowMultiplier1.single)
    annotation (Line(points={{-28.58,-26},{-15.4,-26}}, color={191,0,0}));
  connect(coldPlatePolestar.Top, heatFlowMultiplier1.distributed) annotation (
      Line(points={{12.7429,-49.58},{12.7429,-25.79},{-10.6,-25.79},{-10.6,
          -26}},
        color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end PolestarValidationWithCooling;
