within DynTherM.Tests.Battery;
model PolestarValidationWith2DCooling
  "Validation of Polestar battery module with its cooling system"

  package Coolant = DynTherM.Media.IncompressibleTableBased.MEG_Polestar;

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
  parameter Length t_resin = 0.0025 "Thickness of resin between cells and frame";
  parameter Length t_frame = 0.005 "Frame thickness";

  // Cold Plate
  parameter Integer N_cv_channels = 2 "Number of control Volumes for each channel in the cold plate";
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
  Systems.Battery.ColdPlatePolestar2D coldPlatePolestar(
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
    N_cv=N_cv_channels,
    Nt=3,
    N_channels=1)
    annotation (Placement(transformation(extent={{-74,-132},{102,-6}})));
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
  Components.TwoDimensional.WallConductionDiscretized ThermalInterface(
    redeclare model Mat = DynTherM.Materials.AluminiumColdPlate,
    t=0.092,
    A=W_cell*(t_cell + t_fw/2)*Ns*Np,
    Tstart=298.15,
    Nx=Ns*Np,
    Ny=1)
    "Thermal Interface material between cooling plate and frame which adds thermal resistance"
    annotation (Placement(transformation(extent={{-26,-24},{4,-2}})));
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
          {-60,-60},{-50,-60},{-50,-63.96},{-42.5714,-63.96}},
        color={0,0,0}));
  connect(pressure_sink1.inlet, coldPlatePolestar.outlet) annotation (Line(
        points={{-66,-82},{-52,-82},{-52,-72.78},{-42.5714,-72.78}},
        color={0,0,0}));
  connect(pouchModuleParallel.Bottom, ThermalInterface.inlet)  annotation (Line(
        points={{-7.2,6.24},{-10,6.24},{-10,-9.7},{-11,-9.7}}, color={191,0,0}));
  for i in 1:6 loop
    for j in 1:N_cv_channels loop
        connect(ThermalInterface.outlet.ports[2*i,1], coldPlatePolestar.Top.ports[N_cv_channels,i]);
        connect(ThermalInterface.outlet.ports[(2*i)-1,1], coldPlatePolestar.Top.ports[N_cv_channels,i]);
    end for;
  end for  annotation (Line(
        points={{-11,-16.3},{8,-16.3},{8,-38},{12.7429,-38},{12.7429,-47.58}},
        color={191,0,0}));
 annotation (Line(
        points={{-11,-16.3},{8,-16.3},{8,-38},{12.7429,-38},{12.7429,-47.58}},
        color={191,0,0}),
              Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor = {0,0,255},
                fillColor = {75,138,73},
                pattern = LinePattern.None,
                fillPattern = FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(
        coordinateSystem(preserveAspectRatio=false), graphics={Line(points={{-10,
              -16},{-10,-26},{12,-26},{12,-44}}, color={238,46,47})}),
    experiment(
      StopTime=1200,
      Interval=1,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>Reproducing the fast charging simulation results for a battery module for the work [1], using the same inputs and the parameters. This also takes into account the temperature variation in the moduel</p>
<h4>Comments:</h4>
<ul>
<li>The battery temperature results (Average temperature, minnimum temperature and maximum temperature) are in quite good agreement with the work [1].</li>
<li>The thickness of the resin and frame are not known. So, proceeded with good guesses for their dimensions. Its worth noting that the thickness of the resin will have more effect on the temperature distribution of the cells becuase of its lower thermal conductivity.</li>
</ul>
<h4>References</h4>
<p>[1] I. Gul. &quot;Time efficient simulations for advanced battery cooling concepts&quot;, M.Sc. thesis, Polestar / Universitat Politecnica de Catalunya, 2023.</p>
</html>"));
end PolestarValidationWith2DCooling;
