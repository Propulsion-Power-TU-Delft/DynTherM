within DynTherM.Tests.Battery;
model PolestarBatteryValidation "Validation of Polestar battery module"

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

  Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (
      Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=0,
        origin={0,60})));
  Modelica.Blocks.Sources.TimeTable I_charging(table=[0,600; 200,600; 200,600; 500,
        600; 501,360; 700,360; 701,300; 900,300; 901,360; 1000,360; 1001,210; 1200,
        210])
             annotation (Placement(transformation(extent={{80,76},{64,92}})));
  Systems.Battery.PouchModuleParallel pouchModuleParallel(
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
    annotation (Placement(transformation(extent={{-32,-16},{40,56}})));
  Modelica.Electrical.Analog.Basic.Ground ground
    annotation (Placement(transformation(extent={{52,-12},{68,4}})));
  Modelica.Blocks.Math.Gain gain(k=-1)
    annotation (Placement(transformation(extent={{42,78},{30,90}})));
  BoundaryConditions.thermal_distributed thermal_distributed(
    Nx=Ns*Np,
    Ny=1,
    T=(11 + 273.15)*ones(Ns*Np, 1),
    use_di_Q=false,
    use_di_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-38,-34},{38,-66}})));

  BoundaryConditions.thermal_distributed thermal_distributed1(
    Nx=Ns*Np,
    Ny=1,
    T=(11 + 273.15)*ones(Ns*Np, 1),
    use_di_Q=false,
    use_di_T=true,
    use_in_T=false)
    annotation (Placement(transformation(extent={{-116,46},{-40,14}})));
equation
  connect(pouchModuleParallel.p, signalCurrent.p) annotation (Line(points={{-17.6,
          12.8},{-30,12.8},{-30,60},{-8,60}}, color={0,0,255}));
  connect(pouchModuleParallel.n, signalCurrent.n) annotation (Line(points={{11.2,
          12.8},{11.2,12},{32,12},{32,60},{8,60}}, color={0,0,255}));
  connect(pouchModuleParallel.n, ground.p)
    annotation (Line(points={{11.2,12.8},{60,12.8},{60,4}}, color={0,0,255}));
  connect(gain.u, I_charging.y)
    annotation (Line(points={{43.2,84},{63.2,84}}, color={0,0,127}));
  connect(gain.y, signalCurrent.i)
    annotation (Line(points={{29.4,84},{0,84},{0,69.6}}, color={0,0,127}));
  connect(thermal_distributed.thermal, pouchModuleParallel.Bottom) annotation (
      Line(points={{0,-50},{0,-20},{-3.2,-20},{-3.2,-3.76}}, color={191,0,0}));
  connect(thermal_distributed1.thermal, pouchModuleParallel.Top) annotation (
      Line(points={{-78,30},{-78,48},{-4,48},{-4,29.36},{-3.2,29.36}},
                                                       color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Polygon(
          origin={24,16},
          lineColor={78,138,73},
          fillColor={78,138,73},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}),
                                                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>The test validates the electro-thermal model of the battery module, without the implementation of the cooling system. Instead of modelling the cooling system, a constant surface temperature is applied to the side where cooling plate is going to be attached. </p>
<p>It can be seen that the Moduel behaves thermally the same as in test &quot;PolestarValidationWithCooling&quot; if instead of cooling plate, a constant surface temperature of 30<sup><span style=\"font-size: 6pt;\">0</span></sup>C is applied.</p>
</html>"));
end PolestarBatteryValidation;
