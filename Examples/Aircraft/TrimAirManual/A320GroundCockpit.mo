within DynTherM.Examples.Aircraft.TrimAirManual;
model A320GroundCockpit "Aircraft on ground, two temperature zones"
  // Pack discharge temperature determined by cockpit cooling requirement
  // Trim air flow determined by cabin temperature set point

  Modelica.Units.SI.MassFlowRate m_fresh_min
    "Minimum mass flow rate of fresh air that must be provided by the ECS - fixed by standard CFR 25.831";
  Modelica.Blocks.Sources.Constant m_ECS(k=1.0)
    annotation (Placement(transformation(extent={{30,80},{10,100}})));
  Modelica.Blocks.Continuous.PID PID_m_rec(
    k=500,
    Ti=5,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    y_start=180) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={46,56})));
  Modelica.Blocks.Math.Add add_m_rec(k1=-1, k2=+1) annotation (Placement(
        transformation(
        extent={{10,10},{-10,-10}},
        rotation=0,
        origin={60,84})));
  Modelica.Blocks.Continuous.PID PID_P_cab(
    k=-5e-3,
    Ti=5,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    y_start=0.8) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-48,56})));
  Modelica.Blocks.Math.Add add_P_cab(k1=+1, k2=-1) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-62,84})));
  Modelica.Blocks.Sources.RealExpression target_m_rec(y=m_ECS.y/(1/A320.rec_target
         - 1)) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={90,90})));
  Modelica.Blocks.Sources.RealExpression target_P_cab(y=environment.P_cab_target)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-90,90})));
  Modelica.Blocks.Sources.Constant T_target(k=300.15) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={90,-90})));
  Modelica.Blocks.Math.Add add_T(k1=+1, k2=-1) annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={84,-30})));
  Modelica.Blocks.Continuous.PID PID_T(
    k=1,
    Ti=5,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    y_start=0.8) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={66,4})));
  Systems.Aircraft.AirbusA320 A320(
    redeclare model HTC_int_upper =
        Components.HeatTransfer.HTCorrelations.InternalConvection.Cylinder,
    redeclare model HTC_int_lower =
        Components.HeatTransfer.HTCorrelations.InternalConvection.Cylinder,
    redeclare model HTC_ext_upper =
        Components.HeatTransfer.HTCorrelations.ExternalConvection.AircraftOnGroundFree
        (
        Nx=1,
        Ny=1,
        R_ext=A320.R_fuselage),
    redeclare model HTC_ext_lower =
        Components.HeatTransfer.HTCorrelations.ExternalConvection.AircraftOnGroundFree
        (
        Nx=1,
        Ny=1,
        R_ext=A320.R_fuselage),
    N_pax=196,
    N_crew=6,
    N_pilots=3,
    Q_el=1200,
    Q_galley=1500,
    Q_avionics=10000,
    cabinLights=100,
    inFlightEntertainment=100,
    rec_target=0.45,
    theta_1(displayUnit="rad"),
    theta_2(displayUnit="rad"),
    theta_3(displayUnit="rad"),
    theta_4(displayUnit="rad"),
    theta_5(displayUnit="rad"),
    theta_6(displayUnit="rad"),
    theta_7(displayUnit="rad"),
    theta_8(displayUnit="rad"),
    theta_front(displayUnit="rad"))
    annotation (Placement(transformation(extent={{-72,-48},{44,42}})));
  Modelica.Blocks.Sources.Constant T_trim(k=373.15) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-20,90})));
  Modelica.Blocks.Sources.Constant m_trim_fd(k=0.0) annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={50,-90})));
  Modelica.Blocks.Sources.Constant m_trim_cab(k=0.09) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,-90})));
  inner Components.Environment environment(
    ISA_plus=0,
    phi_amb=0.0,
    phi_amb_ground=0.22,
    T_ground(displayUnit="degC") = 298.15,
    use_ext_sw=true,
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.steadyState)
    annotation (Placement(transformation(extent={{-82,-64},{-42,-24}})));
equation
  m_fresh_min = 0.25/60*(A320.N_pax + A320.N_crew + A320.N_pilots);
  connect(target_P_cab.y, add_P_cab.u1)
    annotation (Line(points={{-79,90},{-74,90}}, color={0,0,127}));
  connect(add_P_cab.y, PID_P_cab.u)
    annotation (Line(points={{-51,84},{-48,84},{-48,68}}, color={0,0,127}));
  connect(add_m_rec.y, PID_m_rec.u)
    annotation (Line(points={{49,84},{46,84},{46,68}}, color={0,0,127}));
  connect(add_m_rec.u2, target_m_rec.y)
    annotation (Line(points={{72,90},{79,90}}, color={0,0,127}));
  connect(add_T.u1, T_target.y)
    annotation (Line(points={{90,-42},{90,-79}}, color={0,0,127}));
  connect(add_T.y, PID_T.u)
    annotation (Line(points={{84,-19},{84,4},{78,4}}, color={0,0,127}));
  connect(A320.cabinPressure, add_P_cab.u2) annotation (Line(points={{-34.7143,
          23.25},{-34.7143,34},{-80,34},{-80,78},{-74,78}},
                                               color={0,0,127}));
  connect(A320.recirculatedMassflow, add_m_rec.u1) annotation (Line(points={{39.8571,
          23.25},{39.8571,34},{80,34},{80,78},{72,78}},
                                                 color={0,0,127}));
  connect(m_ECS.y, A320.m_ECS) annotation (Line(points={{9,90},{6.71429,90},{
          6.71429,23.25}},
                       color={0,0,127}));
  connect(A320.T_trim, T_trim.y) annotation (Line(points={{-1.57143,23.25},{-4,
          23.25},{-4,90},{-9,90}},
                        color={0,0,127}));
  connect(A320.T_ECS, PID_T.y) annotation (Line(points={{39.8571,4.5},{39.8571,
          4},{55,4}}, color={0,0,127}));
  connect(A320.cockpitTemperature, add_T.u2) annotation (Line(points={{39.8571,
          -19.5},{39.8571,-50},{78,-50},{78,-42}},         color={0,0,127}));
  connect(PID_m_rec.y, A320.fanSpeed) annotation (Line(points={{46,45},{46,42},
          {31.5714,42},{31.5714,23.25}},color={0,0,127}));
  connect(PID_P_cab.y, A320.outflowValveOpening) annotation (Line(points={{-48,45},
          {-48,42},{-26.4286,42},{-26.4286,23.25}},        color={0,0,127}));
  connect(m_trim_cab.y, A320.m_trim_cabin) annotation (Line(points={{-19,-90},{
          -1.57143,-90},{-1.57143,-19.5}},  color={0,0,127}));
  connect(m_trim_fd.y, A320.m_trim_cockpit) annotation (Line(points={{39,-90},{
          32,-90},{32,-19.5},{31.5714,-19.5}},  color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=100, Interval=0.1));
end A320GroundCockpit;
