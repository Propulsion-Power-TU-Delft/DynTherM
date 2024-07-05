within DynTherM.Examples.Aircraft.TrimAirControl;
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
        origin={30,50})));
  Modelica.Blocks.Math.Add add_m_rec(k1=+1, k2=-1) annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={56,70})));
  Modelica.Blocks.Sources.RealExpression target_m_rec(y=m_ECS.y/(1/A320.rec_target
         - 1)) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={90,76})));
  Modelica.Blocks.Sources.RealExpression target_P_cab(y=environment.P_cab_target)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-90,76})));
  Systems.Aircraft.AirbusA320 A320(
    redeclare model HTC_int_upper =
        Components.HeatTransfer.HTCorrelations.InternalConvection.Cylinder,
    redeclare model HTC_int_lower =
        Components.HeatTransfer.HTCorrelations.InternalConvection.Cylinder,
    redeclare model HTC_ext_upper =
        Components.HeatTransfer.HTCorrelations.ExternalConvection.AircraftOnGroundFree
        (R_ext=A320.R_fuselage),
    redeclare model HTC_ext_lower =
        Components.HeatTransfer.HTCorrelations.ExternalConvection.AircraftOnGroundFree
        (R_ext=A320.R_fuselage),
    N_pax=196,
    N_crew=6,
    N_pilots=3,
    Q_el=1200,
    Q_galley=1500,
    Q_avionics=10000,
    cabinLights=100,
    inFlightEntertainment=100,
    rec_target=0.5,
    allowFlowReversal=environment.allowFlowReversal,
    initOpt=environment.initOpt)
    annotation (Placement(transformation(extent={{-74,-46},{42,44}})));
  Modelica.Blocks.Sources.Constant T_target(k=300.15) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={90,-90})));
  Modelica.Blocks.Sources.Constant T_trim(k=373.15) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-20,90})));
  Modelica.Blocks.Sources.Constant m_trim_fd(k=0.0) annotation (Placement(
        transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={-90,-90})));
  Modelica.Blocks.Continuous.LimPID PID_T_cab(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.001,
    Ti=5,
    yMax=1,
    yMin=0,
    initType=Modelica.Blocks.Types.Init.SteadyState) annotation (Placement(
        transformation(
        extent={{10,10},{-10,-10}},
        rotation=-90,
        origin={-4,-50})));
  Modelica.Blocks.Continuous.LimPID PID_T_fd(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=1,
    Ti=5,
    yMax=313.15,
    yMin=233.15,
    initType=Modelica.Blocks.Types.Init.SteadyState) annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={70,6})));
  Modelica.Blocks.Math.Add add_P_cab(k1=+1, k2=-1) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-56,70})));
  Modelica.Blocks.Continuous.PID PID_P_cab(
    k=-5e-3,
    Ti=5,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    y_start=0.8) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-28,50})));
  inner Components.Environment environment(
    ISA_plus=16,
    phi_amb=0.6,
    phi_amb_ground=0.6,
    T_ground(displayUnit="degC") = 298.15,
    use_ext_sw=true,
    allowFlowReversal=false,
    initOpt=DynTherM.Choices.InitOpt.steadyState)
    annotation (Placement(transformation(extent={{-82,-64},{-42,-24}})));
equation
  m_fresh_min = 0.25/60*(A320.N_pax + A320.N_crew + A320.N_pilots);
  connect(add_m_rec.y, PID_m_rec.u)
    annotation (Line(points={{45,70},{30,70},{30,62}}, color={0,0,127}));
  connect(m_ECS.y, A320.m_ECS) annotation (Line(points={{9,90},{4.71429,90},{
          4.71429,25.25}},
                       color={0,0,127}));
  connect(A320.T_trim, T_trim.y) annotation (Line(points={{-3.57143,25.25},{
          -4,25.25},{-4,90},{-9,90}},
                        color={0,0,127}));
  connect(PID_m_rec.y, A320.fanSpeed) annotation (Line(points={{30,39},{30,
          25.125},{29.5714,25.125},{29.5714,25.25}},
                                        color={0,0,127}));
  connect(m_trim_fd.y, A320.m_trim_cockpit) annotation (Line(points={{-79,-90},
          {30,-90},{30,-18},{29.5714,-18},{29.5714,-17.5}},
                                                color={0,0,127}));
  connect(add_m_rec.u1, target_m_rec.y)
    annotation (Line(points={{68,76},{79,76}}, color={0,0,127}));
  connect(A320.recirculatedMassflow, add_m_rec.u2) annotation (Line(points={{37.8571,
          25.25},{37.8571,26},{80,26},{80,64},{68,64}},         color={0,0,
          127}));
  connect(PID_T_cab.y, A320.m_trim_cabin) annotation (Line(points={{-4,-39},{
          -4,-17.5},{-3.57143,-17.5}}, color={0,0,127}));
  connect(A320.cabinTemperature, PID_T_cab.u_m) annotation (Line(points={{
          4.71429,-17.5},{18,-17.5},{18,-50},{8,-50}}, color={0,0,127}));
  connect(PID_T_cab.u_s, T_target.y) annotation (Line(points={{-4,-62},{-4,
          -72},{90,-72},{90,-79}}, color={0,0,127}));
  connect(A320.T_ECS, PID_T_fd.y) annotation (Line(points={{37.8571,6.5},{
          40.4286,6.5},{40.4286,6},{59,6}}, color={0,0,127}));
  connect(A320.cockpitTemperature, PID_T_fd.u_m) annotation (Line(points={{37.8571,
          -17.5},{37.8571,-30},{70,-30},{70,-6}},         color={0,0,127}));
  connect(T_target.y, PID_T_fd.u_s)
    annotation (Line(points={{90,-79},{90,6},{82,6}}, color={0,0,127}));
  connect(A320.outflowValveOpening, PID_P_cab.y) annotation (Line(points={{
          -28.4286,25.25},{-28.4286,26.125},{-28,26.125},{-28,39}}, color={0,
          0,127}));
  connect(add_P_cab.y, PID_P_cab.u)
    annotation (Line(points={{-45,70},{-28,70},{-28,62}}, color={0,0,127}));
  connect(target_P_cab.y, add_P_cab.u1)
    annotation (Line(points={{-79,76},{-68,76}}, color={0,0,127}));
  connect(A320.cabinPressure, add_P_cab.u2) annotation (Line(points={{-36.7143,
          25.25},{-80,25.25},{-80,64},{-68,64}},          color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=100, Interval=0.1));
end A320GroundCockpit;
