within DynTherM.Systems;
package E9X "Models for Elysian E9X battery-powered aircraft"
  package IceProtectionSystem "Ice protection system"
    record IPS

      extends Modelica.Icons.Record;

      // F9X data
      parameter DynTherM.Choices.WingSection section=DynTherM.Choices.WingSection.Root "Wing section under analysis";
      parameter Length c_root=5.02 "Chord length at root" annotation (Dialog(tab="Aircraft data"));
      parameter Length c_mean=3.73 "Chord length at mean" annotation (Dialog(tab="Aircraft data"));
      parameter Length c_tip=2.01 "Chord length at tip" annotation (Dialog(tab="Aircraft data"));
      parameter Length t_root=0.85 "Airfoil thickness at root" annotation (Dialog(tab="Aircraft data"));
      parameter Length t_mean=0.51 "Airfoil thickness at mean" annotation (Dialog(tab="Aircraft data"));
      parameter Length t_tip=0.18 "Airfoil thickness at tip" annotation (Dialog(tab="Aircraft data"));
      parameter Length t_skin=1e-3 "Wing skin thickness" annotation (Dialog(tab="Aircraft data"));

      // Operating conditions
      parameter MassFlowRate m_flow_root=2.0 "Total coolant mass flow rate at root section" annotation (Dialog(tab="Operating conditions"));
      parameter MassFlowRate m_flow_mean=1.8 "Total coolant mass flow rate at mean section" annotation (Dialog(tab="Operating conditions"));
      parameter MassFlowRate m_flow_tip=1.6 "Total coolant mass flow rate at tip section" annotation (Dialog(tab="Operating conditions"));
      parameter Temperature T_in=333.15 "Coolant inlet temperature" annotation (Dialog(tab="Operating conditions"));
      parameter Pressure P_in=1e5 "Coolant inlet pressure" annotation (Dialog(tab="Operating conditions"));

      // IPS data
      parameter Length L=5 "Length of one IPS section" annotation (Dialog(tab="IPS data"));
      parameter Real xc_front_spar_root=0.2
        "Normalized (over chord) front spar distance from leading edge at root" annotation (Dialog(tab="IPS data"));
      parameter Real xc_front_spar_mean=0.25
        "Normalized (over chord) front spar distance from leading edge at mean" annotation (Dialog(tab="IPS data"));
      parameter Real xc_front_spar_tip=0.3
        "Normalized (over chord) front spar distance from leading edge at tip" annotation (Dialog(tab="IPS data"));
      parameter Length W=10e-3 "Width of the minichannels" annotation (Dialog(tab="IPS data"));
      parameter Length H=1e-3 "Height of the minichannels" annotation (Dialog(tab="IPS data"));
      parameter Length t_ext=0.5e-3 "Thickness of external minichannels walls" annotation (Dialog(tab="IPS data"));
      parameter Length t_int=0.25e-3 "Thickness of internal minichannels walls" annotation (Dialog(tab="IPS data"));

      // Simulation options
      parameter Integer N_cv=10
        "Number of longitudinal control volumes used to discretize each minichannel";
      parameter Temperature T_start_solid=T_in
        "Temperature of solid part - start value" annotation (Dialog(tab="Initialization"));
      parameter Temperature T_start_fluid=T_in
        "Temperature of fluid part - start value" annotation (Dialog(tab="Initialization"));
      parameter Pressure P_start=1e5
        "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
      parameter MassFlowRate m_flow_start=1
        "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));

      Mass m_mc "Mass of minichannels";
      MassFlowRate m_flow_mc "Coolant mass flow rate per minichannel";
      HeatFlowRate Q_mc "Heat flow rate exchanged by the minichannels";

      // Geometry
      Length R_le_root "Airfoil leading edge radius at root";
      Length R_le_mean "Airfoil leading edge radius at mean";
      Length R_le_tip "Airfoil leading edge radius at tip";
      Length x_front_spar_root "Front spar distance from leading edge at root";
      Length x_front_spar_mean "Front spar distance from leading edge at mean";
      Length x_front_spar_tip "Front spar distance from leading edge at tip";
      Length W_IPS_root "Width of IPS unrolled surface at root";
      Length W_IPS_mean "Width of IPS unrolled surface at mean";
      Length W_IPS_tip "Width of IPS unrolled surface at tip";
      Real N_root "Number of minichannels at root";
      Real N_mean "Number of minichannels at mean";
      Real N_tip "Number of minichannels at tip";

      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end IPS;

    model WingSkinHEX

      package Coolant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.5); //Modelica.Media.Water.StandardWater; // "Coolant";
      outer DynTherM.Components.Environment environment "Environmental properties";

      Real N_upper "Number of minichannels located in the upper section";
      Real N_frontal "Number of minichannels located in the frontal section";
      Real N_lower "Number of minichannels located in the lower section";

      IPS F9X(
        section=DynTherM.Choices.WingSection.Root,
        m_flow_mean=1.6,
        m_flow_tip=1,
        T_in=343.15,
        P_in=200000,
        N_cv=3,
        T_start_solid=323.15,
        T_start_fluid=333.15,
        P_start=300000,
        m_flow_start=0.01)
        annotation (Placement(transformation(extent={{-150,
                112},{-130,132}})));
      Components.OneDimensional.RectangularChannels1D minichannel_upper(
        redeclare model Mat = Materials.Aluminium,
        redeclare package Medium = Coolant,
        L=F9X.L,
        W=F9X.W,
        H=F9X.H,
        t_north=F9X.t_ext + F9X.t_skin,
        t_east=F9X.t_int,
        t_south=F9X.t_ext,
        t_west=F9X.t_int,
        T_start_solid=F9X.T_start_solid,
        T_start_fluid=F9X.T_start_fluid,
        P_start=F9X.P_start,
        X_start=X_start,
        state_start=state_start,
        m_flow_start=F9X.m_flow_start,
        N=F9X.N_cv,
        N_channels=1)
        annotation (Placement(transformation(extent={{22,52},{78,108}})));
      Components.OneDimensional.ExternalConvection1D convection_upper(redeclare
          model HTC =
            Components.HeatTransfer.HTCorrelations.ExternalConvection.WingTakeOff
            ( c=c), N=F9X.N_cv)
        annotation (Placement(transformation(extent={{-16,136},{16,106}})));
      Components.HeatTransfer.SolarRadiation solar_radiation_upper(
        E_tb_fixed=973.43,
        E_td_fixed=85.7,
        theta_fixed(displayUnit="deg") = 0.13055062804918,
        csi=0)
        annotation (Placement(transformation(
            extent={{-18,18},{18,-18}},
            rotation=-90,
            origin={108,120})));
      Components.Adaptors.irradianceMultiplier multiplier_upper(Nx=F9X.N_cv, Ny=1)
        annotation (Placement(transformation(
            extent={{-12,12},{12,-12}},
            rotation=-90,
            origin={70,120})));
      Components.OneDimensional.WallRadiation1D radiation_upper(
        redeclare model Material =
            Materials.Paints.ConductivePaints.BrilliantAluminumPaint,
        csi=solar_radiation_upper.csi,
        N=F9X.N_cv) annotation (Placement(transformation(
            extent={{-16,15},{16,-15}},
            rotation=-90,
            origin={34,121})));
      Components.MassTransfer.PlenumSimple plenum(
        redeclare package Medium = Coolant,
        V=1e-4,
        m_flow_start=F9X.m_flow_start*3,
        P_start=F9X.P_start,
        T_start=F9X.T_start_fluid,
        X_start=X_start,
        state_start=state_start,
        noInitialPressure=true)
        annotation (Placement(transformation(extent={{-122,-12},{-98,12}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow EPU annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-110,110})));
      Modelica.Blocks.Interfaces.RealInput Q_input annotation (Placement(
            transformation(
            extent={{-14,-14},{14,14}},
            rotation=-90,
            origin={-110,146}), iconTransformation(
            extent={{-14,-14},{14,14}},
            rotation=-90,
            origin={-110,146})));
      Components.MassTransfer.Fan pump(
        redeclare package Medium = Coolant,
        eta_is=0.7,
        eta_m=0.99,
        omega_nom(displayUnit="rad/s") = 314.15926535898,
        volFlow_nom=3e-5,
        Head_nom=75,
        rho_start(displayUnit="kg/m3") = 1000,
        redeclare
          DynTherM.Components.MassTransfer.FanCharacteristics.FlowCharacteristics.linearFlow
          flowModel)
        annotation (Placement(transformation(extent={{-60,-14},{-32,14}})));
      BoundaryConditions.mechanical mechanical(
        omega(displayUnit="rpm") = 314.15926535898,
        use_omega=false,
        use_in_omega=true)
        annotation (Placement(transformation(extent={{9,-6},{-9,6}},
            rotation=180,
            origin={-49,28})));
      Components.OneDimensional.RectangularChannels1D minichannel_frontal(
        redeclare model Mat = Materials.Aluminium,
        redeclare package Medium = Coolant,
        L=F9X.L,
        W=F9X.W,
        H=F9X.H,
        t_north=F9X.t_ext + F9X.t_skin,
        t_east=F9X.t_int,
        t_south=F9X.t_ext,
        t_west=F9X.t_int,
        T_start_solid=F9X.T_start_solid,
        T_start_fluid=F9X.T_start_fluid,
        P_start=F9X.P_start,
        X_start=X_start,
        state_start=state_start,
        m_flow_start=F9X.m_flow_start,
        N=F9X.N_cv,
        N_channels=1)
        annotation (Placement(transformation(extent={{22,-28},{78,28}})));
      Components.OneDimensional.ExternalConvection1D convection_frontal(redeclare
          model HTC =
            Components.HeatTransfer.HTCorrelations.ExternalConvection.WingTakeOff
            ( c=c), N=F9X.N_cv)
        annotation (Placement(transformation(extent={{-16,56},{16,26}})));
      Components.HeatTransfer.SolarRadiation solar_radiation_frontal(
        E_tb_fixed=48.11,
        E_td_fixed=225.27,
        E_tr_fixed=182.71,
        theta_fixed(displayUnit="deg") = 1.5217525748139,
        csi=1.5707963267949)
        annotation (Placement(transformation(
            extent={{-18,18},{18,-18}},
            rotation=-90,
            origin={108,40})));
      Components.Adaptors.irradianceMultiplier multiplier_frontal(Nx=F9X.N_cv, Ny=1)
        annotation (Placement(transformation(
            extent={{-12,12},{12,-12}},
            rotation=-90,
            origin={70,40})));
      Components.OneDimensional.WallRadiation1D radiation_frontal(
        redeclare model Material =
            Materials.Paints.ConductivePaints.BrilliantAluminumPaint,
        csi=solar_radiation_frontal.csi,
        N=F9X.N_cv) annotation (Placement(transformation(
            extent={{-16,15},{16,-15}},
            rotation=-90,
            origin={34,41})));
      Components.OneDimensional.RectangularChannels1D minichannel_lower(
        redeclare model Mat = Materials.Aluminium,
        redeclare package Medium = Coolant,
        L=F9X.L,
        W=F9X.W,
        H=F9X.H,
        t_north=F9X.t_ext + F9X.t_skin,
        t_east=F9X.t_int,
        t_south=F9X.t_ext,
        t_west=F9X.t_int,
        T_start_solid=F9X.T_start_solid,
        T_start_fluid=F9X.T_start_fluid,
        P_start=F9X.P_start,
        X_start=X_start,
        state_start=state_start,
        m_flow_start=F9X.m_flow_start,
        N=F9X.N_cv,
        N_channels=1)
        annotation (Placement(transformation(extent={{22,-108},{78,-52}})));
      Components.OneDimensional.ExternalConvection1D convection_lower(redeclare
          model HTC =
            Components.HeatTransfer.HTCorrelations.ExternalConvection.WingTakeOff
            ( c=c), N=F9X.N_cv)
        annotation (Placement(transformation(extent={{-16,-24},{16,-54}})));
      Components.HeatTransfer.SolarRadiation solar_radiation_lower(
        E_td_fixed=366.76,
        E_tr_fixed=365.43,
        theta_fixed(displayUnit="deg") = 3.0110420255406,
        csi=3.1415926535898)
        annotation (Placement(transformation(
            extent={{-18,18},{18,-18}},
            rotation=-90,
            origin={108,-40})));
      Components.Adaptors.irradianceMultiplier multiplier_lower(Nx=F9X.N_cv, Ny=1)
        annotation (Placement(transformation(
            extent={{-12,12},{12,-12}},
            rotation=-90,
            origin={70,-40})));
      Components.OneDimensional.WallRadiation1D radiation_lower(
        redeclare model Material =
            Materials.Paints.ConductivePaints.BrilliantAluminumPaint,
        csi=solar_radiation_lower.csi,
        N=F9X.N_cv) annotation (Placement(transformation(
            extent={{-16,15},{16,-15}},
            rotation=-90,
            origin={34,-39})));
      Modelica.Blocks.Interfaces.RealInput omega_input annotation (Placement(
            transformation(
            extent={{14,-14},{-14,14}},
            rotation=180,
            origin={-166,34}), iconTransformation(
            extent={{-14,-14},{14,14}},
            rotation=-90,
            origin={120,146})));
      Modelica.Blocks.Interfaces.RealOutput output_mass_flow annotation (Placement(
            transformation(
            extent={{-12,-12},{12,12}},
            rotation=-90,
            origin={-80,-128}), iconTransformation(
            extent={{14,-14},{-14,14}},
            rotation=90,
            origin={0,-118})));
      Sensors.MassflowSensor sensor(redeclare package Medium = Coolant)
        annotation (Placement(transformation(extent={{-90,10},{-70,-10}})));
      BoundaryConditions.pressure_sink pressure_sink(
        redeclare package Medium = Coolant,
        use_ambient=false,
        P_di=F9X.P_in,
        T_di=F9X.T_in)
        annotation (Placement(transformation(extent={{150,-10},{170,10}})));
      Components.OneDimensional.InternalConvection1D internal_convection_upper(
          redeclare model HTC =
            DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection.Cylinder,
          N=F9X.N_cv)
        annotation (Placement(transformation(extent={{80,114},{100,94}})));
      Components.OneDimensional.InternalConvection1D internal_convection_frontal(
          redeclare model HTC =
            Components.HeatTransfer.HTCorrelations.InternalConvection.Cylinder, N=
           F9X.N_cv)
        annotation (Placement(transformation(extent={{80,34},{100,14}})));
      Components.OneDimensional.InternalConvection1D internal_convection_lower(
          redeclare model HTC =
            Components.HeatTransfer.HTCorrelations.InternalConvection.Cylinder, N=
           F9X.N_cv)
        annotation (Placement(transformation(extent={{80,-46},{100,-66}})));
      BoundaryConditions.thermal_flux_distributed internal_bc_upper(
        Nx=F9X.N_cv,
        Ny=1,
        T=environment.T_amb*ones(F9X.N_cv, 1),
        use_phi=false,
        use_T=true)
        annotation (Placement(transformation(extent={{80,108},{100,118}})));
      BoundaryConditions.thermal_flux_distributed internal_bc_frontal(
        Nx=F9X.N_cv,
        Ny=1,
        T=environment.T_amb*ones(F9X.N_cv, 1),
        use_phi=false,
        use_T=true)
        annotation (Placement(transformation(extent={{80,28},{100,38}})));
      BoundaryConditions.thermal_flux_distributed internal_bc_lower(
        Nx=F9X.N_cv,
        Ny=1,
        T=environment.T_amb*ones(F9X.N_cv, 1),
        use_phi=false,
        use_T=true)
        annotation (Placement(transformation(extent={{80,-52},{100,-42}})));
    protected
      parameter Coolant.ThermodynamicState state_start=
        Coolant.setState_pTX(F9X.P_start, F9X.T_start_fluid, X_start)
        "Starting thermodynamic state";
      parameter MassFraction X_start[Coolant.nX]=Coolant.reference_X
        "Mass fractions - start value";

      Length c "Airfoil chord related to the wing section under analysis";
      Real N "Number of minichannels related to the wing section under analysis";
      MassFlowRate m_flow "Mass flow rate related to the wing section under analysis";

    equation

      // Geometry
      F9X.R_le_root = F9X.t_root/2;
      F9X.R_le_mean = F9X.t_mean/2;
      F9X.R_le_tip  = F9X.t_tip/2;

      F9X.x_front_spar_root = F9X.c_root*F9X.xc_front_spar_root;
      F9X.x_front_spar_mean = F9X.c_mean*F9X.xc_front_spar_mean;
      F9X.x_front_spar_tip  = F9X.c_tip*F9X.xc_front_spar_tip;

      F9X.W_IPS_root = (F9X.x_front_spar_root - F9X.R_le_root) + pi*F9X.R_le_root;
      F9X.W_IPS_mean = (F9X.x_front_spar_mean - F9X.R_le_mean) + pi*F9X.R_le_mean;
      F9X.W_IPS_tip  = (F9X.x_front_spar_tip  - F9X.R_le_tip)  + pi*F9X.R_le_tip;

      F9X.N_root = ceil(F9X.W_IPS_root/F9X.W);
      F9X.N_mean = ceil(F9X.W_IPS_mean/F9X.W);
      F9X.N_tip  = ceil(F9X.W_IPS_tip/F9X.W);

      N_upper = ceil(N/3);
      N_lower = ceil(N/3);
      N_frontal = N - N_upper - N_lower;

      F9X.m_flow_mc = m_flow/N;
      F9X.m_mc      = minichannel_upper.m_tot*N_upper +
                      minichannel_frontal.m_tot*N_frontal +
                      minichannel_lower.m_tot*N_lower;
      F9X.Q_mc      = minichannel_upper.Q*N_upper +
                      minichannel_frontal.Q*N_frontal +
                      minichannel_lower.Q*N_lower;

      if F9X.section == DynTherM.Choices.WingSection.Root then
        N      = F9X.N_root;
        c      = F9X.c_root;
        m_flow = F9X.m_flow_root;

      elseif F9X.section == DynTherM.Choices.WingSection.Mean then
        N      = F9X.N_mean;
        c      = F9X.c_mean;
        m_flow = F9X.m_flow_mean;

      elseif F9X.section == DynTherM.Choices.WingSection.Tip then
        N      = F9X.N_tip;
        c      = F9X.c_tip;
        m_flow = F9X.m_flow_tip;

      end if;

    //   m_pump_root =0.453592*(8.5942*(0.0092*(2*m_flow_root/(minichannel.cv[1].fluid.rho
    //      + minichannel.cv[N_cv].fluid.rho)*61023.78)^1.3857) + 2.4229);
    //   m_pump_mean = 0.453592*(8.5942*(0.0092*(2*m_flow_mean/
    //     (minichannel_mean.cv[1].fluid.rho + minichannel_mean.cv[N_cv].fluid.rho)*61023.78)^1.3857) + 2.4229);
    //   m_pump_tip = 0.453592*(8.5942*(0.0092*(2*m_flow_tip/
    //     (minichannel_tip.cv[1].fluid.rho + minichannel_tip.cv[N_cv].fluid.rho)*61023.78)^1.3857) + 2.4229);
    //   m_pump_tot = 2*(m_pump_root + 2*m_pump_mean + m_pump_tip);
    //   m_tot = m_mc_tot + m_pump_tot;

      connect(radiation_upper.outlet, multiplier_upper.distributed) annotation (
          Line(points={{35.5,121},{35.5,120},{62.8,120}},
                                                        color={238,46,47}));
      connect(multiplier_upper.single, solar_radiation_upper.inlet)
        annotation (Line(points={{77.2,120},{96.84,120}}, color={191,0,0}));
      connect(plenum.thermalPort, EPU.port)
        annotation (Line(points={{-110,10.8},{-110,100}}, color={191,0,0}));
      connect(EPU.Q_flow, Q_input)
        annotation (Line(points={{-110,120},{-110,146}}, color={0,0,127}));
      connect(mechanical.mechanical, pump.shaft)
        annotation (Line(points={{-46,28},{-46,14}}, color={135,135,135}));
      connect(radiation_frontal.outlet, multiplier_frontal.distributed)
        annotation (Line(points={{35.5,41},{35.5,40},{62.8,40}},
                                                               color={238,46,47}));
      connect(multiplier_frontal.single, solar_radiation_frontal.inlet)
        annotation (Line(points={{77.2,40},{96.84,40}}, color={191,0,0}));
      connect(radiation_lower.outlet, multiplier_lower.distributed) annotation (
          Line(points={{35.5,-39},{35.5,-40},{62.8,-40}},
                                                        color={238,46,47}));
      connect(multiplier_lower.single, solar_radiation_lower.inlet)
        annotation (Line(points={{77.2,-40},{96.84,-40}}, color={191,0,0}));
      connect(omega_input, mechanical.in_omega)
        annotation (Line(points={{-166,34},{-59.8,34}}, color={0,0,127}));
      connect(plenum.outlet, sensor.inlet)
        annotation (Line(points={{-98,0},{-90,0}}, color={0,0,0}));
      connect(sensor.outlet, pump.inlet) annotation (Line(points={{-70,0},{-67,0},{-67,
              1.77636e-15},{-60,1.77636e-15}}, color={0,0,0}));
      connect(sensor.y, output_mass_flow)
        annotation (Line(points={{-80,-11},{-80,-128}}, color={0,0,127}));
      connect(pump.outlet, minichannel_frontal.inlet) annotation (Line(points={{
              -32,1.77636e-15},{-5,1.77636e-15},{-5,3.55271e-15},{22,3.55271e-15}},
            color={0,0,0}));
      connect(pump.outlet, minichannel_upper.inlet) annotation (Line(points={{-32,
              0},{-20,0},{-20,80},{22,80}}, color={0,0,0}));
      connect(pump.outlet, minichannel_lower.inlet) annotation (Line(points={{-32,
              0},{-20,0},{-20,-80},{22,-80}}, color={0,0,0}));
      connect(minichannel_upper.solid_surface_north, convection_upper.inlet)
        annotation (Line(points={{29,92.88},{0,92.88},{0,116.5}}, color={255,127,
              0}));
      connect(minichannel_frontal.solid_surface_north, convection_frontal.inlet)
        annotation (Line(points={{29,12.88},{0,12.88},{0,36.5}}, color={255,127,0}));
      connect(minichannel_lower.solid_surface_north, convection_lower.inlet)
        annotation (Line(points={{29,-67.12},{0,-67.12},{0,-43.5}}, color={255,
              127,0}));
      connect(minichannel_frontal.solid_surface_north, radiation_frontal.inlet)
        annotation (Line(points={{29,12.88},{29,34.94},{29.5,34.94},{29.5,41}},
            color={255,127,0}));
      connect(minichannel_lower.solid_surface_north, radiation_lower.inlet)
        annotation (Line(points={{29,-67.12},{29,-45.06},{29.5,-45.06},{29.5,-39}},
            color={255,127,0}));
      connect(minichannel_upper.solid_surface_north, radiation_upper.inlet)
        annotation (Line(points={{29,92.88},{28,92.88},{28,118},{29.5,118},{29.5,
              121}}, color={255,127,0}));
      connect(minichannel_frontal.outlet, pressure_sink.inlet) annotation (Line(
            points={{78,3.55271e-15},{114,3.55271e-15},{114,0},{150,0}}, color={0,0,
              0}));
      connect(minichannel_upper.outlet, pressure_sink.inlet)
        annotation (Line(points={{78,80},{150,80},{150,0}}, color={0,0,0}));
      connect(minichannel_lower.outlet, pressure_sink.inlet)
        annotation (Line(points={{78,-80},{150,-80},{150,0}}, color={0,0,0}));
      connect(pressure_sink.inlet, plenum.inlet) annotation (Line(points={{150,0},{150,
              -100},{-140,-100},{-140,0},{-122,0}}, color={0,0,0}));
      connect(minichannel_upper.solid_surface_south, internal_convection_upper.inlet)
        annotation (Line(points={{57,92.88},{57,101},{90,101}}, color={255,127,0}));
      connect(minichannel_frontal.solid_surface_south,
        internal_convection_frontal.inlet) annotation (Line(points={{57,12.88},{
              57,21},{90,21}}, color={255,127,0}));
      connect(minichannel_lower.solid_surface_south, internal_convection_lower.inlet)
        annotation (Line(points={{57,-67.12},{57,-59},{90,-59}}, color={255,127,0}));
      connect(internal_convection_upper.outlet, internal_bc_upper.thermal_flux)
        annotation (Line(points={{90,107},{90,113}}, color={255,127,0}));
      connect(internal_convection_frontal.outlet, internal_bc_frontal.thermal_flux)
        annotation (Line(points={{90,27},{90,33}}, color={255,127,0}));
      connect(internal_convection_lower.outlet, internal_bc_lower.thermal_flux)
        annotation (Line(points={{90,-53},{90,-47}}, color={255,127,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-120},
                {180,140}})),                                        Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-160,-120},{180,140}})),
        experiment(
          StopTime=80,
          Interval=1,
          __Dymola_Algorithm="Dassl"));
    end WingSkinHEX;

    model WSHEX_hot_day
      WingSkinHEX_insulation_MIL WSHEX
        annotation (Placement(transformation(extent={{-22,-32},{40,14}})));
      Modelica.Blocks.Continuous.PID PID_m_flow(
        k=1000,
        Ti=0.01,
        initType=Modelica.Blocks.Types.Init.InitialOutput,
        y_start=314.16) annotation (Placement(transformation(
            extent={{-8,8},{8,-8}},
            rotation=90,
            origin={66,-20})));
      Modelica.Blocks.Math.Add add_m_flow(k1=-1, k2=+1) annotation (Placement(
            transformation(
            extent={{-8,-8},{8,8}},
            rotation=0,
            origin={48,-48})));
      Modelica.Blocks.Sources.RealExpression target_m_flow(y=WSHEX.m_flow/WSHEX.N_frontal)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={10,-52})));
      inner Components.Environment environment(
        ISA_plus=35,
        phi_amb=0.1,
        phi_amb_ground=0.1,
        T_ground=323.15,
        use_ext_sw=true,
        allowFlowReversal=false,
        initOpt=DynTherM.Choices.InitOpt.fixedState)
        annotation (Placement(transformation(extent={{52,52},{80,80}})));
      Modelica.Blocks.Sources.RealExpression Q_scaling(y=WSHEX.N_frontal)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-70,70})));
      inner parameter ExternData.JSONFile NLR_data(fileName=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://DynTherM/ExternalData/F9X/EPU_heat_load_NLR_case6A.json"))
        "JSON file" annotation (Placement(transformation(extent={{-80,-80},{-60,
                -60}})));
      inner parameter ExternData.JSONFile F9X_data(fileName=
            ModelicaServices.ExternalReferences.loadResource(
            "modelica://DynTherM/ExternalData/F9X/F9X_takeoff_climb.json"))
        "JSON file"
        annotation (Placement(transformation(extent={{-60,-80},{-40,-60}})));
      Modelica.Blocks.Sources.TimeTable Q_EPU(table=NLR_data.getRealArray2D(
              "Q_EPU",
              429,
              2))
        annotation (Placement(transformation(extent={{-80,42},{-64,58}})));
      Modelica.Blocks.Sources.Ramp altitude(
        height=7588,
        duration=1424,
        startTime=31)
        annotation (Placement(transformation(extent={{20,60},{36,76}})));
      Modelica.Blocks.Sources.TimeTable V_inf(table=F9X_data.getRealArray2D(
              "V_inf",
              429,
              2)) annotation (Placement(transformation(extent={{20,32},{36,48}})));
      Modelica.Blocks.Math.Division div_Q annotation (Placement(transformation(
            extent={{7,-7},{-7,7}},
            rotation=90,
            origin={-13,33})));
      Modelica.Blocks.Math.Add add_Q(k2=-1)
        annotation (Placement(transformation(extent={{-40,38},{-26,52}})));
      Modelica.Blocks.Continuous.LimPID PID_T(
        controllerType=Modelica.Blocks.Types.SimpleController.PI,
        k=1000,
        Ti=0.01,
        yMax=35e3,
        yMin=0,
        initType=Modelica.Blocks.Types.Init.InitialOutput,
        y_start=35e3,
        homotopyType=Modelica.Blocks.Types.LimiterHomotopy.UpperLimit)
        annotation (Placement(transformation(
            extent={{-8,8},{8,-8}},
            rotation=90,
            origin={-46,0})));
      Modelica.Blocks.Sources.Constant target_T(k=60 + 273.15)
        annotation (Placement(transformation(extent={{-80,-8},{-64,8}})));
    equation
      connect(add_m_flow.y, PID_m_flow.u) annotation (Line(points={{56.8,-48},{66,
              -48},{66,-29.6}}, color={0,0,127}));
      connect(PID_m_flow.y, WSHEX.pump_speed) annotation (Line(points={{66,
              -11.2},{66,22},{29.0588,22},{29.0588,15.0615}},
                                                       color={0,0,127}));
      connect(V_inf.y, environment.V_inf) annotation (Line(points={{36.8,40},{44,
              40},{44,54.8},{52,54.8}}, color={0,0,127}));
      connect(altitude.y, environment.altitude) annotation (Line(points={{36.8,68},
              {44,68},{44,60.4},{52,60.4}}, color={0,0,127}));
      connect(Q_scaling.y, div_Q.u2) annotation (Line(points={{-59,70},{-8.8,70},
              {-8.8,41.4}}, color={0,0,127}));
      connect(target_T.y, PID_T.u_m)
        annotation (Line(points={{-63.2,0},{-55.6,0}}, color={0,0,127}));
      connect(add_Q.y, div_Q.u1) annotation (Line(points={{-25.3,45},{-17.2,45},{
              -17.2,41.4}}, color={0,0,127}));
      connect(PID_T.y, add_Q.u2) annotation (Line(points={{-46,8.8},{-46,40.8},{
              -41.4,40.8}}, color={0,0,127}));
      connect(Q_EPU.y, add_Q.u1) annotation (Line(points={{-63.2,50},{-63.3,50},{
              -63.3,49.2},{-41.4,49.2}}, color={0,0,127}));
      connect(div_Q.y, WSHEX.Q_input) annotation (Line(points={{-13,25.3},{-13,
              20.1808},{-12.8824,20.1808},{-12.8824,15.0615}}, color={0,0,127}));
      connect(WSHEX.EPU_temperature, PID_T.u_s) annotation (Line(points={{
              -11.0588,-32},{-11.0588,-40},{-46,-40},{-46,-9.6}}, color={0,0,127}));
      connect(WSHEX.coolant_mass_flow, add_m_flow.u1) annotation (Line(points={{29.0588,
              -32},{29.0588,-43.2},{38.4,-43.2}},         color={0,0,127}));
      connect(target_m_flow.y, add_m_flow.u2) annotation (Line(points={{21,-52},{
              37.7,-52},{37.7,-52.8},{38.4,-52.8}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,
                -80},{80,80}})),                                     Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-80,-80},{80,80}})),
        experiment(
          StopTime=1455,
          Interval=1,
          __Dymola_Algorithm="Dassl"));
    end WSHEX_hot_day;

    model WingSkinHEX_insulation_MIL

      package Coolant = DynTherM.Media.IncompressibleTableBased.MIL_PRF_23699 "Coolant";
      outer DynTherM.Components.Environment environment "Environmental properties";

      Real N_upper "Number of minichannels located in the upper section";
      Real N_frontal "Number of minichannels located in the frontal section";
      Real N_lower "Number of minichannels located in the lower section";

      IPS F9X(
        section=DynTherM.Choices.WingSection.Root,
        m_flow_root=8,
        m_flow_mean=1.6,
        m_flow_tip=3.2,
        T_in=343.15,
        P_in=200000,
        N_cv=3,
        T_start_solid=323.15,
        T_start_fluid=333.15,
        P_start=3000000,
        m_flow_start=0.12)
        annotation (Placement(transformation(extent={{-150,
                112},{-130,132}})));
      Components.OneDimensional.RectangularChannels1D minichannel_upper(
        redeclare model Mat = Materials.Aluminium,
        redeclare package Medium = Coolant,
        L=F9X.L,
        W=F9X.W,
        H=F9X.H,
        t_north=F9X.t_ext + F9X.t_skin,
        t_east=F9X.t_int,
        t_south=F9X.t_ext,
        t_west=F9X.t_int,
        T_start_solid=F9X.T_start_solid,
        T_start_fluid=F9X.T_start_fluid,
        P_start=F9X.P_start,
        X_start=X_start,
        state_start=state_start,
        m_flow_start=F9X.m_flow_start,
        N=F9X.N_cv,
        N_channels=1)
        annotation (Placement(transformation(extent={{22,52},{78,108}})));
      Components.OneDimensional.ExternalConvection1D convection_upper(redeclare
          model HTC =
            DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.Wing
            (c=c),  N=F9X.N_cv)
        annotation (Placement(transformation(extent={{-16,136},{16,106}})));
      Components.HeatTransfer.SolarRadiation solar_radiation_upper(
        E_tb_fixed=973.43,
        E_td_fixed=85.7,
        theta_fixed(displayUnit="deg") = 0.13055062804918,
        csi=0)
        annotation (Placement(transformation(
            extent={{-18,18},{18,-18}},
            rotation=-90,
            origin={108,120})));
      Components.Adaptors.irradianceMultiplier multiplier_upper(Nx=F9X.N_cv, Ny=1)
        annotation (Placement(transformation(
            extent={{-12,12},{12,-12}},
            rotation=-90,
            origin={70,120})));
      Components.OneDimensional.WallRadiation1D radiation_upper(
        redeclare model Material =
            Materials.Paints.ConductivePaints.BrilliantAluminumPaint,
        csi=solar_radiation_upper.csi,
        N=F9X.N_cv) annotation (Placement(transformation(
            extent={{-16,15},{16,-15}},
            rotation=-90,
            origin={34,121})));
      Components.MassTransfer.PlenumSimple plenum(
        redeclare package Medium = Coolant,
        V=1e-4,
        m_flow_start=F9X.m_flow_start*3,
        P_start=F9X.P_start,
        T_start=F9X.T_start_fluid,
        X_start=X_start,
        state_start=state_start,
        noInitialPressure=true)
        annotation (Placement(transformation(extent={{-122,-12},{-98,12}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow EPU annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-110,110})));
      Modelica.Blocks.Interfaces.RealInput Q_input annotation (Placement(
            transformation(
            extent={{-14,-14},{14,14}},
            rotation=-90,
            origin={-110,146}), iconTransformation(
            extent={{-14,-14},{14,14}},
            rotation=-90,
            origin={-110,146})));
      Components.MassTransfer.Fan pump(
        redeclare package Medium = Coolant,
        eta_is=0.7,
        eta_m=0.99,
        omega_nom(displayUnit="rad/s") = 314.15926535898,
        volFlow_nom=15.6e-5,
        Head_nom=4e3,
        rho_start(displayUnit="kg/m3") = 1000,
        redeclare
          DynTherM.Components.MassTransfer.FanCharacteristics.FlowCharacteristics.linearFlow
          flowModel)
        annotation (Placement(transformation(extent={{-60,-14},{-32,14}})));
      BoundaryConditions.mechanical mechanical(
        omega(displayUnit="rpm") = 314.15926535898,
        use_omega=false,
        use_in_omega=true)
        annotation (Placement(transformation(extent={{9,-6},{-9,6}},
            rotation=180,
            origin={-49,54})));
      Components.OneDimensional.RectangularChannels1D minichannel_frontal(
        redeclare model Mat = Materials.Aluminium,
        redeclare package Medium = Coolant,
        L=F9X.L,
        W=F9X.W,
        H=F9X.H,
        t_north=F9X.t_ext + F9X.t_skin,
        t_east=F9X.t_int,
        t_south=F9X.t_ext,
        t_west=F9X.t_int,
        T_start_solid=F9X.T_start_solid,
        T_start_fluid=F9X.T_start_fluid,
        P_start=F9X.P_start,
        X_start=X_start,
        state_start=state_start,
        m_flow_start=F9X.m_flow_start,
        N=F9X.N_cv,
        N_channels=1)
        annotation (Placement(transformation(extent={{22,-28},{78,28}})));
      Components.OneDimensional.ExternalConvection1D convection_frontal(redeclare
          model HTC =
            DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.Wing
            (c=c),  N=F9X.N_cv)
        annotation (Placement(transformation(extent={{-16,56},{16,26}})));
      Components.HeatTransfer.SolarRadiation solar_radiation_frontal(
        E_tb_fixed=48.11,
        E_td_fixed=225.27,
        E_tr_fixed=182.71,
        theta_fixed(displayUnit="deg") = 1.5217525748139,
        csi=1.5707963267949)
        annotation (Placement(transformation(
            extent={{-18,18},{18,-18}},
            rotation=-90,
            origin={108,40})));
      Components.Adaptors.irradianceMultiplier multiplier_frontal(Nx=F9X.N_cv, Ny=1)
        annotation (Placement(transformation(
            extent={{-12,12},{12,-12}},
            rotation=-90,
            origin={70,40})));
      Components.OneDimensional.WallRadiation1D radiation_frontal(
        redeclare model Material =
            Materials.Paints.ConductivePaints.BrilliantAluminumPaint,
        csi=solar_radiation_frontal.csi,
        N=F9X.N_cv) annotation (Placement(transformation(
            extent={{-16,15},{16,-15}},
            rotation=-90,
            origin={34,41})));
      Components.OneDimensional.RectangularChannels1D minichannel_lower(
        redeclare model Mat = Materials.Aluminium,
        redeclare package Medium = Coolant,
        L=F9X.L,
        W=F9X.W,
        H=F9X.H,
        t_north=F9X.t_ext + F9X.t_skin,
        t_east=F9X.t_int,
        t_south=F9X.t_ext,
        t_west=F9X.t_int,
        T_start_solid=F9X.T_start_solid,
        T_start_fluid=F9X.T_start_fluid,
        P_start=F9X.P_start,
        X_start=X_start,
        state_start=state_start,
        m_flow_start=F9X.m_flow_start,
        N=F9X.N_cv,
        N_channels=1)
        annotation (Placement(transformation(extent={{22,-108},{78,-52}})));
      Components.OneDimensional.ExternalConvection1D convection_lower(redeclare
          model HTC =
            DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.Wing
            (c=c),  N=F9X.N_cv)
        annotation (Placement(transformation(extent={{-16,-24},{16,-54}})));
      Components.HeatTransfer.SolarRadiation solar_radiation_lower(
        E_td_fixed=366.76,
        E_tr_fixed=365.43,
        theta_fixed(displayUnit="deg") = 3.0110420255406,
        csi=3.1415926535898)
        annotation (Placement(transformation(
            extent={{-18,18},{18,-18}},
            rotation=-90,
            origin={108,-40})));
      Components.Adaptors.irradianceMultiplier multiplier_lower(Nx=F9X.N_cv, Ny=1)
        annotation (Placement(transformation(
            extent={{-12,12},{12,-12}},
            rotation=-90,
            origin={70,-40})));
      Components.OneDimensional.WallRadiation1D radiation_lower(
        redeclare model Material =
            Materials.Paints.ConductivePaints.BrilliantAluminumPaint,
        csi=solar_radiation_lower.csi,
        N=F9X.N_cv) annotation (Placement(transformation(
            extent={{-16,15},{16,-15}},
            rotation=-90,
            origin={34,-39})));
      Modelica.Blocks.Interfaces.RealInput pump_speed annotation (Placement(
            transformation(
            extent={{14,-14},{-14,14}},
            rotation=180,
            origin={-166,60}), iconTransformation(
            extent={{-14,-14},{14,14}},
            rotation=-90,
            origin={120,146})));
      Modelica.Blocks.Interfaces.RealOutput coolant_mass_flow annotation (
          Placement(transformation(
            extent={{-12,-12},{12,12}},
            rotation=-90,
            origin={-80,-128}), iconTransformation(
            extent={{14,-14},{-14,14}},
            rotation=90,
            origin={120,-120})));
      Sensors.MassflowSensor sensor(redeclare package Medium = Coolant)
        annotation (Placement(transformation(extent={{-90,10},{-70,-10}})));
      BoundaryConditions.pressure_sink pressure_sink(
        redeclare package Medium = Coolant,
        use_ambient=false,
        P_di=F9X.P_in,
        T_di=F9X.T_in)
        annotation (Placement(transformation(extent={{150,-10},{170,10}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
        annotation (Placement(transformation(extent={{-120,20},{-140,40}})));
      Modelica.Blocks.Interfaces.RealOutput EPU_temperature annotation (Placement(
            transformation(
            extent={{-12,-12},{12,12}},
            rotation=180,
            origin={-168,30}), iconTransformation(
            extent={{14,-14},{-14,14}},
            rotation=90,
            origin={-100,-120})));
    protected
      parameter Coolant.ThermodynamicState state_start=
        Coolant.setState_pTX(F9X.P_start, F9X.T_start_fluid, X_start)
        "Starting thermodynamic state";
      parameter MassFraction X_start[Coolant.nX]=Coolant.reference_X
        "Mass fractions - start value";

      Length c "Airfoil chord related to the wing section under analysis";
      Real N "Number of minichannels related to the wing section under analysis";
      MassFlowRate m_flow "Mass flow rate related to the wing section under analysis";

    equation

      // Geometry
      F9X.R_le_root = F9X.t_root/2;
      F9X.R_le_mean = F9X.t_mean/2;
      F9X.R_le_tip  = F9X.t_tip/2;

      F9X.x_front_spar_root = F9X.c_root*F9X.xc_front_spar_root;
      F9X.x_front_spar_mean = F9X.c_mean*F9X.xc_front_spar_mean;
      F9X.x_front_spar_tip  = F9X.c_tip*F9X.xc_front_spar_tip;

      F9X.W_IPS_root = (F9X.x_front_spar_root - F9X.R_le_root) + pi*F9X.R_le_root;
      F9X.W_IPS_mean = (F9X.x_front_spar_mean - F9X.R_le_mean) + pi*F9X.R_le_mean;
      F9X.W_IPS_tip  = (F9X.x_front_spar_tip  - F9X.R_le_tip)  + pi*F9X.R_le_tip;

      F9X.N_root = ceil(F9X.W_IPS_root/F9X.W);
      F9X.N_mean = ceil(F9X.W_IPS_mean/F9X.W);
      F9X.N_tip  = ceil(F9X.W_IPS_tip/F9X.W);

      N_upper = ceil(N/3);
      N_lower = ceil(N/3);
      N_frontal = N - N_upper - N_lower;

      F9X.m_flow_mc = m_flow/N;
      F9X.m_mc      = minichannel_upper.m_tot*N_upper +
                      minichannel_frontal.m_tot*N_frontal +
                      minichannel_lower.m_tot*N_lower;
      F9X.Q_mc      = minichannel_upper.Q*N_upper +
                      minichannel_frontal.Q*N_frontal +
                      minichannel_lower.Q*N_lower;

      if F9X.section == DynTherM.Choices.WingSection.Root then
        N      = F9X.N_root;
        c      = F9X.c_root;
        m_flow = F9X.m_flow_root;

      elseif F9X.section == DynTherM.Choices.WingSection.Mean then
        N      = F9X.N_mean;
        c      = F9X.c_mean;
        m_flow = F9X.m_flow_mean;

      elseif F9X.section == DynTherM.Choices.WingSection.Tip then
        N      = F9X.N_tip;
        c      = F9X.c_tip;
        m_flow = F9X.m_flow_tip;

      end if;

    //   m_pump_root =0.453592*(8.5942*(0.0092*(2*m_flow_root/(minichannel.cv[1].fluid.rho
    //      + minichannel.cv[N_cv].fluid.rho)*61023.78)^1.3857) + 2.4229);
    //   m_pump_mean = 0.453592*(8.5942*(0.0092*(2*m_flow_mean/
    //     (minichannel_mean.cv[1].fluid.rho + minichannel_mean.cv[N_cv].fluid.rho)*61023.78)^1.3857) + 2.4229);
    //   m_pump_tip = 0.453592*(8.5942*(0.0092*(2*m_flow_tip/
    //     (minichannel_tip.cv[1].fluid.rho + minichannel_tip.cv[N_cv].fluid.rho)*61023.78)^1.3857) + 2.4229);
    //   m_pump_tot = 2*(m_pump_root + 2*m_pump_mean + m_pump_tip);
    //   m_tot = m_mc_tot + m_pump_tot;

      connect(radiation_upper.outlet, multiplier_upper.distributed) annotation (
          Line(points={{35.5,121},{35.5,120},{62.8,120}},
                                                        color={238,46,47}));
      connect(multiplier_upper.single, solar_radiation_upper.inlet)
        annotation (Line(points={{77.2,120},{96.84,120}}, color={191,0,0}));
      connect(plenum.thermalPort, EPU.port)
        annotation (Line(points={{-110,10.8},{-110,100}}, color={191,0,0}));
      connect(EPU.Q_flow, Q_input)
        annotation (Line(points={{-110,120},{-110,146}}, color={0,0,127}));
      connect(mechanical.mechanical, pump.shaft)
        annotation (Line(points={{-46,54},{-46,14}}, color={135,135,135}));
      connect(radiation_frontal.outlet, multiplier_frontal.distributed)
        annotation (Line(points={{35.5,41},{35.5,40},{62.8,40}},
                                                               color={238,46,47}));
      connect(multiplier_frontal.single, solar_radiation_frontal.inlet)
        annotation (Line(points={{77.2,40},{96.84,40}}, color={191,0,0}));
      connect(radiation_lower.outlet, multiplier_lower.distributed) annotation (
          Line(points={{35.5,-39},{35.5,-40},{62.8,-40}},
                                                        color={238,46,47}));
      connect(multiplier_lower.single, solar_radiation_lower.inlet)
        annotation (Line(points={{77.2,-40},{96.84,-40}}, color={191,0,0}));
      connect(pump_speed, mechanical.in_omega)
        annotation (Line(points={{-166,60},{-59.8,60}}, color={0,0,127}));
      connect(plenum.outlet, sensor.inlet)
        annotation (Line(points={{-98,0},{-90,0}}, color={0,0,0}));
      connect(sensor.outlet, pump.inlet) annotation (Line(points={{-70,0},{-67,0},{-67,
              1.77636e-15},{-60,1.77636e-15}}, color={0,0,0}));
      connect(sensor.y, coolant_mass_flow)
        annotation (Line(points={{-80,-11},{-80,-128}}, color={0,0,127}));
      connect(pump.outlet, minichannel_frontal.inlet) annotation (Line(points={{
              -32,1.77636e-15},{-5,1.77636e-15},{-5,3.55271e-15},{22,3.55271e-15}},
            color={0,0,0}));
      connect(pump.outlet, minichannel_upper.inlet) annotation (Line(points={{-32,
              0},{-20,0},{-20,80},{22,80}}, color={0,0,0}));
      connect(pump.outlet, minichannel_lower.inlet) annotation (Line(points={{-32,
              0},{-20,0},{-20,-80},{22,-80}}, color={0,0,0}));
      connect(minichannel_upper.solid_surface_north, convection_upper.inlet)
        annotation (Line(points={{29,92.88},{0,92.88},{0,116.5}}, color={255,127,
              0}));
      connect(minichannel_frontal.solid_surface_north, convection_frontal.inlet)
        annotation (Line(points={{29,12.88},{0,12.88},{0,36.5}}, color={255,127,0}));
      connect(minichannel_lower.solid_surface_north, convection_lower.inlet)
        annotation (Line(points={{29,-67.12},{0,-67.12},{0,-43.5}}, color={255,
              127,0}));
      connect(minichannel_frontal.solid_surface_north, radiation_frontal.inlet)
        annotation (Line(points={{29,12.88},{29,34.94},{29.5,34.94},{29.5,41}},
            color={255,127,0}));
      connect(minichannel_lower.solid_surface_north, radiation_lower.inlet)
        annotation (Line(points={{29,-67.12},{29,-45.06},{29.5,-45.06},{29.5,-39}},
            color={255,127,0}));
      connect(minichannel_upper.solid_surface_north, radiation_upper.inlet)
        annotation (Line(points={{29,92.88},{28,92.88},{28,118},{29.5,118},{29.5,
              121}}, color={255,127,0}));
      connect(minichannel_frontal.outlet, pressure_sink.inlet) annotation (Line(
            points={{78,3.55271e-15},{114,3.55271e-15},{114,0},{150,0}}, color={0,0,
              0}));
      connect(minichannel_upper.outlet, pressure_sink.inlet)
        annotation (Line(points={{78,80},{150,80},{150,0}}, color={0,0,0}));
      connect(minichannel_lower.outlet, pressure_sink.inlet)
        annotation (Line(points={{78,-80},{150,-80},{150,0}}, color={0,0,0}));
      connect(pressure_sink.inlet, plenum.inlet) annotation (Line(points={{150,0},{150,
              -100},{-140,-100},{-140,0},{-122,0}}, color={0,0,0}));
      connect(EPU_temperature, temperatureSensor.T)
        annotation (Line(points={{-168,30},{-141,30}}, color={0,0,127}));
      connect(temperatureSensor.port, EPU.port) annotation (Line(points={{-120,30},
              {-110,30},{-110,100}}, color={191,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-120},
                {180,140}}), graphics={Bitmap(extent={{-138,-134},{158,162}},
                fileName="modelica://DynTherM/Figures/AntiIcing.png")}),
                                                                     Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-160,-120},{180,140}})),
        experiment(
          StopTime=80,
          Interval=1,
          __Dymola_Algorithm="Dassl"));
    end WingSkinHEX_insulation_MIL;

    model WingSkinHEX_insulation_MEG50

      package Coolant = DynTherM.Media.IncompressibleTableBased.MEG(X=0.5) "Coolant";
      outer DynTherM.Components.Environment environment "Environmental properties";

      Real N_upper "Number of minichannels located in the upper section";
      Real N_frontal "Number of minichannels located in the frontal section";
      Real N_lower "Number of minichannels located in the lower section";

      IPS F9X(
        section=DynTherM.Choices.WingSection.Tip,
        m_flow_mean=1.6,
        m_flow_tip=2,
        T_in=343.15,
        P_in=200000,
        N_cv=3,
        T_start_solid=323.15,
        T_start_fluid=333.15,
        P_start=300000,
        m_flow_start=0.01)
        annotation (Placement(transformation(extent={{-150,
                112},{-130,132}})));
      Components.OneDimensional.RectangularChannels1D minichannel_upper(
        redeclare model Mat = Materials.Aluminium,
        redeclare package Medium = Coolant,
        L=F9X.L,
        W=F9X.W,
        H=F9X.H,
        t_north=F9X.t_ext + F9X.t_skin,
        t_east=F9X.t_int,
        t_south=F9X.t_ext,
        t_west=F9X.t_int,
        T_start_solid=F9X.T_start_solid,
        T_start_fluid=F9X.T_start_fluid,
        P_start=F9X.P_start,
        X_start=X_start,
        state_start=state_start,
        m_flow_start=F9X.m_flow_start,
        N=F9X.N_cv,
        N_channels=1)
        annotation (Placement(transformation(extent={{22,52},{78,108}})));
      Components.OneDimensional.ExternalConvection1D convection_upper(redeclare
          model HTC =
            DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.Wing
            (c=c),  N=F9X.N_cv)
        annotation (Placement(transformation(extent={{-16,136},{16,106}})));
      Components.HeatTransfer.SolarRadiation solar_radiation_upper(
        E_tb_fixed=973.43,
        E_td_fixed=85.7,
        theta_fixed(displayUnit="deg") = 0.13055062804918,
        csi=0)
        annotation (Placement(transformation(
            extent={{-18,18},{18,-18}},
            rotation=-90,
            origin={108,120})));
      Components.Adaptors.irradianceMultiplier multiplier_upper(Nx=F9X.N_cv, Ny=1)
        annotation (Placement(transformation(
            extent={{-12,12},{12,-12}},
            rotation=-90,
            origin={70,120})));
      Components.OneDimensional.WallRadiation1D radiation_upper(
        redeclare model Material =
            Materials.Paints.ConductivePaints.BrilliantAluminumPaint,
        csi=solar_radiation_upper.csi,
        N=F9X.N_cv) annotation (Placement(transformation(
            extent={{-16,15},{16,-15}},
            rotation=-90,
            origin={34,121})));
      Components.MassTransfer.PlenumSimple plenum(
        redeclare package Medium = Coolant,
        V=1e-4,
        m_flow_start=F9X.m_flow_start*3,
        P_start=F9X.P_start,
        T_start=F9X.T_start_fluid,
        X_start=X_start,
        state_start=state_start,
        noInitialPressure=true)
        annotation (Placement(transformation(extent={{-122,-12},{-98,12}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow EPU annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-110,110})));
      Modelica.Blocks.Interfaces.RealInput Q_input annotation (Placement(
            transformation(
            extent={{-14,-14},{14,14}},
            rotation=-90,
            origin={-110,146}), iconTransformation(
            extent={{-14,-14},{14,14}},
            rotation=-90,
            origin={-110,146})));
      Components.MassTransfer.Fan pump(
        redeclare package Medium = Coolant,
        eta_is=0.7,
        eta_m=0.99,
        omega_nom(displayUnit="rad/s") = 314.15926535898,
        volFlow_nom=3e-5,
        Head_nom=75,
        rho_start(displayUnit="kg/m3") = 1000,
        redeclare
          DynTherM.Components.MassTransfer.FanCharacteristics.FlowCharacteristics.linearFlow
          flowModel)
        annotation (Placement(transformation(extent={{-60,-14},{-32,14}})));
      BoundaryConditions.mechanical mechanical(
        omega(displayUnit="rpm") = 314.15926535898,
        use_omega=false,
        use_in_omega=true)
        annotation (Placement(transformation(extent={{9,-6},{-9,6}},
            rotation=180,
            origin={-49,54})));
      Components.OneDimensional.RectangularChannels1D minichannel_frontal(
        redeclare model Mat = Materials.Aluminium,
        redeclare package Medium = Coolant,
        L=F9X.L,
        W=F9X.W,
        H=F9X.H,
        t_north=F9X.t_ext + F9X.t_skin,
        t_east=F9X.t_int,
        t_south=F9X.t_ext,
        t_west=F9X.t_int,
        T_start_solid=F9X.T_start_solid,
        T_start_fluid=F9X.T_start_fluid,
        P_start=F9X.P_start,
        X_start=X_start,
        state_start=state_start,
        m_flow_start=F9X.m_flow_start,
        N=F9X.N_cv,
        N_channels=1)
        annotation (Placement(transformation(extent={{22,-28},{78,28}})));
      Components.OneDimensional.ExternalConvection1D convection_frontal(redeclare
          model HTC =
            DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.Wing
            (c=c),  N=F9X.N_cv)
        annotation (Placement(transformation(extent={{-16,56},{16,26}})));
      Components.HeatTransfer.SolarRadiation solar_radiation_frontal(
        E_tb_fixed=48.11,
        E_td_fixed=225.27,
        E_tr_fixed=182.71,
        theta_fixed(displayUnit="deg") = 1.5217525748139,
        csi=1.5707963267949)
        annotation (Placement(transformation(
            extent={{-18,18},{18,-18}},
            rotation=-90,
            origin={108,40})));
      Components.Adaptors.irradianceMultiplier multiplier_frontal(Nx=F9X.N_cv, Ny=1)
        annotation (Placement(transformation(
            extent={{-12,12},{12,-12}},
            rotation=-90,
            origin={70,40})));
      Components.OneDimensional.WallRadiation1D radiation_frontal(
        redeclare model Material =
            Materials.Paints.ConductivePaints.BrilliantAluminumPaint,
        csi=solar_radiation_frontal.csi,
        N=F9X.N_cv) annotation (Placement(transformation(
            extent={{-16,15},{16,-15}},
            rotation=-90,
            origin={34,41})));
      Components.OneDimensional.RectangularChannels1D minichannel_lower(
        redeclare model Mat = Materials.Aluminium,
        redeclare package Medium = Coolant,
        L=F9X.L,
        W=F9X.W,
        H=F9X.H,
        t_north=F9X.t_ext + F9X.t_skin,
        t_east=F9X.t_int,
        t_south=F9X.t_ext,
        t_west=F9X.t_int,
        T_start_solid=F9X.T_start_solid,
        T_start_fluid=F9X.T_start_fluid,
        P_start=F9X.P_start,
        X_start=X_start,
        state_start=state_start,
        m_flow_start=F9X.m_flow_start,
        N=F9X.N_cv,
        N_channels=1)
        annotation (Placement(transformation(extent={{22,-108},{78,-52}})));
      Components.OneDimensional.ExternalConvection1D convection_lower(redeclare
          model HTC =
            DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.Wing
            (c=c),  N=F9X.N_cv)
        annotation (Placement(transformation(extent={{-16,-24},{16,-54}})));
      Components.HeatTransfer.SolarRadiation solar_radiation_lower(
        E_td_fixed=366.76,
        E_tr_fixed=365.43,
        theta_fixed(displayUnit="deg") = 3.0110420255406,
        csi=3.1415926535898)
        annotation (Placement(transformation(
            extent={{-18,18},{18,-18}},
            rotation=-90,
            origin={108,-40})));
      Components.Adaptors.irradianceMultiplier multiplier_lower(Nx=F9X.N_cv, Ny=1)
        annotation (Placement(transformation(
            extent={{-12,12},{12,-12}},
            rotation=-90,
            origin={70,-40})));
      Components.OneDimensional.WallRadiation1D radiation_lower(
        redeclare model Material =
            Materials.Paints.ConductivePaints.BrilliantAluminumPaint,
        csi=solar_radiation_lower.csi,
        N=F9X.N_cv) annotation (Placement(transformation(
            extent={{-16,15},{16,-15}},
            rotation=-90,
            origin={34,-39})));
      Modelica.Blocks.Interfaces.RealInput pump_speed annotation (Placement(
            transformation(
            extent={{14,-14},{-14,14}},
            rotation=180,
            origin={-166,60}), iconTransformation(
            extent={{-14,-14},{14,14}},
            rotation=-90,
            origin={120,146})));
      Modelica.Blocks.Interfaces.RealOutput coolant_mass_flow annotation (
          Placement(transformation(
            extent={{-12,-12},{12,12}},
            rotation=-90,
            origin={-80,-128}), iconTransformation(
            extent={{14,-14},{-14,14}},
            rotation=90,
            origin={120,-120})));
      Sensors.MassflowSensor sensor(redeclare package Medium = Coolant)
        annotation (Placement(transformation(extent={{-90,10},{-70,-10}})));
      BoundaryConditions.pressure_sink pressure_sink(
        redeclare package Medium = Coolant,
        use_ambient=false,
        P_di=F9X.P_in,
        T_di=F9X.T_in)
        annotation (Placement(transformation(extent={{150,-10},{170,10}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
        annotation (Placement(transformation(extent={{-120,20},{-140,40}})));
      Modelica.Blocks.Interfaces.RealOutput EPU_temperature annotation (Placement(
            transformation(
            extent={{-12,-12},{12,12}},
            rotation=180,
            origin={-168,30}), iconTransformation(
            extent={{14,-14},{-14,14}},
            rotation=90,
            origin={-100,-120})));
    protected
      parameter Coolant.ThermodynamicState state_start=
        Coolant.setState_pTX(F9X.P_start, F9X.T_start_fluid, X_start)
        "Starting thermodynamic state";
      parameter MassFraction X_start[Coolant.nX]=Coolant.reference_X
        "Mass fractions - start value";

      Length c "Airfoil chord related to the wing section under analysis";
      Real N "Number of minichannels related to the wing section under analysis";
      MassFlowRate m_flow "Mass flow rate related to the wing section under analysis";

    equation

      // Geometry
      F9X.R_le_root = F9X.t_root/2;
      F9X.R_le_mean = F9X.t_mean/2;
      F9X.R_le_tip  = F9X.t_tip/2;

      F9X.x_front_spar_root = F9X.c_root*F9X.xc_front_spar_root;
      F9X.x_front_spar_mean = F9X.c_mean*F9X.xc_front_spar_mean;
      F9X.x_front_spar_tip  = F9X.c_tip*F9X.xc_front_spar_tip;

      F9X.W_IPS_root = (F9X.x_front_spar_root - F9X.R_le_root) + pi*F9X.R_le_root;
      F9X.W_IPS_mean = (F9X.x_front_spar_mean - F9X.R_le_mean) + pi*F9X.R_le_mean;
      F9X.W_IPS_tip  = (F9X.x_front_spar_tip  - F9X.R_le_tip)  + pi*F9X.R_le_tip;

      F9X.N_root = ceil(F9X.W_IPS_root/F9X.W);
      F9X.N_mean = ceil(F9X.W_IPS_mean/F9X.W);
      F9X.N_tip  = ceil(F9X.W_IPS_tip/F9X.W);

      N_upper = ceil(N/3);
      N_lower = ceil(N/3);
      N_frontal = N - N_upper - N_lower;

      F9X.m_flow_mc = m_flow/N;
      F9X.m_mc      = minichannel_upper.m_tot*N_upper +
                      minichannel_frontal.m_tot*N_frontal +
                      minichannel_lower.m_tot*N_lower;
      F9X.Q_mc      = minichannel_upper.Q*N_upper +
                      minichannel_frontal.Q*N_frontal +
                      minichannel_lower.Q*N_lower;

      if F9X.section == DynTherM.Choices.WingSection.Root then
        N      = F9X.N_root;
        c      = F9X.c_root;
        m_flow = F9X.m_flow_root;

      elseif F9X.section == DynTherM.Choices.WingSection.Mean then
        N      = F9X.N_mean;
        c      = F9X.c_mean;
        m_flow = F9X.m_flow_mean;

      elseif F9X.section == DynTherM.Choices.WingSection.Tip then
        N      = F9X.N_tip;
        c      = F9X.c_tip;
        m_flow = F9X.m_flow_tip;

      end if;

    //   m_pump_root =0.453592*(8.5942*(0.0092*(2*m_flow_root/(minichannel.cv[1].fluid.rho
    //      + minichannel.cv[N_cv].fluid.rho)*61023.78)^1.3857) + 2.4229);
    //   m_pump_mean = 0.453592*(8.5942*(0.0092*(2*m_flow_mean/
    //     (minichannel_mean.cv[1].fluid.rho + minichannel_mean.cv[N_cv].fluid.rho)*61023.78)^1.3857) + 2.4229);
    //   m_pump_tip = 0.453592*(8.5942*(0.0092*(2*m_flow_tip/
    //     (minichannel_tip.cv[1].fluid.rho + minichannel_tip.cv[N_cv].fluid.rho)*61023.78)^1.3857) + 2.4229);
    //   m_pump_tot = 2*(m_pump_root + 2*m_pump_mean + m_pump_tip);
    //   m_tot = m_mc_tot + m_pump_tot;

      connect(radiation_upper.outlet, multiplier_upper.distributed) annotation (
          Line(points={{35.5,121},{35.5,120},{62.8,120}},
                                                        color={238,46,47}));
      connect(multiplier_upper.single, solar_radiation_upper.inlet)
        annotation (Line(points={{77.2,120},{96.84,120}}, color={191,0,0}));
      connect(plenum.thermalPort, EPU.port)
        annotation (Line(points={{-110,10.8},{-110,100}}, color={191,0,0}));
      connect(EPU.Q_flow, Q_input)
        annotation (Line(points={{-110,120},{-110,146}}, color={0,0,127}));
      connect(mechanical.mechanical, pump.shaft)
        annotation (Line(points={{-46,54},{-46,14}}, color={135,135,135}));
      connect(radiation_frontal.outlet, multiplier_frontal.distributed)
        annotation (Line(points={{35.5,41},{35.5,40},{62.8,40}},
                                                               color={238,46,47}));
      connect(multiplier_frontal.single, solar_radiation_frontal.inlet)
        annotation (Line(points={{77.2,40},{96.84,40}}, color={191,0,0}));
      connect(radiation_lower.outlet, multiplier_lower.distributed) annotation (
          Line(points={{35.5,-39},{35.5,-40},{62.8,-40}},
                                                        color={238,46,47}));
      connect(multiplier_lower.single, solar_radiation_lower.inlet)
        annotation (Line(points={{77.2,-40},{96.84,-40}}, color={191,0,0}));
      connect(pump_speed, mechanical.in_omega)
        annotation (Line(points={{-166,60},{-59.8,60}}, color={0,0,127}));
      connect(plenum.outlet, sensor.inlet)
        annotation (Line(points={{-98,0},{-90,0}}, color={0,0,0}));
      connect(sensor.outlet, pump.inlet) annotation (Line(points={{-70,0},{-67,0},{-67,
              1.77636e-15},{-60,1.77636e-15}}, color={0,0,0}));
      connect(sensor.y, coolant_mass_flow)
        annotation (Line(points={{-80,-11},{-80,-128}}, color={0,0,127}));
      connect(pump.outlet, minichannel_frontal.inlet) annotation (Line(points={{
              -32,1.77636e-15},{-5,1.77636e-15},{-5,3.55271e-15},{22,3.55271e-15}},
            color={0,0,0}));
      connect(pump.outlet, minichannel_upper.inlet) annotation (Line(points={{-32,
              0},{-20,0},{-20,80},{22,80}}, color={0,0,0}));
      connect(pump.outlet, minichannel_lower.inlet) annotation (Line(points={{-32,
              0},{-20,0},{-20,-80},{22,-80}}, color={0,0,0}));
      connect(minichannel_upper.solid_surface_north, convection_upper.inlet)
        annotation (Line(points={{29,92.88},{0,92.88},{0,116.5}}, color={255,127,
              0}));
      connect(minichannel_frontal.solid_surface_north, convection_frontal.inlet)
        annotation (Line(points={{29,12.88},{0,12.88},{0,36.5}}, color={255,127,0}));
      connect(minichannel_lower.solid_surface_north, convection_lower.inlet)
        annotation (Line(points={{29,-67.12},{0,-67.12},{0,-43.5}}, color={255,
              127,0}));
      connect(minichannel_frontal.solid_surface_north, radiation_frontal.inlet)
        annotation (Line(points={{29,12.88},{29,34.94},{29.5,34.94},{29.5,41}},
            color={255,127,0}));
      connect(minichannel_lower.solid_surface_north, radiation_lower.inlet)
        annotation (Line(points={{29,-67.12},{29,-45.06},{29.5,-45.06},{29.5,-39}},
            color={255,127,0}));
      connect(minichannel_upper.solid_surface_north, radiation_upper.inlet)
        annotation (Line(points={{29,92.88},{28,92.88},{28,118},{29.5,118},{29.5,
              121}}, color={255,127,0}));
      connect(minichannel_frontal.outlet, pressure_sink.inlet) annotation (Line(
            points={{78,3.55271e-15},{114,3.55271e-15},{114,0},{150,0}}, color={0,0,
              0}));
      connect(minichannel_upper.outlet, pressure_sink.inlet)
        annotation (Line(points={{78,80},{150,80},{150,0}}, color={0,0,0}));
      connect(minichannel_lower.outlet, pressure_sink.inlet)
        annotation (Line(points={{78,-80},{150,-80},{150,0}}, color={0,0,0}));
      connect(pressure_sink.inlet, plenum.inlet) annotation (Line(points={{150,0},{150,
              -100},{-140,-100},{-140,0},{-122,0}}, color={0,0,0}));
      connect(EPU_temperature, temperatureSensor.T)
        annotation (Line(points={{-168,30},{-141,30}}, color={0,0,127}));
      connect(temperatureSensor.port, EPU.port) annotation (Line(points={{-120,30},
              {-110,30},{-110,100}}, color={191,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-120},
                {180,140}}), graphics={Bitmap(extent={{-138,-134},{158,162}},
                fileName="modelica://DynTherM/Figures/AntiIcing.png")}),
                                                                     Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-160,-120},{180,140}})),
        experiment(
          StopTime=80,
          Interval=1,
          __Dymola_Algorithm="Dassl"));
    end WingSkinHEX_insulation_MEG50;
  end IceProtectionSystem;

  package BatteryPack
    model IsolatedPackControlled
      // Battery pack modeled as a single lumped mass
      // No heat transfer with external environment
      // Battery temperature is actively controlled by TMS
      inner parameter ExternData.JSONFile E9X_data(fileName=
            ModelicaServices.ExternalReferences.loadResource("modelica://DynTherM/ExternalData/F9X/E9X_data.json"))
        "JSON file"
        annotation (Placement(transformation(extent={{60,-100},{80,-80}})));
      Modelica.Blocks.Sources.TimeTable P_bat(table=E9X_data.getRealArray2D(
            "P_bat",
            443,
            2))   annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
      Modelica.Blocks.Math.Product product1
        annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
      Modelica.Blocks.Sources.Constant const(k=1 - data_battery.eta_pack)
        annotation (Placement(transformation(extent={{-100,40},{-80,60}})));
      Battery.BatteryPackData data_battery(E_pack=0)
        annotation (Placement(transformation(extent={{82,-98},{98,-80}})));
      Components.HeatTransfer.HeatCapacity battery_pack(T_start=293.15, C=
            data_battery.m_tot*data_battery.c)
        annotation (Placement(transformation(extent={{-16,-8},{16,-40}})));
      inner Components.Environment environment(initOpt=DynTherM.Choices.InitOpt.fixedState)
        annotation (Placement(transformation(extent={{68,70},{102,104}})));
      Modelica.Blocks.Sources.TimeTable h(table=E9X_data.getRealArray2D(
            "h",
            443,
            2)) annotation (Placement(transformation(extent={{30,80},{50,100}})));
      Modelica.Blocks.Sources.TimeTable V_inf(table=E9X_data.getRealArray2D(
            "V_inf",
            443,
            2)) annotation (Placement(transformation(extent={{30,40},{50,60}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow Q_battery
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={0,48})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor T_battery
        annotation (Placement(transformation(extent={{20,2},{40,22}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow Q_TMS annotation (
          Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=180,
            origin={-30,12})));
      Modelica.Blocks.Continuous.LimPID PID_T(
        controllerType=Modelica.Blocks.Types.SimpleController.PID,
        k=100,
        Ti=0.1,
        yMax=0,
        yMin=-4e5,
        initType=Modelica.Blocks.Types.Init.InitialOutput,
        y_start=0,
        homotopyType=Modelica.Blocks.Types.LimiterHomotopy.UpperLimit)
        annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=90,
            origin={-50,-30})));
      Modelica.Blocks.Sources.Ramp T_target(
        height=15,
        duration=1620,
        offset=273.15 + 20,
        startTime=0) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-50,-70})));
    equation
      connect(P_bat.y, product1.u1) annotation (Line(points={{-79,90},{-70,90},{-70,
              76},{-62,76}}, color={0,0,127}));
      connect(const.y, product1.u2) annotation (Line(points={{-79,50},{-70,50},{-70,
              64},{-62,64}}, color={0,0,127}));
      connect(V_inf.y, environment.V_inf) annotation (Line(points={{51,50},{60,50},{
              60,73.4},{68,73.4}}, color={0,0,127}));
      connect(h.y, environment.altitude) annotation (Line(points={{51,90},{60,90},{60,
              80.2},{68,80.2}}, color={0,0,127}));
      connect(Q_battery.port, battery_pack.port) annotation (Line(points={{-1.77636e-15,
              38},{-1.77636e-15,15},{0,15},{0,-8}}, color={191,0,0}));
      connect(Q_battery.Q_flow, product1.y) annotation (Line(points={{1.77636e-15,58},
              {1.77636e-15,70},{-39,70}}, color={0,0,127}));
      connect(battery_pack.port, T_battery.port)
        annotation (Line(points={{0,-8},{0,12},{20,12}}, color={191,0,0}));
      connect(Q_TMS.port, battery_pack.port)
        annotation (Line(points={{-20,12},{0,12},{0,-8}}, color={191,0,0}));
      connect(PID_T.y, Q_TMS.Q_flow)
        annotation (Line(points={{-50,-19},{-50,12},{-40,12}}, color={0,0,127}));
      connect(T_battery.T, PID_T.u_m) annotation (Line(points={{41,12},{50,12},{50,-92},
              {-80,-92},{-80,-30},{-62,-30}}, color={0,0,127}));
      connect(T_target.y, PID_T.u_s)
        annotation (Line(points={{-50,-59},{-50,-42}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=8782.2, __Dymola_Algorithm="Dassl"));
    end IsolatedPackControlled;

    model IsolatedPackNoControl
      // Battery pack modeled as a single lumped mass
      // No heat transfer with external environment
      // Battery temperature is not actively controlled
      inner parameter ExternData.JSONFile E9X_data(fileName=
            ModelicaServices.ExternalReferences.loadResource("modelica://DynTherM/ExternalData/F9X/E9X_data.json"))
        "JSON file"
        annotation (Placement(transformation(extent={{60,-100},{80,-80}})));
      Modelica.Blocks.Sources.TimeTable P_bat(table=E9X_data.getRealArray2D(
            "P_bat",
            443,
            2))   annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
      Modelica.Blocks.Math.Product product1
        annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
      Modelica.Blocks.Sources.Constant const(k=1 - data_battery.eta_pack)
        annotation (Placement(transformation(extent={{-100,40},{-80,60}})));
      Battery.BatteryPackData data_battery(E_pack=0)
        annotation (Placement(transformation(extent={{82,-98},{98,-80}})));
      Components.HeatTransfer.HeatCapacity battery_pack(T_start=293.15, C=
            data_battery.m_tot*data_battery.c)
        annotation (Placement(transformation(extent={{-16,-8},{16,-40}})));
      inner Components.Environment environment(initOpt=DynTherM.Choices.InitOpt.fixedState)
        annotation (Placement(transformation(extent={{68,70},{102,104}})));
      Modelica.Blocks.Sources.TimeTable h(table=E9X_data.getRealArray2D(
            "h",
            443,
            2)) annotation (Placement(transformation(extent={{30,80},{50,100}})));
      Modelica.Blocks.Sources.TimeTable V_inf(table=E9X_data.getRealArray2D(
            "V_inf",
            443,
            2)) annotation (Placement(transformation(extent={{30,40},{50,60}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow Q_battery
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={0,48})));
    equation
      connect(P_bat.y, product1.u1) annotation (Line(points={{-79,90},{-70,90},{-70,
              76},{-62,76}}, color={0,0,127}));
      connect(const.y, product1.u2) annotation (Line(points={{-79,50},{-70,50},{-70,
              64},{-62,64}}, color={0,0,127}));
      connect(V_inf.y, environment.V_inf) annotation (Line(points={{51,50},{60,50},{
              60,73.4},{68,73.4}}, color={0,0,127}));
      connect(h.y, environment.altitude) annotation (Line(points={{51,90},{60,90},{60,
              80.2},{68,80.2}}, color={0,0,127}));
      connect(Q_battery.port, battery_pack.port) annotation (Line(points={{-1.77636e-15,
              38},{-1.77636e-15,15},{0,15},{0,-8}}, color={191,0,0}));
      connect(Q_battery.Q_flow, product1.y) annotation (Line(points={{1.77636e-15,58},
              {1.77636e-15,70},{-39,70}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=8782.2, __Dymola_Algorithm="Dassl"));
    end IsolatedPackNoControl;

    model PackNoControl
      // Battery pack modeled as a single lumped mass
      // No heat transfer with external environment
      // Battery temperature is not actively controlled
      inner parameter ExternData.JSONFile E9X_data(fileName=
            ModelicaServices.ExternalReferences.loadResource("modelica://DynTherM/ExternalData/F9X/E9X_data.json"))
        "JSON file"
        annotation (Placement(transformation(extent={{60,-100},{80,-80}})));
      Modelica.Blocks.Sources.TimeTable P_bat(table=E9X_data.getRealArray2D(
            "P_bat",
            443,
            2))   annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
      Modelica.Blocks.Math.Product product1
        annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
      Modelica.Blocks.Sources.Constant const(k=1 - data_battery.eta_pack)
        annotation (Placement(transformation(extent={{-100,40},{-80,60}})));
      Battery.BatteryPackData data_battery(E_pack=0)
        annotation (Placement(transformation(extent={{82,-98},{98,-80}})));
      Components.HeatTransfer.HeatCapacity battery_pack(T_start=293.15, C=
            data_battery.m_tot*data_battery.c)
        annotation (Placement(transformation(extent={{-16,-8},{16,-40}})));
      inner Components.Environment environment(initOpt=DynTherM.Choices.InitOpt.fixedState)
        annotation (Placement(transformation(extent={{68,70},{102,104}})));
      Modelica.Blocks.Sources.TimeTable h(table=E9X_data.getRealArray2D(
            "h",
            443,
            2)) annotation (Placement(transformation(extent={{30,80},{50,100}})));
      Modelica.Blocks.Sources.TimeTable V_inf(table=E9X_data.getRealArray2D(
            "V_inf",
            443,
            2)) annotation (Placement(transformation(extent={{30,40},{50,60}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow Q_battery
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={0,48})));
    equation
      connect(P_bat.y, product1.u1) annotation (Line(points={{-79,90},{-70,90},{-70,
              76},{-62,76}}, color={0,0,127}));
      connect(const.y, product1.u2) annotation (Line(points={{-79,50},{-70,50},{-70,
              64},{-62,64}}, color={0,0,127}));
      connect(V_inf.y, environment.V_inf) annotation (Line(points={{51,50},{60,50},{
              60,73.4},{68,73.4}}, color={0,0,127}));
      connect(h.y, environment.altitude) annotation (Line(points={{51,90},{60,90},{60,
              80.2},{68,80.2}}, color={0,0,127}));
      connect(Q_battery.port, battery_pack.port) annotation (Line(points={{-1.77636e-15,
              38},{-1.77636e-15,15},{0,15},{0,-8}}, color={191,0,0}));
      connect(Q_battery.Q_flow, product1.y) annotation (Line(points={{1.77636e-15,58},
              {1.77636e-15,70},{-39,70}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=8782.2, __Dymola_Algorithm="Dassl"));
    end PackNoControl;
  end BatteryPack;
end E9X;
