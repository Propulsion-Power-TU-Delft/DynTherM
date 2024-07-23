within DynTherM.Components;
package TwoDimensional "Package collecting the components modeling coupled heat and mass transfer and featuring a 2D spatial discretization"

  model WallConduction2D "Dynamic model of conduction in a planar surface"
    replaceable model Mat=Materials.Aluminium constrainedby
      Materials.Properties "Material choice" annotation (choicesAllMatching=true);

    input Real N=1 "Number of walls in parallel" annotation (Dialog(enable=true));
    input Length w "Width of the Plane, sides facing east and west"  annotation (Dialog(enable=true));
    input Length l "Length of the plane,sides facing north and south" annotation (Dialog(enable=true));
    input Length dz "Thickness of the slice, out of plane" annotation (Dialog(enable=true));

    // Initialization
    parameter Temperature Tstart=300
      "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
      "Initialization option" annotation (Dialog(tab="Initialization"));

    Mass m "Mass of the volume";
    Modelica.Units.SI.HeatCapacity Cm "Heat capacity of the volume";
    Temperature T_vol "Average temperature of the volume";

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inletN annotation (
        Placement(transformation(extent={{-10,60},{12,82}}), iconTransformation(
            extent={{-10,60},{12,82}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outletW
      annotation (Placement(transformation(extent={{-100,-10},{-80,10}}),
          iconTransformation(extent={{-100,-10},{-80,10}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outletS
      annotation (Placement(transformation(extent={{-12,-82},{10,-60}}),
          iconTransformation(extent={{-12,-82},{10,-60}})));

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inletE
      annotation (Placement(transformation(extent={{80,-10},{100,10}}),
          iconTransformation(extent={{80,-10},{100,10}})));

  equation

    m=Mat.rho*w*l*dz;
    Cm=m*Mat.cm;

    N*Cm*der(T_vol) =inletN.Q_flow +outletS.Q_flow  +inletE.Q_flow  +outletW.Q_flow         "Energy balance";

    inletN.Q_flow = (Mat.lambda*N*w*dz*(inletN.T - T_vol))/(l/2)
      "Heat conduction through north side";
    outletS.Q_flow = (Mat.lambda*N*w*dz*(outletS.T - T_vol))/(l/2)
      "Heat conduction through the southern side";
    inletE.Q_flow = (Mat.lambda*N*l*dz*(inletE.T - T_vol))/(w/2)
      "Heat conduction through the eastern side";
    outletW.Q_flow = (Mat.lambda*N*l*dz*(outletW.T - T_vol))/(w/2)
      "Heat conduction through the Western side";

  initial equation
    if initOpt == Choices.InitOpt.steadyState then
      der(T_vol) = 0;
    elseif initOpt == Choices.InitOpt.fixedState then
      T_vol = Tstart;
    else
      // do nothing
    end if;
    annotation (
      Icon(graphics={   Rectangle(
            extent={{-80,60},{80,-60}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
          Text(
            extent={{-32,20},{32,-44}},
            lineColor={255,255,255},
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="Plane
"),       Line(
            points={{-2,-34},{-2,36}},
            color={0,0,0},
            arrow={Arrow.None,Arrow.Filled},
            thickness=0.5)}),
      Documentation(info="<html>
<p>The heat capacity (which is lumped at the center of the wall thickness) is accounted for, as well as the thermal resistance due to the finite heat conduction coefficient. Longitudinal heat conduction is neglected. </p>
<p>The model can be used to reproduce the heat transfer through many walls in parallel. In that case, the heat flow rate is split equally among the different elements, assuming there is no heat transfer and temperature difference between them.</p>
<p>Model adapted from ThermoPower library by Francesco Casella.</p>
</html>", revisions="<html>
</html>"));
  end WallConduction2D;

  model ConductionPlanoConcave2D
    "Dynamic model of conduction heat transfer through a Plano-Concave surface"

    replaceable model Mat=Materials.Aluminium constrainedby
      Materials.Properties "Material choice" annotation (choicesAllMatching=true);

    input Real N = 1;
    input Length R "Radius of curvature of concave side" annotation (Dialog(enable=true));
    input Length p "Dimension of parallel sides" annotation (Dialog(enable=true));
    input Length np "Dimension non-curved non-parallel side" annotation (Dialog(enable=true));
    input Length dz "Thickness of the slice, out of plane" annotation (Dialog(enable=true));

    // Initialization
    parameter Temperature Tstart=300
      "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
      "Initialization option" annotation (Dialog(tab="Initialization"));

    Length t_avg "Equivalent thickness if it was a rectangle";
    Area As "Plane area";
    Angle theeta "Angle subtended by the curved side";
    Angle beta "Adjacent angle of theeta in the right angle triangle";

    Mass m "Mass of the wall";
    Modelica.Units.SI.HeatCapacity Cm "Heat capacity of the wall";
    Temperature T_vol "Average temperature of the wall";

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a inletPlanar
      "Thermal port for the planar side" annotation (Placement(transformation(
            extent={{-10,80},{10,100}}),iconTransformation(extent={{-10,80},{10,100}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b OutletOppoCon
      "Thermal port for the side opposite to concave side" annotation (Placement(
          transformation(extent={{-60,-10},{-40,10}}),  iconTransformation(extent={{-60,-10},
              {-40,10}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b outletPlanar
      "Thermal port for the planar side" annotation (Placement(transformation(extent={{-10,
              -100},{10,-80}}), iconTransformation(extent={{-10,-100},{10,-80}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a InletConcave
      "Thermal port for the concave side" annotation (Placement(transformation(
            extent={{12,-10},{32,10}}),  iconTransformation(extent={{12,-10},{32,10}})));

  equation
  //  theta = acos((np*np - 2*R*R) / (2*R*R))      "Law of cosines";
    np*np = 2*R*R + 2*R*R*cos(theeta)              "Law of cosines";
    theeta + 2*beta = pi                           "Sum of angles on one side of straight line";
    As =  np * ( p + R*sin(beta)) - (theeta*R*R/2) - R*R*sin(beta)*cos(beta)  "Using geometry";
    t_avg = As/np                                  "Using geometry";

    m=Mat.rho*As*dz;
    Cm=m*Mat.cm;

    N*Cm*der(T_vol) =inletPlanar.Q_flow + outletPlanar.Q_flow + InletConcave.Q_flow
       + OutletOppoCon.Q_flow                                                         "Energy balance";
    inletPlanar.Q_flow = (Mat.lambda*N*dz*p)*(inletPlanar.T - T_vol)/(np/2)
      "Heat conduction through the upper half of planar side";
    outletPlanar.Q_flow = (Mat.lambda*N*dz*p)*(outletPlanar.T - T_vol)/(np/2)
      "Heat conduction through the lower half of planar side";
    InletConcave.Q_flow = (Mat.lambda*N*dz*np)*(InletConcave.T - T_vol)/(t_avg/2)
      "Heat conduction through the concave side";
    OutletOppoCon.Q_flow = (Mat.lambda*N*dz*np)*(OutletOppoCon.T - T_vol)/(t_avg/2)
      "Heat conduction through planar side opposite to the concave";

  initial equation
    if initOpt == Choices.InitOpt.steadyState then
      der(T_vol) = 0;
    elseif initOpt == Choices.InitOpt.fixedState then
      T_vol = Tstart;
    else
      // do nothing
    end if;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Polygon(
            points={{40,80},{32,60},{26,40},{22,22},{20,0},{22,-20},{26,-40},{34,-60},
                {42,-80},{40,-80},{-40,-80},{-40,80},{40,80}},
            lineColor={28,108,200},
            fillColor={215,215,215},
            fillPattern=FillPattern.Backward)}),                   Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end ConductionPlanoConcave2D;

  model WallConductionDiscretized
    "Dynamic model of conduction in a planar surface implementing 1D discretization"

    replaceable model Mat = Materials.Aluminium constrainedby
      Materials.Properties "Material choice" annotation (choicesAllMatching=true);
    model CV = Components.HeatTransfer.WallConduction "Control volume";

    // Geometry
    input Length t "Wall thickness" annotation (Dialog(enable=true));
    input Area A "Wall surface" annotation (Dialog(enable=true));

    // Initialization
    parameter Temperature Tstart "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter DynTherM.Choices.InitOpt initOpt=Choices.InitOpt.fixedState
      "Initialization option" annotation (Dialog(tab="Initialization"));

    // Discretization
    parameter Integer Nx(min=1) "Number of wall sections in x-direction";
    parameter Integer Ny(min=1) "Number of wall sections in y-direction";

    CV cv[Nx,Ny](
      redeclare model Mat=Mat,
      each t=t,
      each A=A/(Nx*Ny),
      each Tstart=Tstart,
      each initOpt=initOpt);

    CustomInterfaces.DistributedHeatPort_B outlet(Nx=Nx, Ny=Ny)
                                                              annotation (
        Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={20,3.55271e-15}),iconTransformation(
          extent={{-60,-48},{60,48}},
          rotation=180,
          origin={7.10543e-15,-30})));
    CustomInterfaces.DistributedHeatPort_A inlet(Nx=Nx, Ny=Ny)
                                                             annotation (
        Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={-20,3.55271e-15}),
                                   iconTransformation(
          extent={{-60,-48},{60,48}},
          rotation=180,
          origin={7.10543e-15,30})));

  equation
    for i in 1:Nx loop
      for j in 1:Ny loop
        connect(outlet.ports[i,j], cv[i,j].outlet);
        connect(inlet.ports[i,j], cv[i,j].inlet);
      end for;
    end for;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                        Rectangle(
            extent={{-100,20},{98,-20}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
          Line(
            points={{2,-42},{-38,-42}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={-22,18},
            rotation=90),
          Line(
            points={{-20,44},{-60,44}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={24,40},
            rotation=90),
          Line(
            points={{-40,4},{-2.28848e-17,4}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={-56,20},
            rotation=90),
          Text(
            extent={{-32,32},{32,-32}},
            lineColor={255,255,255},
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="WALL"),
          Line(
            points={{-40,4},{-2.28848e-17,4}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={64,20},
            rotation=90)}),                      Diagram(coordinateSystem(
            preserveAspectRatio=false)),
                Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>Heat transfer is modelled only in transversal direction, i.e., through the wall thickness.</p>
<p>Heat transfer among adjacent wall control volumes is neglected.</p>
</html>"));
  end WallConductionDiscretized;

  model WallConductionDiscretized2D
    "Dynamic model of conduction in a planar surface implementing 1D discretization in vertical direction"

    replaceable model Mat = Materials.Aluminium constrainedby
      Materials.Properties "Material choice" annotation (choicesAllMatching=true);
    model CV = DynTherM.Components.TwoDimensional.WallConduction2D
                                                                 "Control volume";

    // Geometry
    input Length t "Wall thickness" annotation (Dialog(enable=true));
    input Length h "Wall height, dimension perpendicular to thickness along heat transfer" annotation (Dialog(enable=true));
    input Area A "Wall surface" annotation (Dialog(enable=true));

    // Initialization
    parameter Temperature Tstart "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter DynTherM.Choices.InitOpt initOpt=Choices.InitOpt.fixedState
      "Initialization option" annotation (Dialog(tab="Initialization"));

    // Discretization
    parameter Integer N(min=1) "Number of vertical sections in which the wall is discretized";

    CV cv[N](
      redeclare model Mat=Mat,
      each t=t,
      each h=h,
      each A=A/N,
      each Tstart=Tstart,
      each initOpt=initOpt);

    CustomInterfaces.DistributedHeatPort_B outlet(Nx=N, Ny=1) annotation (
        Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={20,3.55271e-15}),iconTransformation(
          extent={{-60,-48},{60,48}},
          rotation=180,
          origin={7.10543e-15,-30})));
    CustomInterfaces.DistributedHeatPort_A inlet(Nx=N, Ny=1) annotation (
        Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={-20,3.55271e-15}),
                                   iconTransformation(
          extent={{-60,-48},{60,48}},
          rotation=180,
          origin={7.10543e-15,30})));

    CustomInterfaces.DistributedHeatPort_B outletH(Nx=N, Ny=1)
                                                              annotation (
        Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={20,3.55271e-15}),iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-96,0})));
    CustomInterfaces.DistributedHeatPort_A inletH(Nx=N, Ny=1)
                                                             annotation (
        Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={-20,3.55271e-15}),
                                   iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={96,0})));
  equation
    for i in 1:N loop
      connect(outlet.ports[i,1], cv[i].outlet);
      connect(inlet.ports[i,1], cv[i].inlet);
    end for;
    for i in 1:N loop
      connect(outletH.ports[i,1], cv[i].outletH);
      connect(inletH.ports[i,1], cv[i].inletH);
    end for;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                        Rectangle(
            extent={{-100,20},{98,-20}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
          Line(
            points={{2,-42},{-38,-42}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={-22,18},
            rotation=90),
          Line(
            points={{-20,44},{-60,44}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={24,40},
            rotation=90),
          Line(
            points={{-40,4},{-2.28848e-17,4}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={-56,20},
            rotation=90),
          Text(
            extent={{-32,32},{32,-32}},
            lineColor={255,255,255},
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="WALL"),
          Line(
            points={{-40,4},{-2.28848e-17,4}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={64,20},
            rotation=90)}),                      Diagram(coordinateSystem(
            preserveAspectRatio=false)),
                Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>Heat transfer is modelled only in transversal direction, i.e., through the wall thickness.</p>
<p>Heat transfer in vertical direction, i.e., among adjacent wall control volumes, is neglected.</p>
</html>"));
  end WallConductionDiscretized2D;

  model ColdPlateCircularChannelCV
    "Control volume for modelling of heat transfer through a portion of circular channel in a cold plate"

    replaceable model Mat = Materials.Aluminium constrainedby
      Materials.Properties "Material choice" annotation (choicesAllMatching=true);
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

    // Options
    parameter Boolean allowFlowReversal=true
      "= true to allow flow reversal, false restricts to design direction";
    parameter Choices.PDropOpt DP_opt
      "Select the type of pressure drop to impose";
    parameter Choices.InitOpt initOpt
      "Initialization option" annotation (Dialog(tab="Initialization"));

    // Geometry
    parameter Integer N=1 "Number of control volumes in parallel";
    parameter Length L "Length of the control volume, in the flow direction" annotation (Dialog(tab="Geometry"));
    parameter Length t "Thickness of the cold Plate" annotation (Dialog(tab="Geometry"));
    parameter Length d "Center to center distance between the parallel pipes" annotation (Dialog(tab="Geometry"));
    parameter Length R_int "Internal radius of the pipe control volume" annotation (Dialog(tab="Geometry"));
    parameter Length Roughness=0.015*10^(-3) "Pipe roughness" annotation (Dialog(tab="Geometry"));

  // Initialization
    parameter Temperature T_start_solid=288.15
      "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
    parameter Temperature T_start_fluid=288.15
      "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
    parameter Pressure P_start=101325
      "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
    parameter MassFraction X_start[Medium.nX]=Medium.reference_X
      "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
    parameter Medium.ThermodynamicState state_start=
        Medium.setState_pTX(P_start, T_start_fluid, X_start)
      "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
    parameter MassFlowRate m_flow_start=1
      "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
   parameter Density rho_start=1 "Density - start value" annotation (Dialog(tab="Initialization"));
    parameter Velocity u_start=20 "Flow velocity - start value" annotation (Dialog(tab="Initialization"));
    parameter Pressure dP_start=100 "Pressure drop - start value" annotation (Dialog(tab="Initialization"));
    parameter ReynoldsNumber Re_start=20e3 "Reynolds number - start value" annotation (Dialog(tab="Initialization"));
    parameter PrandtlNumber Pr_start=1.5 "Prandtl number - start value" annotation (Dialog(tab="Initialization"));

    MassTransfer.CircularPipe circularPipe(
      redeclare package Medium = Medium,
      allowFlowReversal=allowFlowReversal,
      DP_opt=DP_opt,
      m_flow_start=m_flow_start,
      P_start=P_start,
      T_start=T_start_fluid,
      X_start=X_start,
      u_start=u_start,
      rho_start=rho_start,
      dP_start=dP_start,
      state_start=state_start,
      Re_start=Re_start,
      Pr_start=Pr_start,
      N=N,
      L=L,
      D=R_int*2,
      Roughness=Roughness)
      annotation (Placement(transformation(extent={{-40,-44},{40,36}})));

    CustomInterfaces.FluidPort_A          inlet(
      redeclare package Medium = Medium,
      m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
            m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-52,-10},
              {-40,2}},       rotation=0), iconTransformation(extent={{-40,-8},{-26,
              6}})));
    CustomInterfaces.FluidPort_B          outlet(
      redeclare package Medium = Medium,
      m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=-
            m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{40,-10},
              {52,2}},        rotation=0), iconTransformation(extent={{24,-8},{40,
              8}})));
    ConductionPlanoConcave2D PCWest(
      R=R_int,
      p=d/2 - R_int*d/sqrt(d*d + t*t),
      np=2*R_int*t/sqrt(d*d + t*t),
      dz=L,
      Tstart=T_start_solid,
      redeclare model Mat = Mat,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{-88,-8},{-44,48}})));
    ConductionPlanoConcave2D PCEast(
      R=R_int,
      p=d/2 - R_int*d/sqrt(d*d + t*t),
      np=2*R_int*t/sqrt(d*d + t*t),
      redeclare model Mat = Mat,
      dz=L,
      Tstart=T_start_solid,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{88,-8},{44,48}})));
    ConductionPlanoConcave2D PCSouth(
      R=R_int,
      p=t/2 - R_int*t/sqrt(d*d + t*t),
      np=2*R_int*d/sqrt(d*d + t*t),
      dz=L,
      Tstart=T_start_solid,
      redeclare model Mat = Mat,
      initOpt=initOpt) annotation (Placement(transformation(
          extent={{22,-28},{-22,28}},
          rotation=-90,
          origin={0,-42})));
    ConductionPlanoConcave2D PCNorth(
      R=R_int,
      p=t/2 - R_int*t/sqrt(d*d + t*t),
      np=2*R_int*d/sqrt(d*d + t*t),
      dz=L,
      Tstart=T_start_solid,
      redeclare model Mat = Mat,
      initOpt=initOpt) annotation (Placement(transformation(
          extent={{-22,-28},{22,28}},
          rotation=-90,
          origin={0,64})));
    WallConduction2D PlaneNW(
      w=t/2 - R_int*t/sqrt(d*d + t*t),
      l=d/2 - R_int*d/sqrt(d*d + t*t),
      dz=L,
      Tstart=T_start_solid,
      redeclare model Mat = Mat,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{-82,50},{-52,80}})));
    WallConduction2D PlaneSW(
      w=t/2 - R_int*t/sqrt(d*d + t*t),
      l=d/2 - R_int*d/sqrt(d*d + t*t),
      dz=L,
      Tstart=T_start_solid,
      redeclare model Mat = Mat,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{-82,-56},{-52,-26}})));
    WallConduction2D PlaneSE(
      w=t/2 - R_int*t/sqrt(d*d + t*t),
      l=d/2 - R_int*d/sqrt(d*d + t*t),
      dz=L,
      Tstart=T_start_solid,
      redeclare model Mat = Mat,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{52,-58},{82,-28}})));
    WallConduction2D PlaneNE(
      w=t/2 - R_int*t/sqrt(d*d + t*t),
      l=d/2 - R_int*d/sqrt(d*d + t*t),
      dz=L,
      Tstart=T_start_solid,
      redeclare model Mat = Mat,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{50,50},{80,80}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a NorthWestHor annotation (
        Placement(transformation(extent={{-106,56},{-86,76}}), iconTransformation(
            extent={{-72,40},{-60,52}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a West annotation (
        Placement(transformation(extent={{-106,10},{-86,30}}), iconTransformation(
            extent={{-72,-6},{-60,6}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a SouthWestHor annotation (
        Placement(transformation(extent={{-106,-50},{-86,-30}}),
          iconTransformation(extent={{-72,-50},{-60,-38}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a SouthBottom annotation (
        Placement(transformation(extent={{-10,-90},{10,-70}}), iconTransformation(
            extent={{-6,-72},{6,-60}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b SouthEastHor annotation (
        Placement(transformation(extent={{86,-52},{106,-32}}), iconTransformation(
            extent={{60,-50},{72,-38}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a East annotation (
        Placement(transformation(extent={{86,10},{106,30}}), iconTransformation(
            extent={{60,-6},{72,6}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b NorthEastHor annotation (
        Placement(transformation(extent={{86,56},{106,76}}), iconTransformation(
            extent={{60,38},{72,50}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a NorthTop annotation (
        Placement(transformation(extent={{-10,86},{10,106}}), iconTransformation(
            extent={{-6,60},{6,72}})));
  equation
    connect(inlet, circularPipe.inlet)
      annotation (Line(points={{-46,-4},{-40,-4}},
                                                 color={0,0,0}));
    connect(circularPipe.outlet, outlet)
      annotation (Line(points={{40,-4},{46,-4}},
                                               color={0,0,0}));
    connect(PCWest.InletConcave, circularPipe.thermalPort)
      annotation (Line(points={{-61.16,20},{0,20},{0,11.2}},  color={191,0,0}));
    connect(PCNorth.InletConcave, circularPipe.thermalPort)
      annotation (Line(points={{0,59.16},{0,11.2}},  color={191,0,0}));
    connect(PCEast.InletConcave, circularPipe.thermalPort)
      annotation (Line(points={{61.16,20},{0,20},{0,11.2}},  color={191,0,0}));
    connect(PCSouth.InletConcave, circularPipe.thermalPort)
      annotation (Line(points={{0,-37.16},{0,11.2}},  color={191,0,0}));
    connect(PCWest.inletPlanar, PlaneNW.outletS) annotation (Line(points={{-66,45.2},
            {-66,46},{-67.15,46},{-67.15,54.35}}, color={191,0,0}));
    connect(PlaneNW.inletE, PCNorth.outletPlanar) annotation (Line(points={{-53.5,
            65},{-34,65},{-34,64},{-25.2,64}}, color={191,0,0}));
    connect(PlaneSW.inletE, PCSouth.outletPlanar) annotation (Line(points={{-53.5,
            -41},{-53.5,-42},{-25.2,-42}}, color={191,0,0}));
    connect(PlaneSW.inletN, PCWest.outletPlanar) annotation (Line(points={{-66.85,
            -30.35},{-66,-28},{-66,-5.2}}, color={191,0,0}));
    connect(PCSouth.inletPlanar, PlaneSE.outletW) annotation (Line(points={{25.2,-42},
            {48,-42},{48,-43},{53.5,-43}}, color={191,0,0}));
    connect(PlaneSE.inletN, PCEast.outletPlanar) annotation (Line(points={{67.15,-32.35},
            {67.15,-14},{66,-14},{66,-5.2}}, color={191,0,0}));
    connect(PCEast.inletPlanar, PlaneNE.outletS) annotation (Line(points={{66,45.2},
            {66,46},{64.85,46},{64.85,54.35}}, color={191,0,0}));
    connect(PCNorth.inletPlanar, PlaneNE.outletW) annotation (Line(points={{25.2,64},
            {46,64},{46,65},{51.5,65}}, color={191,0,0}));
    connect(PlaneNW.outletW, NorthWestHor) annotation (Line(points={{-80.5,65},{-80.5,
            66},{-96,66}}, color={191,0,0}));
    connect(PlaneSW.outletW, SouthWestHor) annotation (Line(points={{-80.5,-41},{-80.5,
            -40},{-96,-40}}, color={191,0,0}));
    connect(PCSouth.OutletOppoCon, SouthBottom)
      annotation (Line(points={{0,-53},{0,-80}}, color={191,0,0}));
    connect(PlaneSE.inletE, SouthEastHor) annotation (Line(points={{80.5,-43},{80.5,
            -42},{96,-42}}, color={191,0,0}));
    connect(PCEast.OutletOppoCon, East)
      annotation (Line(points={{77,20},{96,20}}, color={191,0,0}));
    connect(West, PCWest.OutletOppoCon)
      annotation (Line(points={{-96,20},{-77,20}}, color={191,0,0}));
    connect(PlaneNE.inletE, NorthEastHor)
      annotation (Line(points={{78.5,65},{80,66},{96,66}}, color={191,0,0}));
    connect(PCNorth.OutletOppoCon, NorthTop)
      annotation (Line(points={{0,75},{0,96}}, color={191,0,0}));
    connect(PlaneNW.inletN, NorthTop) annotation (Line(points={{-66.85,75.65},{
            -68,75.65},{-68,84},{0,84},{0,96}}, color={191,0,0}));
    connect(PlaneNE.inletN, NorthTop) annotation (Line(points={{65.15,75.65},{
            65.15,84},{0,84},{0,96}}, color={191,0,0}));
    connect(PlaneSW.outletS, SouthBottom) annotation (Line(points={{-67.15,-51.65},
            {-67.15,-66},{0,-66},{0,-80}}, color={191,0,0}));
    connect(PlaneSE.outletS, SouthBottom) annotation (Line(points={{66.85,-53.65},
            {66.85,-66},{0,-66},{0,-80}}, color={191,0,0}));
      annotation (Line(points={{46,-4},{46,-4}}, color={0,0,0}),
                  Placement(transformation(extent={{-28,-28},{28,28}})),
                Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Ellipse(
            extent={{40,40},{-40,-40}},
            lineColor={0,0,0},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-28,-28},{-60,-28},{-60,30},{-26,30},{-34,22},{-38,12},{-40,2},
                {-40,-6},{-38,-14},{-34,-22},{-28,-28}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{28,-28},{60,-28},{60,30},{26,30},{34,22},{38,12},{40,2},{40,-6},
                {38,-14},{34,-22},{28,-28}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-15,-29},{17,-29},{17,29},{-17,29},{-9,21},{-5,11},{-3,1},{-3,
                -7},{-5,-15},{-9,-23},{-15,-29}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            origin={-1,43},
            rotation=90),
          Polygon(
            points={{29,-15},{29,17},{-29,17},{-29,-17},{-21,-9},{-11,-5},{-1,-3},
                {7,-3},{15,-5},{23,-9},{29,-15}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            origin={1,-43},
            rotation=180),
          Rectangle(
            extent={{-60,60},{-30,26}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-60,-28},{-28,-60}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{30,-26},{60,-60}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{28,60},{60,28}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end ColdPlateCircularChannelCV;

  model ColdPlateCircularChannel1D
    "Circular channel in a cold plate with 1D spatial discretization"

    replaceable model Mat = Materials.Aluminium constrainedby
      Materials.Properties "Material choice" annotation (choicesAllMatching=true);
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

    model CV = TwoDimensional.ColdPlateCircularChannelCV
                                          "Control volume";
    model I = DynTherM.Components.MassTransfer.PlenumSimple
      "Inertia between two adjacent control volumes";

    // Options
    parameter Boolean allowFlowReversal=true
      "= true to allow flow reversal, false restricts to design direction";
   parameter Choices.PDropOpt DP_opt
      "Select the type of pressure drop to impose";
    parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
      "Initialization option" annotation (Dialog(tab="Initialization"));

    // Geometry
    parameter Length L "Channel length" annotation (Dialog(tab="Geometry"));
    parameter Length t "Thickness of the cold Plate" annotation (Dialog(tab="Geometry"));
    parameter Length d "Center to center distance between the Channels"
                                                                       annotation (Dialog(tab="Geometry"));
    parameter Length R_int "Channel internal radius" annotation (Dialog(tab="Geometry"));
    parameter Length Roughness=0.015*10^(-3) "Channel roughness" annotation (Dialog(tab="Geometry"));
    parameter Volume V_inertia=1e-10 "Volume of the plenum placed between two consecutive control volumes" annotation (Dialog(tab="Geometry"));

    // Initialization
    parameter Temperature T_start_solid=288.15 "Temperature of the solid part - start value" annotation (Dialog(tab="Initialization"));
    parameter Temperature T_start_fluid=288.15 "Fluid temperature - start value" annotation (Dialog(tab="Initialization"));
    parameter Pressure P_start=101325 "Fluid pressure - start value" annotation (Dialog(tab="Initialization"));
    parameter MassFraction X_start[Medium.nX]=Medium.reference_X
      "Mass fractions - start value" annotation (Dialog(tab="Initialization"));
    parameter Medium.ThermodynamicState state_start = Medium.setState_pTX(P_start, T_start_fluid, X_start)
      "Starting thermodynamic state" annotation (Dialog(tab="Initialization"));
    parameter MassFlowRate m_flow_start=1
      "Mass flow rate - start value" annotation (Dialog(tab="Initialization"));
    parameter Density rho_start=1 "Density - start value" annotation (Dialog(tab="Initialization"));
    parameter Velocity u_start=20 "Flow velocity - start value" annotation (Dialog(tab="Initialization"));
    parameter Pressure dP_start=100 "Pressure drop - start value" annotation (Dialog(tab="Initialization"));
    parameter ReynoldsNumber Re_start=20e3 "Reynolds number - start value" annotation (Dialog(tab="Initialization"));
    parameter PrandtlNumber Pr_start=1.5 "Prandtl number - start value" annotation (Dialog(tab="Initialization"));

    // Discretization
    parameter Integer N_cv(min=1) "Number of control volumes in which the cooling channels are discretized";
    parameter Integer Nt=3  "Number of control volumes across the thickness of the cooling plate";
    parameter Integer N_channels(min=1) "Number of channels";

    CV cv[N_cv](
      redeclare model Mat = Mat,
      redeclare package Medium = Medium,
      each N=N_channels,
      each L=L/N_cv,
      each t=t,
      each d=d,
      each R_int=R_int,
      each Roughness=Roughness,
      each T_start_solid=T_start_solid,
      each T_start_fluid=T_start_fluid,
      each P_start=P_start,
      each X_start=X_start,
      each state_start=state_start,
      each m_flow_start=m_flow_start,
      each u_start=u_start,
      each rho_start=rho_start,
      each dP_start=dP_start,
      each Re_start=Re_start,
      each Pr_start=Pr_start,
      each initOpt=initOpt,
      each DP_opt=DP_opt,
      each allowFlowReversal=allowFlowReversal);

    I inertia[N_cv-1](
      redeclare package Medium = Medium,
      each V=V_inertia,
      each P_start=P_start,
      each T_start=T_start_fluid,
      each X_start=X_start,
      each state_start=state_start,
      each m_flow_start=m_flow_start,
      each noInitialPressure=true,
      each noInitialTemperature=false,
      each initOpt=initOpt,
      each allowFlowReversal=allowFlowReversal);

    DynTherM.CustomInterfaces.FluidPort_A inlet(
      redeclare package Medium = Medium,
      m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
            m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-106,-6},
              {-94,6}},       rotation=0), iconTransformation(extent={{-110,-10},{
              -90,10}})));
    DynTherM.CustomInterfaces.FluidPort_B outlet(
      redeclare package Medium = Medium,
      m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=
            -m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{94,-6},
              {106,6}},       rotation=0), iconTransformation(extent={{90,-10},{110,
              10}})));

    CustomInterfaces.DistributedHeatPort_B TopSurface(Nx=N_cv, Ny=1) annotation (
        Placement(transformation(extent={{8,14},{88,80}}), iconTransformation(
            extent={{8,14},{88,80}})));

    CustomInterfaces.DistributedHeatPort_B BottomSurface(Nx=N_cv, Ny=1)
      annotation (Placement(transformation(extent={{8,-80},{88,-14}}),
          iconTransformation(extent={{8,-80},{88,-14}})));
    CustomInterfaces.DistributedHeatPort_A EastSide(Nx=N_cv, Ny=Nt)
      annotation (
        Placement(transformation(extent={{-82,-82},{-12,-12}}),
          iconTransformation(extent={{-82,-82},{-12,-12}})));
    CustomInterfaces.DistributedHeatPort_A WestSide(Nx=N_cv, Ny=Nt)
      annotation (
        Placement(transformation(extent={{-82,12},{-12,82}}), iconTransformation(
            extent={{-84,12},{-14,82}})));
  equation

    // thermal connections (Top,N and Bottom,S)
    for i in 1:N_cv loop
      connect(TopSurface.ports[i, 1], cv[i].NorthTop);
      connect(BottomSurface.ports[i, 1], cv[i].SouthBottom);
    end for;
    // thermal connections (Side ways, West side)
    for i in 1:N_cv loop
        connect(WestSide.ports[i,1], cv[i].NorthWestHor);
        connect(WestSide.ports[i,2], cv[i].West);
        connect(WestSide.ports[i,3], cv[i].SouthWestHor);
    end for;
    // thermal connections (Side ways, East side)
    for i in 1:N_cv loop
        connect(EastSide.ports[i,1], cv[i].NorthEastHor);
        connect(EastSide.ports[i,2], cv[i].East);
        connect(EastSide.ports[i,3], cv[i].SouthEastHor);
    end for;

    // internal flow connections
    for i in 1:(N_cv-1) loop
      connect(cv[i].outlet, inertia[i].inlet);
      connect(inertia[i].outlet, cv[i+1].inlet);
    end for;

    // boundary flow connections
    connect(inlet, cv[1].inlet);
    connect(outlet, cv[N_cv].outlet);

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                        Rectangle(
            extent={{-100,40},{100,20}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
                        Rectangle(
            extent={{-100,-20},{100,-40}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
          Rectangle(extent={{-100,20},{100,-20}}, lineColor={0,0,0}),
          Line(
            points={{-60,20},{-60,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{-20,20},{-20,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{20,20},{20,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{60,20},{60,-20}},
            color={0,0,0},
            pattern=LinePattern.Dash)}),         Diagram(coordinateSystem(
            preserveAspectRatio=false)),
      Documentation(info="<html>
<p><span style=\"font-family: Courier New;\">Model created by stacking CircularCV in series and adding SimplePlenum in between to improve solver robustness.</span></p>
</html>"));
  end ColdPlateCircularChannel1D;
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),      Text(
          extent={{-84,88},{92,-88}},
          lineColor={0,0,0},
          textString="2D")}));
end TwoDimensional;
