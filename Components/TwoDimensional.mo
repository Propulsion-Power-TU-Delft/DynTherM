within DynTherM.Components;
package TwoDimensional "Package collecting the components modeling coupled heat and mass transfer and featuring a 2D spatial discretization"

  model WallConductionCV2D "Control volume dynamic model of 2D heat conduction in a planar surface"
    replaceable model MatX=Materials.Aluminium constrainedby
      Materials.Properties
      "Material properties along x direction" annotation (choicesAllMatching=true);
    replaceable model MatY=Materials.Aluminium constrainedby
      Materials.Properties
      "Material properties along y direction" annotation (choicesAllMatching=true);

    input Length x "Distance between east and west ports" annotation (Dialog(enable=true));
    input Length y "Distance between north and south ports" annotation (Dialog(enable=true));
    input Length z "Thickness, out of plane" annotation (Dialog(enable=true));
    input Mass dm(start=0) "Mass variation over time" annotation (Dialog(enable=true));

    // Initialization
    parameter Temperature Tstart=298.15 "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
      "Initialization option" annotation (Dialog(tab="Initialization"));

    Mass m "Mass";
    Volume V "Volume";
    HeatCapacity Cm "Heat capacity";
    Temperature T_vol "Average temperature";

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a North annotation (
        Placement(transformation(extent={{-10,60},{10,80}}), iconTransformation(
            extent={{-10,60},{10,80}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b West annotation (
        Placement(transformation(extent={{-100,-10},{-80,10}}),
          iconTransformation(extent={{-100,-10},{-80,10}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b South annotation (
        Placement(transformation(extent={{-10,-80},{10,-60}}), iconTransformation(
            extent={{-10,-80},{10,-60}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a East annotation (
        Placement(transformation(extent={{80,-10},{100,10}}), iconTransformation(
            extent={{80,-10},{100,10}})));
    Modelica.Blocks.Interfaces.RealInput Q_int annotation (Placement(
          transformation(extent={{-68,36},{-28,76}}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={-40,70})));

  equation
    V = x*y*z;
    m = MatX.rho*V - dm;
    Cm = MatX.cm*m;

    Cm*der(T_vol) = North.Q_flow + South.Q_flow + East.Q_flow + West.Q_flow +
      Q_int "Energy balance";

    North.Q_flow = (MatY.lambda*x*z*(North.T - T_vol))/(y/2) "Heat conduction through northern side";
    South.Q_flow = (MatY.lambda*x*z*(South.T - T_vol))/(y/2) "Heat conduction through the southern side";
    East.Q_flow = (MatX.lambda*y*z*(East.T - T_vol))/(x/2) "Heat conduction through the eastern side";
    West.Q_flow = (MatX.lambda*y*z*(West.T - T_vol))/(x/2) "Heat conduction through the western side";

    assert(MatX.rho == MatY.rho, "The density of the material must be equal in x and y direction");
    assert(MatX.cm == MatY.cm, "The specific heat capacity of the material must be equal in x and y direction");

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
          Ellipse(
            extent={{20,-22},{-24,22}},
            lineColor={0,0,0},
            fillColor={238,46,47},
            fillPattern=FillPattern.Sphere)}),
      Documentation(info="<html>
<p>Extension of the <i>WallConduction </i>model to calculate two-dimensional heat transfer through a planar surface.</p>
<p>The model accounts for anisotropic thermal conductivity in x and y directions. The remaining material properties must be isotropic.</p>
<p>The model accounts for internal heat generation within the control volume, if any.</p>
<p><br><img src=\"modelica://DynTherM/Figures/Wall Conduction 2D.png\"/></p>
</html>", revisions="<html>
</html>"));
  end WallConductionCV2D;

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
    parameter Temperature Tstart=298.15
      "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
      "Initialization option" annotation (Dialog(tab="Initialization"));

    Length t_avg "Equivalent thickness if it was a rectangle";
    Area As "Plane area";
    Angle theeta "Angle subtended by the curved side";
    Angle beta  "Adjacent angle of theeta in the right angle triangle";

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
    theeta  = acos((np*np - 2*R*R) / (2*R*R))      "Law of cosines";
  //  np*np = 2*R*R + 2*R*R*cos(theeta)              "Law of cosines";
    theeta + 2*beta = pi                          "Sum of angles on one side of straight line";
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
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>This model is further extension of <i>WallConduction2D </i>to calculate heat transfer through a plano-concave surface. </p>
<h4>Modelling assumptions: </h4>
<ul>
<li>The temperature of each side is assumed to be constant, denoted by nodal point T<sub>E</sub>, T<sub>W</sub>, T<sub>N</sub> and T<sub>s</sub>.</li>
<li>The heat capacity of surface of temperature T<sub>vol</sub> is lumped at a single point, located at the centroid of rectangle with the same surface area A<sub>s</sub> and height h.</li>
</ul>
<p><img src=\"modelica://DynTherM/Figures/Conduction_Plano_Concave.png\"/></p>
<h4>Equations</h4>
<p>The average thickness t<sub>avg, </sub>is the thickness of a rectangle having similar surface area A<sub>s</sub> and height h.</p>
<p>        <img src=\"modelica://DynTherM/Resources/Images/equations/equation-srCCxBop.png\" alt=\"t_avg = A_s/h\"/></p>
<p>        <img src=\"modelica://DynTherM/Resources/Images/equations/equation-YGItPSny.png\" alt=\"Q_E = k*h*dz*((T_E - T_vol)/(t_avg/2))\"/></p>
<p>        <img src=\"modelica://DynTherM/Resources/Images/equations/equation-qtcHdckv.png\" alt=\"Q_N = k*y*dz*((T_N - T_vol)/(h/2))\"/></p>
</html>"));
  end ConductionPlanoConcave2D;

  model WallConductionHorizontal2D "Dynamic model of 2D heat conduction in a planar surface discretised along the horizontal direction"

    replaceable model MatX=Materials.Aluminium constrainedby Materials.Properties
      "Material properties along x direction" annotation (choicesAllMatching=true);
    replaceable model MatY=Materials.Aluminium constrainedby Materials.Properties
      "Material properties along y direction" annotation (choicesAllMatching=true);
    model CV = DynTherM.Components.TwoDimensional.WallConductionCV2D
      "Control volume";

    // Geometry
    input Length x "Distance between east and west ports" annotation (Dialog(enable=true));
    input Length y "Distance between north and south ports" annotation (Dialog(enable=true));
    input Length z "Thickness, out of plane" annotation (Dialog(enable=true));

    // Initialization
    parameter Temperature Tstart=298.15 "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
      "Initialization option" annotation (Dialog(tab="Initialization"));

    // Discretization
    parameter Integer N(min=1) "Number of horizontal sections in which the wall is discretized";

    CV cv[N](
      redeclare model MatX=MatX,
      redeclare model MatY=MatY,
      each x=x/N,
      each y=y,
      each z=z,
      each dm=0,
      each Tstart=Tstart,
      each initOpt=initOpt);

    Mass m "Mass";
    Volume V "Volume";
    Area A "Main heat transfer area";

    CustomInterfaces.OneDimensional.HeatPort1D_B South(Nx=N) annotation (
        Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={20,3.55271e-15}), iconTransformation(
          extent={{-60,-48},{60,48}},
          rotation=180,
          origin={7.10543e-15,-30})));
    CustomInterfaces.OneDimensional.HeatPort1D_A North(Nx=N) annotation (
        Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={-20,3.55271e-15}), iconTransformation(
          extent={{-60,-48},{60,48}},
          rotation=180,
          origin={7.10543e-15,30})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b West annotation (
        Placement(transformation(extent={{-100,-10},{-80,10}}),
          iconTransformation(extent={{-100,-10},{-80,10}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a East annotation (
        Placement(transformation(extent={{80,-10},{100,10}}), iconTransformation(
            extent={{80,-10},{100,10}})));

  equation
    m = sum(cv.m);
    A = x*z;
    V = x*y*z;

    for i in 1:N loop
      cv[i].Q_int = 0;
    end for;

    // External connections
    connect(West, cv[1].West);
    connect(East, cv[end].East);

    for i in 1:N loop
      connect(South.ports[i], cv[i].South);
      connect(North.ports[i], cv[i].North);
    end for;

    // Internal connections
    for i in 1:(N - 1) loop
      connect(cv[i].East, cv[i + 1].West);
    end for;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                        Rectangle(
            extent={{-80,20},{80,-20}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
          Line(
            points={{2,-42},{-38,-42}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={-24,18},
            rotation=90),
          Line(
            points={{-20,44},{-60,44}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={26,40},
            rotation=90),
          Line(
            points={{-40,4},{-2.28848e-17,4}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={-46,20},
            rotation=90),
          Text(
            extent={{-30,32},{34,-32}},
            lineColor={255,255,255},
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="WALL"),
          Line(
            points={{-40,4},{0,4}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={54,20},
            rotation=90)}),                      Diagram(coordinateSystem(
            preserveAspectRatio=false)),
                Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>Heat transfer is modelled in the transversal, i.e., through the wall thickness, and horizontal, i.e., through adjacent control volumes, directions.</p>
<p>The model accounts for anisotropic thermal conductivity in x and y directions. The remaining material properties must be isotropic.</p>
</html>"));
  end WallConductionHorizontal2D;

  model WallConductionVertical2D "Dynamic model of 2D heat conduction in a planar surface discretised along the vertical direction"

    replaceable model MatX=Materials.Aluminium constrainedby Materials.Properties
      "Material properties along x direction" annotation (choicesAllMatching=true);
    replaceable model MatY=Materials.Aluminium constrainedby Materials.Properties
      "Material properties along y direction" annotation (choicesAllMatching=true);
    model CV = DynTherM.Components.TwoDimensional.WallConductionCV2D
      "Control volume";

    // Geometry
    input Length x "Distance between east and west ports" annotation (Dialog(enable=true));
    input Length y "Distance between north and south ports" annotation (Dialog(enable=true));
    input Length z "Thickness, out of plane" annotation (Dialog(enable=true));

    // Initialization
    parameter Temperature Tstart=298.15 "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter Choices.InitOpt initOpt=Choices.InitOpt.fixedState
      "Initialization option" annotation (Dialog(tab="Initialization"));

    // Discretization
    parameter Integer N(min=1) "Number of horizontal sections in which the wall is discretized";

    CV cv[N](
      redeclare model MatX=MatX,
      redeclare model MatY=MatY,
      each x=x,
      each y=y/N,
      each z=z,
      each dm=0,
      each Tstart=Tstart,
      each initOpt=initOpt);

    Mass m "Mass";
    Volume V "Volume";
    Area A "Main heat transfer area";

    CustomInterfaces.OneDimensional.HeatPort1D_B West(Nx=N) annotation (
        Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={20,3.55271e-15}), iconTransformation(
          extent={{-60,-48},{60,48}},
          rotation=90,
          origin={-30,0})));
    CustomInterfaces.OneDimensional.HeatPort1D_A East(Nx=N) annotation (
        Placement(transformation(
          extent={{-20,-10},{20,10}},
          rotation=-90,
          origin={-20,3.55271e-15}), iconTransformation(
          extent={{-60,-48},{60,48}},
          rotation=90,
          origin={30,0})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b South annotation (
        Placement(transformation(extent={{-10,-100},{10,-80}}),
          iconTransformation(extent={{-10,-100},{10,-80}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a North annotation (
        Placement(transformation(extent={{-10,80},{10,100}}), iconTransformation(
            extent={{-10,80},{10,100}})));

  equation
    m = sum(cv.m);
    A = y*z;
    V = x*y*z;

    for i in 1:N loop
      cv[i].Q_int = 0;
    end for;

    // External connections
    connect(South, cv[1].South);
    connect(North, cv[end].North);

    for i in 1:N loop
      connect(West.ports[i], cv[i].West);
      connect(East.ports[i], cv[i].East);
    end for;

    // Internal connections
    for i in 1:(N - 1) loop
      connect(cv[i].North, cv[i + 1].South);
    end for;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                        Rectangle(
            extent={{-20,80},{20,-80}},
            lineColor={0,0,0},
            fillColor={175,175,175},
            fillPattern=FillPattern.Backward),
          Line(
            points={{2,-42},{2,-82}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={-62,38},
            rotation=90),
          Line(
            points={{-20,44},{-20,4}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={24,20},
            rotation=90),
          Line(
            points={{0,-36},{0,4}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={-16,-40},
            rotation=90),
          Text(
            extent={{-32,32},{32,-32}},
            lineColor={255,255,255},
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="WALL",
            rotation=-90)}),                     Diagram(coordinateSystem(
            preserveAspectRatio=false)),
                Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>Heat transfer is modelled in the transversal, i.e., through the wall thickness, and vertical, i.e., through adjacent control volumes, directions.</p>
<p>The model accounts for anisotropic thermal conductivity in x and y directions. The remaining material properties must be isotropic.</p>
</html>"));
  end WallConductionVertical2D;

  model ColdPlateCircularChannelCV
    "Model of heat transfer and mass transfer through circular channel, in a rectangular solid surface"

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
      N=1,
      L=L,
      D=R_int*2,
      Roughness=Roughness)
      annotation (Placement(transformation(extent={{-28,-28},{28,28}})));

    CustomInterfaces.ZeroDimensional.FluidPort_A inlet(
      redeclare package Medium = Medium,
      m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
            m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-44,-4},
              {-36,4}},      rotation=0), iconTransformation(extent={{-40,-8},{-26,
              6}})));
    CustomInterfaces.ZeroDimensional.FluidPort_B outlet(
      redeclare package Medium = Medium,
      m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=-
            m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{36,-4},
              {44,4}},      rotation=0), iconTransformation(extent={{24,-8},{40,8}})));
    ConductionPlanoConcave2D PCWest(
      R=R_int,
      p=d/2 - R_int*d/sqrt(d*d + t*t),
      np=2*R_int*t/sqrt(d*d + t*t),
      dz=L,
      Tstart=T_start_solid,
      redeclare model Mat = Mat,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{-82,-22},{-38,22}})));
    ConductionPlanoConcave2D PCEast(
      R=R_int,
      p=d/2 - R_int*d/sqrt(d*d + t*t),
      np=2*R_int*t/sqrt(d*d + t*t),
      redeclare model Mat = Mat,
      dz=L,
      Tstart=T_start_solid,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{82,-22},{38,22}})));
    ConductionPlanoConcave2D PCSouth(
      R=R_int,
      p=t/2 - R_int*t/sqrt(d*d + t*t),
      np=2*R_int*d/sqrt(d*d + t*t),
      dz=L,
      Tstart=T_start_solid,
      redeclare model Mat = Mat,
      initOpt=initOpt) annotation (Placement(transformation(
          extent={{22,-26},{-22,26}},
          rotation=-90,
          origin={0,-60})));
    ConductionPlanoConcave2D PCNorth(
      R=R_int,
      p=t/2 - R_int*t/sqrt(d*d + t*t),
      np=2*R_int*d/sqrt(d*d + t*t),
      dz=L,
      Tstart=T_start_solid,
      redeclare model Mat = Mat,
      initOpt=initOpt) annotation (Placement(transformation(
          extent={{-20,-26},{20,26}},
          rotation=-90,
          origin={0,60})));
    WallConductionCV2D PlaneNW(
      y=t/2 - R_int*t/sqrt(d*d + t*t),
      x=d/2 - R_int*d/sqrt(d*d + t*t),
      z=L,
      dm=0,
      Q_int=0,
      Tstart=T_start_solid,
      redeclare model MatX = Mat,
      redeclare model MatY = Mat,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{-76,44},{-44,76}})));
    WallConductionCV2D PlaneSW(
      y=t/2 - R_int*t/sqrt(d*d + t*t),
      x=d/2 - R_int*d/sqrt(d*d + t*t),
      z=L,
      dm=0,
      Q_int=0,
      Tstart=T_start_solid,
      redeclare model MatX = Mat,
      redeclare model MatY = Mat,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{-76,-76},{-44,-44}})));
    WallConductionCV2D PlaneSE(
      y=t/2 - R_int*t/sqrt(d*d + t*t),
      x=d/2 - R_int*d/sqrt(d*d + t*t),
      z=L,
      dm=0,
      Q_int=0,
      Tstart=T_start_solid,
      redeclare model MatX = Mat,
      redeclare model MatY = Mat,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{44,-76},{76,-44}})));
    WallConductionCV2D PlaneNE(
      y=t/2 - R_int*t/sqrt(d*d + t*t),
      x=d/2 - R_int*d/sqrt(d*d + t*t),
      z=L,
      dm=0,
      Q_int=0,
      Tstart=T_start_solid,
      redeclare model MatX = Mat,
      redeclare model MatY = Mat,
      initOpt=initOpt)
      annotation (Placement(transformation(extent={{44,44},{76,76}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a NorthWest annotation (
        Placement(transformation(extent={{-110,50},{-90,70}}),
          iconTransformation(extent={{-72,40},{-60,52}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a West annotation (
        Placement(transformation(extent={{-110,-10},{-90,10}}),iconTransformation(
            extent={{-72,-6},{-60,6}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a SouthWest annotation (
        Placement(transformation(extent={{-110,-70},{-90,-50}}),
          iconTransformation(extent={{-72,-50},{-60,-38}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b South annotation (
        Placement(transformation(extent={{-10,-110},{10,-90}}),
          iconTransformation(extent={{-6,-72},{6,-60}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b SouthEast annotation (
        Placement(transformation(extent={{90,-70},{110,-50}}),
          iconTransformation(extent={{60,-50},{72,-38}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b East annotation (
        Placement(transformation(extent={{90,-10},{110,10}}),iconTransformation(
            extent={{60,-6},{72,6}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b NorthEast annotation (
        Placement(transformation(extent={{90,50},{110,70}}), iconTransformation(
            extent={{60,38},{72,50}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a North annotation (
        Placement(transformation(extent={{-10,90},{10,110}}),
          iconTransformation(extent={{-6,60},{6,72}})));

  equation
    connect(inlet, circularPipe.inlet)
      annotation (Line(points={{-40,0},{-28,0}}, color={0,0,0}));
    connect(circularPipe.outlet, outlet)
      annotation (Line(points={{28,0},{40,0}}, color={0,0,0}));
    connect(PCWest.InletConcave, circularPipe.thermalPort)
      annotation (Line(points={{-55.16,0},{-54,0},{-54,20},{0,20},{0,10.64}},
                                                              color={191,0,0}));
    connect(PCNorth.InletConcave, circularPipe.thermalPort)
      annotation (Line(points={{0,55.6},{0,10.64}},  color={191,0,0}));
    connect(PCEast.InletConcave, circularPipe.thermalPort)
      annotation (Line(points={{55.16,0},{55.16,20},{0,20},{0,10.64}},
                                                             color={191,0,0}));
    connect(PCWest.inletPlanar, PlaneNW.South) annotation (Line(points={{-60,
            19.8},{-60,48.8}},                          color={191,0,0}));
    connect(PlaneNW.East, PCNorth.outletPlanar) annotation (Line(points={{-45.6,
            60},{-23.4,60}},                   color={191,0,0}));
    connect(PlaneSW.East, PCSouth.outletPlanar) annotation (Line(points={{-45.6,
            -60},{-23.4,-60}},             color={191,0,0}));
    connect(PlaneSW.North, PCWest.outletPlanar) annotation (Line(points={{-60,
            -48.8},{-60,-19.8}},           color={191,0,0}));
    connect(PCSouth.inletPlanar, PlaneSE.West) annotation (Line(points={{23.4,
            -60},{45.6,-60}},              color={191,0,0}));
    connect(PlaneSE.North, PCEast.outletPlanar) annotation (Line(points={{60,
            -48.8},{60,-19.8}},                      color={191,0,0}));
    connect(PCEast.inletPlanar, PlaneNE.South) annotation (Line(points={{60,19.8},
            {60,48.8}},                              color={191,0,0}));
    connect(PCNorth.inletPlanar, PlaneNE.West) annotation (Line(points={{23.4,60},
            {45.6,60}},                     color={191,0,0}));
    connect(PlaneNW.West, NorthWest)
      annotation (Line(points={{-74.4,60},{-100,60}}, color={191,0,0}));
    connect(PlaneSW.West, SouthWest)
      annotation (Line(points={{-74.4,-60},{-100,-60}}, color={191,0,0}));
    connect(PCSouth.OutletOppoCon, South)
      annotation (Line(points={{0,-71},{0,-100}}, color={191,0,0}));
    connect(PlaneSE.East, SouthEast)
      annotation (Line(points={{74.4,-60},{100,-60}}, color={191,0,0}));
    connect(PCEast.OutletOppoCon, East)
      annotation (Line(points={{71,0},{100,0}},  color={191,0,0}));
    connect(West, PCWest.OutletOppoCon)
      annotation (Line(points={{-100,0},{-71,0}},  color={191,0,0}));
    connect(PlaneNE.East, NorthEast)
      annotation (Line(points={{74.4,60},{100,60}}, color={191,0,0}));
    connect(PCNorth.OutletOppoCon, North)
      annotation (Line(points={{0,70},{0,100}}, color={191,0,0}));
    connect(PlaneNW.North, North) annotation (Line(points={{-60,71.2},{-60,84},
            {0,84},{0,100}}, color={191,0,0}));
    connect(PlaneNE.North, North) annotation (Line(points={{60,71.2},{60,84},{0,
            84},{0,100}}, color={191,0,0}));
    connect(PlaneSW.South, South) annotation (Line(points={{-60,-71.2},{-60,-86},
            {0,-86},{0,-100}}, color={191,0,0}));
    connect(PlaneSE.South, South) annotation (Line(points={{60,-71.2},{60,-86},
            {0,-86},{0,-100}}, color={191,0,0}));
    connect(PCSouth.InletConcave, circularPipe.thermalPort)
      annotation (Line(points={{0,-55.16},{0,10.64}}, color={191,0,0}));
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
            preserveAspectRatio=false)),
      Documentation(info="<html>
<p><br>The model accounts for heat transfer through a circular channel, encapsulated in a rectangular solid material, and the heat transfer through its outer solid surface, accounting for heat transfer in over all the surfaces. </p>
<p><br><img src=\"modelica://DynTherM/Figures/CircularPipeCV.png\"/></p>
</html>"));
  end ColdPlateCircularChannelCV;

  model ColdPlateCircularChannel1D
    "Circular channel in a cold plate with 1D spatial discretization"

    replaceable model Mat = Materials.Aluminium constrainedby
      Materials.Properties "Material choice" annotation (choicesAllMatching=true);
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);

    model CV = TwoDimensional.ColdPlateCircularChannelCV "Control volume";
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

    CV cv[N_cv](
      redeclare model Mat = Mat,
      redeclare package Medium = Medium,
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

    DynTherM.CustomInterfaces.ZeroDimensional.FluidPort_A inlet(
      redeclare package Medium = Medium,
      m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0, start=
            m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{-106,
              -6},{-94,6}}, rotation=0), iconTransformation(extent={{-110,-10},{-90,
              10}})));
    DynTherM.CustomInterfaces.ZeroDimensional.FluidPort_B outlet(
      redeclare package Medium = Medium,
      m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0, start=-
            m_flow_start),
      P(start=P_start),
      h_outflow(start=Medium.specificEnthalpy(state_start)),
      Xi_outflow(start=X_start)) annotation (Placement(transformation(extent={{94,
              -6},{106,6}}, rotation=0), iconTransformation(extent={{90,-10},{110,
              10}})));

    CustomInterfaces.OneDimensional.HeatPort1D_B TopSurface(Nx=N_cv)
      annotation (Placement(transformation(extent={{8,14},{88,80}}),
          iconTransformation(extent={{8,14},{88,80}})));
    CustomInterfaces.OneDimensional.HeatPort1D_B BottomSurface(Nx=N_cv)
      annotation (Placement(transformation(extent={{8,-80},{88,-14}}),
          iconTransformation(extent={{8,-80},{88,-14}})));
    CustomInterfaces.TwoDimensional.HeatPort2D_A EastSide(Nx=N_cv, Ny=3)
      annotation (Placement(transformation(extent={{-82,-82},{-12,-12}}),
          iconTransformation(extent={{-82,-82},{-12,-12}})));
    CustomInterfaces.TwoDimensional.HeatPort2D_A WestSide(Nx=N_cv, Ny=3)
      annotation (Placement(transformation(extent={{-82,12},{-12,82}}),
          iconTransformation(extent={{-84,12},{-14,82}})));

  equation

    // thermal connections (Top,N and Bottom,S)
    for i in 1:N_cv loop
      connect(TopSurface.ports[i], cv[i].NorthTop);
      connect(BottomSurface.ports[i], cv[i].SouthBottom);
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
<p><span style=\"font-family: Courier New;\">Model created by stacking ColdPlateCircularChannelCV in series and adding SimplePlenum in between to improve solver robustness.</span></p>
</html>"));
  end ColdPlateCircularChannel1D;

  model PouchCellThermal2D "Thermal model of a pouch cell"

    replaceable model InPlaneMat = Materials.PolestarCellInPlane constrainedby
      Materials.Properties "In-plane material properties" annotation (choicesAllMatching=true);

    replaceable model CrossPlaneMat = Materials.PolestarCellCrossPlane constrainedby
      Materials.Properties "Cross-plane material properties" annotation (choicesAllMatching=true);

    model CV = WallConductionCV2D "Control volume";

    // Geometry
    parameter Length W "Width" annotation (Dialog(tab="Geometry"));
    parameter Length H "Height" annotation (Dialog(tab="Geometry"));
    parameter Length t "Thickness" annotation (Dialog(tab="Geometry"));
    input Mass dm(start=0) "Mass variation over time" annotation (Dialog(enable=true));

    // Initialization
    parameter Temperature Tstart "Temperature start value" annotation (Dialog(tab="Initialization"));
    parameter DynTherM.Choices.InitOpt initOpt  "Initialization option" annotation (Dialog(tab="Initialization"));

    // Discretization
    parameter Integer N(min=1) "Number of vertical sections in which the cell is discretized";

    CV cv[N](
      redeclare model MatX=CrossPlaneMat,
      redeclare model MatY=InPlaneMat,
      each x=t,
      each y=H/N,
      each z=W,
      each dm=dm/N,
      each Tstart=Tstart,
      each initOpt=initOpt);

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b Top annotation (Placement(
          transformation(extent={{-10,70},{10,90}}),iconTransformation(extent={{-10,
              54},{6,70}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b Bottom annotation (
        Placement(transformation(extent={{-10,-90},{10,-70}}),
                                                             iconTransformation(
            extent={{-10,-70},{6,-54}})));
    CustomInterfaces.OneDimensional.HeatPort1D_B Left(Nx=N) annotation (
        Placement(transformation(
          extent={{-10,-5},{10,5}},
          rotation=90,
          origin={-41,0}), iconTransformation(
          extent={{-40,-13},{40,13}},
          rotation=90,
          origin={-83,0})));
    CustomInterfaces.OneDimensional.HeatPort1D_B Right(Nx=N) annotation (
        Placement(transformation(
          extent={{-10,-5},{10,5}},
          rotation=90,
          origin={41,0}), iconTransformation(
          extent={{-40,-13},{40,13}},
          rotation=90,
          origin={59,0})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a Average annotation (
        Placement(transformation(extent={{-10,-8},{6,8}}), iconTransformation(
            extent={{-10,-8},{6,8}})));

  equation
    sum(cv.T_vol)/N = Average.T;

    // Port connections
    for i in 1:N loop
      connect(cv[i].West, Left.ports[i]);
      connect(cv[i].East, Right.ports[i]);
      cv[i].Q_int = Average.Q_flow/N;
    end for;

    // Internal connections
    for i in 1:(N-1) loop
      connect(cv[i].North, cv[i+1].South);
    end for;

    // Boundary connections
    connect(cv[1].South, Bottom);
    connect(cv[end].North, Top);

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Bitmap(
            extent={{-88,-88},{88,88}},
            fileName="modelica://DynTherM/Figures/PouchCell.PNG",
            origin={6,0},
            rotation=-90),
          Line(
            points={{0,24},{0,-100}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={-52,-26},
            rotation=90),
          Line(
            points={{0,24},{0,-100}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={-52,0},
            rotation=90),
          Line(
            points={{0,24},{0,-100}},
            color={0,0,0},
            pattern=LinePattern.Dash,
            origin={-52,26},
            rotation=90)}),                      Diagram(coordinateSystem(
            preserveAspectRatio=false)),
                Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>Heat generation is assumed uniform over the vertical control volumes.</p>
<p>Heat conduction is modelled across the cell height (in-plane) with 1D discretization and thickness (cross-plane) with 0D discretization.</p>
<p>Heat conduction across the cell width is disregarded.</p>
<p><br><img src=\"modelica://DynTherM/Figures/PouchCellThermal2D.png\"/></p>
</html>"));
  end PouchCellThermal2D;

  model ExternalConvection2D
    "External convection model implementing 2D spatial discretization"

    outer Components.Environment environment "Environmental properties";
    replaceable package Medium = Modelica.Media.Air.MoistAir constrainedby
      Modelica.Media.Interfaces.PartialMedium "Medium model" annotation(choicesAllMatching = true);
    model CV = DynTherM.Components.HeatTransfer.ExternalConvection "Control volume";

    replaceable model HTC =
      DynTherM.Components.HeatTransfer.HTCorrelations.ExternalConvection.FixedValue
      constrainedby
      DynTherM.Components.HeatTransfer.HTCorrelations.BaseClasses.BaseClassExternal
      annotation (choicesAllMatching=true);

    input Area A "Heat transfer area (total)" annotation (Dialog(enable=true));

    // Discretization
    parameter Integer Nx(min=1) "Number of control volumes in x-direction";
    parameter Integer Ny(min=1) "Number of control volumes in y-direction";

    CV cv[Nx,Ny](
      redeclare model HTC = HTC,
      redeclare package Medium = Medium,
      each A=A_cv);

    Area A_cv "Heat transfer area associated with one control volume";

    CustomInterfaces.TwoDimensional.HeatPort2D_A inlet(Nx=Nx, Ny=Ny)
      annotation (Placement(transformation(extent={{-60,-20},{60,80}}),
          iconTransformation(extent={{-60,-20},{60,80}})));

  equation
    A_cv = A/(Nx*Ny);

    for i in 1:Nx loop
      for j in 1:Ny loop
        connect(cv[i,j].inlet, inlet.ports[i,j]);
      end for;
    end for;

    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
            extent={{-100,20},{100,0}},
            fillColor={192,192,192},
            fillPattern=FillPattern.Backward),
          Line(points={{90,-10},{-90,-10}}, color={0,127,255}),
          Line(points={{78,-16},{90,-10}},   color={0,127,255}),
          Line(points={{90,-30},{-90,-30}}, color={0,127,255}),
          Line(points={{78,-4},{90,-10}},    color={0,127,255}),
          Line(points={{78,-36},{90,-30}},   color={0,127,255}),
          Line(points={{78,-24},{90,-30}},   color={0,127,255})}),
                                                 Diagram(coordinateSystem(
            preserveAspectRatio=false)));
  end ExternalConvection2D;
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
