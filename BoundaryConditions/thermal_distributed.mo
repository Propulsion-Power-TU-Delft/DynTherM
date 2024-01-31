within DynTherM.BoundaryConditions;
model thermal_distributed
  "Model to impose distributed heat flow rate and temperature"

  parameter Integer Nx(min=1) "Number of ports in x-direction";
  parameter Integer Ny(min=1) "Number of ports in y-direction";
  parameter Temperature T[Nx,Ny]=273.15*ones(Nx,Ny) "Temperature" annotation (Dialog(tab="Boundary conditions", enable=use_di_T));
  parameter HeatFlowRate Q[Nx,Ny]=1e3*ones(Nx,Ny) "Heat flow rate" annotation (Dialog(tab="Boundary conditions", enable=use_di_Q));
  parameter Boolean use_di_Q = true "True if heat flow rate is given as parameter" annotation (Dialog(tab="Boundary conditions"), choices(checkBox=true));
  parameter Boolean use_di_T = false "True if temperature is given as parameter" annotation (Dialog(tab="Boundary conditions"), choices(checkBox=true));
  parameter Boolean use_in_Q = false "True if heat flow rate is given as input" annotation (Dialog(tab="Boundary conditions"), choices(checkBox=true));
  parameter Boolean use_in_T = false "True if temperature is given as input" annotation (Dialog(tab="Boundary conditions"), choices(checkBox=true));

  CustomInterfaces.DistributedHeatPort_A thermal(Nx=Nx, Ny=Ny) annotation (Placement(
         transformation(extent={{74,-16},{106,16}}), iconTransformation(extent={{
             74,-16},{106,16}})));
  Modelica.Blocks.Interfaces.RealInput in_Q if use_in_Q annotation (
     Placement(transformation(
       origin={98,14},
       extent={{-10,-10},{10,10}},
       rotation=270), iconTransformation(
       extent={{-4,-4},{4,4}},
       rotation=270,
       origin={80,8})));
  Modelica.Blocks.Interfaces.RealInput in_T if use_in_T annotation (Placement(
       transformation(
       origin={82,14},
       extent={{10,-10},{-10,10}},
       rotation=90), iconTransformation(
       extent={{4,-4},{-4,4}},
       rotation=90,
       origin={100,8})));

protected
  Modelica.Blocks.Interfaces.RealInput in_Q_internal;
  Modelica.Blocks.Interfaces.RealInput in_T_internal;

equation
  //Boundary equations
  if use_di_T then
    thermal.ports.T = T;
  elseif use_in_T then
    thermal.ports.T = in_T_internal*ones(Nx,Ny);
  end if;

  if use_di_Q then
    thermal.ports.Q_flow + Q = zeros(Nx,Ny);
  elseif use_in_Q then
    thermal.ports.Q_flow + in_Q_internal/(Nx*Ny)*ones(Nx,Ny) = zeros(Nx,Ny);
  end if;

  // Connect protected connectors to public conditional connectors
  connect(in_Q, in_Q_internal);
  connect(in_T, in_T_internal);


  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{60,-20},
            {120,20}})),                  Icon(coordinateSystem(
          preserveAspectRatio=false, extent={{60,-20},{120,20}}),     graphics={Text(
          extent={{86,8},{94,0}},
          lineColor={238,46,47},
          textString="thermal distributed")}));
end thermal_distributed;
