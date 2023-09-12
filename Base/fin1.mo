within ThermalManagement.Base;
partial model fin1 "One flow inlet"
  CustomInterfaces.FluidPort_A flowState
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end fin1;
