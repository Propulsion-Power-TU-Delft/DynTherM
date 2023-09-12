within ThermalManagement.CustomInterfaces;
connector Shaft_A "A-type mechanical connector"
  extends MechanicalPort;
  annotation (Icon(graphics={                                                                                     Polygon(
          points={{-100,40},{-40,100},{40,100},{100,40},{100,-40},{40,
              -100},{-40,-100},{-100,-40},{-100,40}},
          lineColor={135,135,135},
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid)}));
end Shaft_A;
