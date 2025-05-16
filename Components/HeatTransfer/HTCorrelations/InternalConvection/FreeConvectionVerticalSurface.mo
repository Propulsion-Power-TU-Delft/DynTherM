within DynTherM.Components.HeatTransfer.HTCorrelations.InternalConvection;
model FreeConvectionVerticalSurface
  "Free convection in a vertical rectangular cavity"
  extends BaseClasses.BaseClassFreeConvectionInternal;
  input Real AR "Aspect ratio (height/width)" annotation (Dialog(enable=true));

equation
  if AR <= 2 then
    Nu = 0.18*(Pr/(0.2 + Pr)*Ra)^0.29;

  elseif (AR > 2) and (AR <= 10) then
    Nu = 0.22*((Pr/(0.2 + Pr)*Ra)^0.28)*AR^(-1/4);

  else
    if Ra <= 1e7 then
      Nu = 0.42*(Ra^(1/4))*(Pr^0.012)*(AR^(-0.3));
    else
      Nu = 0.046*Ra^(1/3);
    end if;
  end if;

  annotation (Documentation(info="<html>
<h4>Reference:</h4>
<p>[1] Incropera, F. P., De Witt, D. P., Bergan, T. L., Lavine, A. S. &quot;Fundamentals of Heat and Mass Transfer&quot;, 2006, 6th ed., John Wiley &amp; Sons, p. 589.</p>
</html>"));
end FreeConvectionVerticalSurface;
